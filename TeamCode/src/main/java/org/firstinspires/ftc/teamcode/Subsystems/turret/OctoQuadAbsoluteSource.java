package org.firstinspires.ftc.teamcode.Subsystems.turret;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import java.util.OptionalDouble;

/**
 * OctoQuadAbsoluteSource — the turret's production absolute angle source: an
 * OctoQuad reading a REV Through Bore encoder's absolute PWM output
 * (1–1024 us = 0–360 deg) on a pulse-width channel.
 *
 * <p>This robot reads TWO Through Bore encoders on one chip: the TRACKING
 * encoder (encoder B, 32T pinion) and the CRT partner (encoder A, 31T pinion).
 * ONE bulk read per loop serves both channels — the primary value is returned
 * by {@link #readRevs()}, the partner's raw fraction is cached and served by
 * {@link #readCrossRevs()} with zero extra I2C traffic.
 *
 * <p><b>Hardening (session 4, the "wild heading" capture):</b> the raw value
 * is RANGE-VALIDATED against the configured pulse window BEFORE it leaves this
 * class — an out-of-window reading (wrong wiring, unseated cable, wrong
 * encoder) returns empty and counts in {@link #oorEvents} instead of being
 * mod-1'd into a plausible-looking angle. The raw microseconds are exposed via
 * {@link #getLastRawUs()} / {@link #getCrossRawUs()} and the applied chip
 * configuration is echoed via {@link #getConfigEcho()} so bench bring-up can
 * prove the chip is in the intended mode.
 *
 * <p><b>Channel banking:</b> the OctoQuad only banks channels as
 * ALL-quadrature, ALL-pulse-width, or bank1(0-3)-quadrature /
 * bank2(4-7)-pulse-width. The two ABS wires sit on channels 6 and 7, so the
 * chip is banked BANK1_QUADRATURE_BANK2_PULSE_WIDTH via
 * {@link #OCTOQUAD_BANK_CONFIG} — change both together if you rewire.
 */
@Configurable
public class OctoQuadAbsoluteSource implements AbsoluteAngleSource {

    // ---- Wiring + PWM window config (Panels -> OctoQuadAbsoluteSource) ----

    /** OctoQuad channel carrying the TRACKING encoder's (B, 32T pinion) ABS PWM wire. */
    public static int TURRET_OCTOQUAD_CHANNEL = 6;

    /** OctoQuad channel carrying encoder A's (31T pinion, CRT partner) ABS PWM wire. */
    public static int CRT_ENCODER_A_OCTOQUAD_CHANNEL = 7;

    /**
     * Channel banking applied at init (re-applied only on OpMode restart):
     * 0-3 stay quadrature for future encoders, 4-7 are pulse-width.
     */
    public static OctoQuad.ChannelBankConfig OCTOQUAD_BANK_CONFIG =
            OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH;

    /** Through Bore absolute PWM range, microseconds (1-1024 us = 0-360 deg). */
    public static int ABS_PWM_MIN_US = 1;
    public static int ABS_PWM_MAX_US = 1024;

    /**
     * Slack on the PWM window validation: readings outside
     * [min - slack, max + slack] are counted as out-of-range glitches and
     * dropped instead of being decoded.
     */
    public static int ABS_PWM_OOR_SLACK_US = 16;

    /**
     * Software sign flip for the TRACKING encoder (encoder B): CCW turret
     * motion must read POSITIVE. Flipping this flag only mirrors the rotation
     * direction — the CRT zero offsets are RAW-frame (see TurretCrt) and stay
     * valid without recalibration.
     */
    public static boolean ENCODER_REVERSED = true;

    private final OctoQuad octoquad;
    private final int channel;
    /** CRT partner channel (encoder A); -1 when this source is single-encoder. */
    private final int crossChannel;
    private final OctoQuad.EncoderDataBlock block = new OctoQuad.EncoderDataBlock();
    private final String fw;
    private final String configEcho;

    /** Diagnostic counters (read by telemetry / the debug OpMode). */
    public volatile int oorEvents = 0;
    public volatile int crcFailEvents = 0;
    /** Same counters for the CRT partner channel (encoder A). */
    public volatile int oorEventsA = 0;
    public volatile int crcFailEventsA = 0;
    private volatile int lastRawUs = Integer.MIN_VALUE;
    private volatile boolean lastReadValid = false;
    /** Cross-channel (encoder A) state from the SAME bulk read. */
    private volatile int crossRawUs = Integer.MIN_VALUE;
    private volatile boolean crossReadValid = false;

    public OctoQuadAbsoluteSource(OctoQuad octoquad) {
        this(octoquad, CRT_ENCODER_A_OCTOQUAD_CHANNEL);
    }

    public OctoQuadAbsoluteSource(OctoQuad octoquad, int crossCh) {
        this.octoquad = octoquad;
        this.channel = TURRET_OCTOQUAD_CHANNEL;
        this.crossChannel = crossCh;
        String v;
        try {
            v = octoquad.getFirmwareVersionString();
        } catch (Exception e) {
            v = "unknown";
        }
        fw = v;
        // Clean slate + production config, persisted through power glitches
        // (same pattern as the SDK OctoQuad sample). NOTE: resetEverything()
        // wipes settings for ALL channels — this must stay the only user of
        // this OctoQuad, or the other users must re-apply their config too.
        octoquad.resetEverything();
        octoquad.setChannelBankConfig(OCTOQUAD_BANK_CONFIG);
        octoquad.setSingleChannelPulseWidthParams(channel,
                new OctoQuad.ChannelPulseWidthParams(ABS_PWM_MIN_US, ABS_PWM_MAX_US));
        if (crossCh >= 0 && crossCh != channel) {
            octoquad.setSingleChannelPulseWidthParams(crossCh,
                    new OctoQuad.ChannelPulseWidthParams(ABS_PWM_MIN_US, ABS_PWM_MAX_US));
        }
        octoquad.saveParametersToFlash();

        // Echo back what the chip ACTUALLY has (flash/config could have
        // failed silently) — a mismatch here explains "wild readings".
        String echo;
        try {
            OctoQuad.ChannelPulseWidthParams got =
                    octoquad.getSingleChannelPulseWidthParams(channel);
            echo = OCTOQUAD_BANK_CONFIG.name() + ", ch" + channel
                    + " PWM " + got.min_length_us + "-" + got.max_length_us
                    + "us, tracksWrap=" + octoquad.getSingleChannelPulseWidthTracksWrap(channel)
                    + (crossCh >= 0 && crossCh != channel
                            ? " + cross ch" + crossCh + " PWM" : "");
        } catch (Exception e) {
            echo = "config echo FAILED: " + e;
        }
        configEcho = echo;
        Log.i("IronLog", "OctoQuadAbsoluteSource ready: fw " + fw + ", " + echo);
    }

    /**
     * ONE bulk read serves both encoder channels: the primary (tracking)
     * channel is returned directly, the CRT partner's data is cached and
     * served by {@link #readCrossRevs()} — zero extra I2C traffic.
     */
    @Override
    public OptionalDouble readRevs() {
        lastReadValid = false;
        crossReadValid = false;
        try {
            octoquad.readAllEncoderData(block);
            if (!block.isDataValid()) {
                crcFailEvents++;
                if (crossChannel >= 0) {
                    crcFailEventsA++;
                }
                return OptionalDouble.empty(); // CRC/bulk-read failure this loop
            }
            // positions[] holds pulse width in microseconds for PWM channels
            // (verified against the SDK SensorOctoQuadAdv sample).
            int us = block.positions[channel];
            lastRawUs = us;
            // RANGE VALIDATION — the mod1() that used to sit here masked
            // garbage into plausible angles. A real Through Bore PWM read
            // lives in [min, max]; anything else is a wiring/config fault.
            if (us < ABS_PWM_MIN_US - ABS_PWM_OOR_SLACK_US || us > ABS_PWM_MAX_US + ABS_PWM_OOR_SLACK_US) {
                oorEvents++;
                return OptionalDouble.empty();
            }
            lastReadValid = true;
            double revs = us / (double) ABS_PWM_MAX_US;
            if (ENCODER_REVERSED) {
                revs = 1.0 - revs;
            }
            // Cross channel (encoder A): decode from the SAME transaction.
            if (crossChannel >= 0 && crossChannel != channel) {
                int usA = block.positions[crossChannel];
                crossRawUs = usA;
                if (usA >= ABS_PWM_MIN_US - ABS_PWM_OOR_SLACK_US
                        && usA <= ABS_PWM_MAX_US + ABS_PWM_OOR_SLACK_US) {
                    // (Encoder A stays RAW here — readCrossRevs() serves the
                    // raw fraction; the solve-frame flip lives in
                    // TurretCrt.solveFrameRevs.)
                    crossReadValid = true;
                    return OptionalDouble.of(revs); // primary value; cross via readCrossRevs()
                }
                oorEventsA++;
                return OptionalDouble.of(revs);
            }
            return OptionalDouble.of(revs);
        } catch (Exception e) {
            return OptionalDouble.empty();
        }
    }

    /**
     * The CRT partner's (encoder A) RAW fractional revolution from the same
     * bulk transaction {@link #readRevs()} just did. Empty when the source is
     * single-encoder, the CRC failed, or encoder A was out-of-window. Never
     * reads I2C on its own — always pair with a fresh {@link #readRevs()}.
     */
    @Override
    public OptionalDouble readCrossRevs() {
        if (!crossReadValid) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(crossRawUs / (double) ABS_PWM_MAX_US);
    }

    /**
     * The primary (tracking) channel's RAW fractional revolution —
     * PRE-{@code ENCODER_REVERSED}, exactly what the calibration dashboards
     * show — from the last bulk transaction {@link #readRevs()} did. The boot
     * solves consume this so the zero offsets stay raw-frame (valid under any
     * direction flags). Empty when the last read failed CRC or the pulse-window
     * check.
     */
    public OptionalDouble readPrimaryRawRevs() {
        if (!lastReadValid) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(lastRawUs / (double) ABS_PWM_MAX_US);
    }

    /** Last raw pulse width in microseconds (Integer.MIN_VALUE before the first read). */
    public int getLastRawUs() {
        return lastRawUs;
    }

    /** Encoder A's last raw pulse width (same transaction). */
    public int getCrossRawUs() {
        return crossRawUs;
    }

    /** True when the most recent read was CRC-valid AND in the pulse window. */
    public boolean isLastReadValid() {
        return lastReadValid;
    }

    /** Encoder A's most recent read validity (same transaction). */
    public boolean isCrossReadValid() {
        return crossReadValid;
    }

    /** What the chip actually has configured (bank, pulse window, wrap tracking). */
    public String getConfigEcho() {
        return configEcho;
    }

    @Override
    public String describe() {
        return "OctoQuad ch" + channel + " PWM abs (fw " + fw + ")"
                + (crossChannel >= 0 && crossChannel != channel
                        ? " + CRT ch" + crossChannel : "");
    }
}
