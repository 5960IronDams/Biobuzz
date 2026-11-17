package org.firstinspires.ftc.teamcode.Subsystems.turret;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import java.util.OptionalDouble;

/**
 * Reads two REV Through Bore absolute PWM encoders on an OctoQuad in a single I2C bulk read:
 * primary tracking encoder B (32T) via {@link #readRevs()} and CRT partner A (31T) via {@link #readCrossRevs()}.
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

    /** Time (s) at which each PWM channel's raw pulse width last changed (~10 Hz refresh). */
    private volatile double lastChangeTimeB = Double.NaN;
    private volatile double lastChangeTimeA = Double.NaN;
    private volatile double lastAgeB = Double.NaN;
    private volatile double lastAgeA = Double.NaN;
    private final long startNs = System.nanoTime();

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
        // Only reset & write flash when the chip's stored config differs from what we want.
        // Avoids unnecessary EEPROM wear across OpMode restarts.
        boolean needsFlash;
        try {
            OctoQuad.ChannelBankConfig bank = octoquad.getChannelBankConfig();
            OctoQuad.ChannelPulseWidthParams chParams = octoquad.getSingleChannelPulseWidthParams(channel);
            boolean chWrap = octoquad.getSingleChannelPulseWidthTracksWrap(channel);
            boolean ok = bank == OCTOQUAD_BANK_CONFIG
                    && chParams != null
                    && chParams.min_length_us == ABS_PWM_MIN_US
                    && chParams.max_length_us == ABS_PWM_MAX_US
                    && !chWrap;
            if (ok && crossCh >= 0 && crossCh != channel) {
                OctoQuad.ChannelPulseWidthParams crossParams =
                        octoquad.getSingleChannelPulseWidthParams(crossCh);
                boolean crossWrap = octoquad.getSingleChannelPulseWidthTracksWrap(crossCh);
                ok = crossParams != null
                        && crossParams.min_length_us == ABS_PWM_MIN_US
                        && crossParams.max_length_us == ABS_PWM_MAX_US
                        && !crossWrap;
            }
            needsFlash = !ok;
        } catch (Exception ignored) {
            needsFlash = true;
        }
        if (needsFlash) {
            octoquad.resetEverything();
            octoquad.setChannelBankConfig(OCTOQUAD_BANK_CONFIG);
            octoquad.setSingleChannelPulseWidthParams(channel,
                    new OctoQuad.ChannelPulseWidthParams(ABS_PWM_MIN_US, ABS_PWM_MAX_US));
            if (crossCh >= 0 && crossCh != channel) {
                octoquad.setSingleChannelPulseWidthParams(crossCh,
                        new OctoQuad.ChannelPulseWidthParams(ABS_PWM_MIN_US, ABS_PWM_MAX_US));
            }
            octoquad.saveParametersToFlash();
        }

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
            double nowS = (System.nanoTime() - startNs) / 1.0e9;
            // Freshness: stamp the moment B's raw width last CHANGED. Stored
            // raw for telemetry/calibration either way.
            if (us != lastRawUs) {
                lastChangeTimeB = nowS;
            }
            lastRawUs = us;
            lastAgeB = Double.isNaN(lastChangeTimeB) ? Double.NaN : nowS - lastChangeTimeB;
            // Range check: reject pulse widths outside [MIN - SLACK, MAX + SLACK] as glitches.
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
                // Freshness: stamp the moment A's raw width last CHANGED.
                if (usA != crossRawUs) {
                    lastChangeTimeA = nowS;
                }
                crossRawUs = usA;
                lastAgeA = Double.isNaN(lastChangeTimeA) ? Double.NaN : nowS - lastChangeTimeA;
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

    /**
     * Seconds since encoder A's raw pulse width last changed (NaN until first change).
     * Because PWM refreshes at ~10 Hz, an age >= 100 ms indicates normal refresh staleness.
     */
    public double getCrossFreshnessAgeSec() {
        return lastAgeA;
    }

    /** Same as {@link #getCrossFreshnessAgeSec()} for the primary (tracking) channel B. */
    public double getFreshnessAgeSec() {
        return lastAgeB;
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
