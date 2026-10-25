package org.firstinspires.ftc.teamcode.Subsystems.turret;

import static org.firstinspires.ftc.teamcode.WattageLib.lib.killaUtils.clamp;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Locale;
import java.util.OptionalDouble;

/**
 * TurretTracker — figures out WHERE THE TURRET IS. It fuses three encoders
 * into one continuous position estimate and hands it to {@link TurretPivot},
 * which does everything else (setpoints, PID, driving). Splitting it this way
 * keeps the "sensing" half and the "acting" half of the turret in separate,
 * readable files.
 *
 * <p><b>The three sensors and what each is good at:</b>
 * <ol>
 *   <li><b>Motor's built-in encoder</b> (the 5203's quadrature, counted in
 *       hardware between loops): EXACT motion every loop at any speed — but
 *       it only knows deltas, so any tiny error accumulates forever.</li>
 *   <li><b>Encoder B</b> (32T pinion, REV Through Bore absolute PWM via the
 *       OctoQuad): knows the ABSOLUTE angle, never drifts — but the PWM train
 *       only refreshes at ~10 Hz, so per-loop it is stale and quantized.</li>
 *   <li><b>Encoder A</b> (31T pinion, same bus): a second absolute opinion
 *       used to (a) solve the true multi-turn angle at boot together with B
 *       (the CRT solve in {@link TurretCrt}) and (b) cross-check every loop
 *       that nothing has slipped.</li>
 * </ol>
 *
 * <p><b>The hybrid state machine (how 1 and 2 are combined):</b>
 * <ul>
 *   <li>{@code WATCH} — PWM owns tracking while the motor encoder proves its
 *       direction at the first powered motion (>= 6 deg of agreed motion).</li>
 *   <li>{@code ACTIVE} — the motor encoder integrates motion every loop; the
 *       PWM absolute hard-snaps the position whenever the turret is AT REST
 *       (erases accumulated drift right before a shot) and is only recorded
 *       (never applied) while moving, because a fresh PWM reading can be
 *       ~100 ms stale at speed.</li>
 *   <li>{@code OFF} — encoder-B PWM-only tracking with a reduced power cap:
 *       the fallback after any fault, and the mode you get with
 *       {@code MOTOR_ENC_HYBRID = false}.</li>
 * </ul>
 *
 * <p>This class never drives anything — it reads encoders and reports. Power,
 * brakes and fault REACTIONS stay in TurretPivot.
 */
@Configurable
public class TurretTracker {

    // =========================================================================
    // Tuning (Panels -> TurretTracker)
    // =========================================================================

    /**
     * Median window for the raw absolute stream before unwrapping. 3 = a
     * single garbage read between two good ones is dropped entirely (one-tick
     * lag on real motion). 1 = off. NOTE (documented limitation): the median
     * runs on WRAPPED fractional revs, so it is not seam-aware — during very
     * fast seam crossings a wrong-branch value can pass; the implausibility
     * gate + the hybrid's wrap-resolved anchor are the backstops.
     */
    public static int ABS_MEDIAN_WINDOW = 3;

    /**
     * Implausibility gate: a per-loop raw step bigger than this speed's step
     * (times dt) is treated as a glitch — NOT integrated, counted, and the
     * tracker waits for the next good read. Sized above the real mechanism:
     * motor no-load at full power plus a hand-spin margin.
     */
    public static double ABS_MAX_PLAUSIBLE_DPS = 2000.0;

    /** Consecutive implausible steps before re-anchoring to the raw reading anyway. */
    public static int ABS_REJECT_REANCHOR = 8;

    /** Boot stability check: number of reads (12 ms apart) sampled at init. */
    public static int ABS_BOOT_READS = 8;

    /** Boot stability check: allowed spread across those reads (deg). */
    public static double ABS_BOOT_SPREAD_DEG = 3.0;

    /** Velocity EMA weight (new-sample fraction) — tames quantization into the D term. 1.0 = off. */
    public static double VELOCITY_EMA_ALPHA = 0.7;

    /** After this long with no fresh PWM change, treat the (held) velocity as 0. */
    public static double ABS_VELOCITY_STALE_S = 0.30;

    // ---- Hybrid motor-encoder tracking ----

    /** 5203 built-in encoder counts per MOTOR-shaft revolution (before the gearbox). */
    public static double MOTOR_ENC_TICKS_PER_MOTOR_REV = 28.0;
    /** 5203 planetary gearbox ratio (19.2:1 = 3591/187). */
    public static double MOTOR_PLANETARY_RATIO = 3591.0 / 187.0;

    /**
     * Master switch for the hybrid tracker. true = hybrid (motor integrates
     * per-loop, PWM absolutes anchor + cross-check). false = the encoder-B
     * PWM-only tracker runs permanently (the fallback mode — also what the
     * hybrid demotes to on any fault).
     */
    public static boolean MOTOR_ENC_HYBRID = true;

    /**
     * Motor encoder polarity: +1 when positive tick delta = CCW turret motion.
     * Bench-verified (capture 022708); flip if the boot direction guard or
     * watch mode latches MOTOR_ENC_DIR.
     */
    public static double MOTOR_ENC_POLARITY = 1.0;

    /** Anchor correction larger than this (deg) is rejected as absurd (recorded, not applied). */
    public static double MOTOR_ENC_ANCHOR_REJECT_DEG = 25.0;
    /** At-rest hard re-anchor runs while the turret is slower than this (deg/s). */
    public static double MOTOR_ENC_REST_ANCHOR_VEL_DPS = 8.0;
    /** Watch promotion: the PWM-tracked turret must move at least this much (deg) ... */
    public static double MOTOR_ENC_CONFIRM_PWM_DEG = 6.0;
    /** ...while the motor encoder integrated at least this much (deg), same sign. */
    public static double MOTOR_ENC_CONFIRM_MOT_DEG = 3.0;
    /** Moving-monitor dead test: a PWM interval this large (deg) counts as evidence ... */
    public static double MOTOR_ENC_DEAD_PWM_DEG = 5.0;
    /** ...against the motor encoder integrating less than this (deg) over the same span. */
    public static double MOTOR_ENC_DEAD_MOT_DEG = 1.0;
    /** Consecutive dead/direction evidences before demoting the hybrid. */
    public static int MOTOR_ENC_STREAK_LIMIT = 2;
    /** Consecutive motor-encoder READ failures before demoting the hybrid. */
    public static int MOTOR_ENC_READ_FAIL_LIMIT = 3;

    /** Consecutive failed feedback reads tolerated before "feedback lost" (I2C hiccups happen). */
    public static int FEEDBACK_FAIL_LIMIT = 10;

    /**
     * Consecutive loops with NO absolute (OctoQuad/encoder-B) read before the
     * ABS_STREAM_LOST fault latches onto the driver-station failure board.
     * This is deliberately NOT {@link #feedbackLost()}: in hybrid ACTIVE the
     * motor encoder keeps tracking (no brake), but the absolute path — the
     * boot-correct position, the at-rest anchors, the A cross-check — is gone
     * and the driver must know. ~50 loops ≈ 2.5-3 s at the measured 49-54 ms
     * loop. Auto-clears after the same count of consecutive GOOD reads (a
     * re-seated wire recovers without an OpMode restart).
     */
    public static int ABS_STREAM_LOST_LOOPS = 50;

    // =========================================================================
    // Live state
    // =========================================================================

    private final AbsoluteAngleSource absoluteSource;
    private final DcMotorEx motor;
    /** Encoder-B revolutions per ONE turret revolution (5.5). */
    private final double revsPerTurretB;
    /** Encoder-A revolutions per ONE turret revolution (176/31 ≈ 5.677). */
    private final double revsPerTurretA;
    /** MOTOR gear ratio (2.75) — scales the motor encoder's ticks. */
    private final double motorGearRatio;

    /**
     * Continuous tracked TURRET angle since boot-zero, in DEGREES (+CCW,
     * multi-turn — survives seam crossings and counter resets). This is the
     * stream every feedback path integrates into.
     */
    private double unwrappedDeg = 0;
    /** Soft zero: subtracted from unwrappedDeg so position() reads the boot angle at the zero moment. */
    private double zeroOffsetDeg = 0;

    /** Last raw absolute reading (encoder revolutions [0,1)) — anchored at boot, unwrapped every loop. */
    private double lastAbsRevs = Double.NaN;
    private boolean absPrimed = false;
    /** Latest raw absolute reading in degrees (NaN when unavailable) — calibration + telemetry. */
    private double lastAbsRawDeg = Double.NaN;
    /** Median-filter history + implausibility gate state. */
    private double absHist0 = Double.NaN;
    private double absHist1 = Double.NaN;
    private int consecutiveRejects = 0;
    /** Timer seconds of the last loop where the raw width actually CHANGED (~10 Hz refresh). */
    private double absLastChangeTime = Double.NaN;
    /** Implausible steps rejected by the gate (not integrated). Large + growing = wiring/config problem. */
    private int absGlitchEvents = 0;
    /** Light EMA on the measured velocity — tames PWM step quantization into the D term (1.0 = off). */
    private double velEmaDegPerSec = 0;

    /** Measured turret velocity (deg/s, +CCW), self-computed from the feedback streams. */
    private double velocityDegPerSec = 0;

    // ---- boot verification state ----
    private boolean bootEncoderVerified = false;
    private double bootCheckSig = Double.NaN;
    private boolean bootDirChecked = false;
    private boolean bootDirMismatch = false;
    private double bootDirSumBRevs = 0.0;
    private double bootDirSumARevs = 0.0;
    private double bootDirLastB = Double.NaN;
    private double bootDirLastA = Double.NaN;
    /** Latched: hand-motion direction of the flipped tracking encoder disagrees with encoder A / the motor. */
    private boolean faultEncoderDirection = false;
    /** Latched: encoder never produced motion during boot verification window. */
    private boolean faultEncoderDeadAtBoot = false;
    /** Wrap/glitch absorption events since boot (expect ~0; nonzero = investigate). */
    private int wrapEvents = 0;

    // ---- CRT cross-check state ----
    /** Latched: the every-loop encoder-A cross-check failed more than CRT_CROSSCHECK_FAIL_LIMIT times. */
    private boolean faultCrtCrosscheck = false;
    /** Consecutive cross-check failures (reset on a pass). */
    private int crtCrosscheckFails = 0;
    /** Last cross-check error in TURRET degrees (NaN = not checked yet / skipped). */
    private double crtCrosscheckErrDeg = Double.NaN;

    // ---- feedback health ----
    /** Consecutive failed feedback reads (tolerates single I2C hiccups; brakes at the limit). */
    private int feedbackFailures = 0;
    /** Which feedback path is actually in use (for telemetry). */
    private final String feedbackSourceName;

    // ---- hybrid motor-encoder state ----
    /**
     * Hybrid tracker operating mode. ACTIVE = motor encoder owns per-loop
     * motion. WATCH = PWM owns tracking while the motor encoder proves its
     * direction at the first powered motion. OFF = PWM-only tracking (the
     * encoder-B fallback — also the demotion target for any hybrid fault).
     */
    private enum MotEncMode { OFF, WATCH, ACTIVE }
    private MotEncMode motorEncMode = MotEncMode.OFF;
    /** Why the hybrid is OFF (telemetry; empty for WATCH/ACTIVE). */
    private String motorEncOffReason = "";
    /** False until the first motor-encoder read after boot/re-zero (delta baseline). */
    private boolean motorEncPrimed = false;
    private int motorEncLastPos = 0;
    /** Motor-encoder implausible steps rejected (counter glitch / mode stomp). */
    private int motorEncGlitchEvents = 0;
    private int motorEncConsecGlitches = 0;
    private int motorEncReadFails = 0;
    /** Watch mode: PWM-tracked position when watch began (NaN = not started) + motor net motion. */
    private double watchStartPos = Double.NaN;
    private double watchMotNetDeg = 0;
    private int watchDeadStreak = 0;
    /** ACTIVE mode: motor-integrated motion since the last fresh PWM reading (deg). */
    private double motNetSinceFresh = 0;
    private int deadStreak = 0;
    private int dirStreak = 0;
    /** Last wrap-resolved absolute reading (encoder-B revs, continuous); NaN = none yet. */
    private double lastAnchoredRevs = Double.NaN;
    /** Last anchor correction (deg, prediction-vs-absolute) — telemetry/slip visibility. */
    private double lastAnchorCorrDeg = Double.NaN;
    private int consecutiveAnchorRejects = 0;
    private int anchorRejectEvents = 0;
    /** Latched: motor encoder dead / persistently disagrees with the absolute path. */
    private boolean faultMotorEnc = false;
    /** Latched: motor encoder direction contradicts the absolute path (polarity/flip suspect). */
    private boolean faultMotorEncDir = false;
    /** Latched: the absolute (OctoQuad/encoder-B) stream stopped being read — DS failure-board alarm. */
    private boolean faultAbsStreamLost = false;
    /** Consecutive loops without an absolute read (drives faultAbsStreamLost). */
    private int absLostStreak = 0;
    /** Consecutive good absolute reads (auto-clears faultAbsStreamLost). */
    private int absGoodStreak = 0;
    // Boot direction guard, third vote (motor encoder — independent of both flip flags).
    private boolean bootDirMotPrimed = false;
    private int bootDirMotLastRaw = 0;
    private double bootDirSumMotDeg = 0;
    private boolean bootDirMotChecked = false;

    private final ElapsedTimeTimer timer = new ElapsedTimeTimer();

    /**
     * @param source                    the OctoQuad absolute source (encoder B + partner A)
     * @param motor                     the 5203 (its built-in encoder is read, never driven)
     * @param encoderRevsPerTurretRev   encoder-B revolutions per turret revolution (176/32)
     * @param encoderARevsPerTurretRev  encoder-A revolutions per turret revolution (176/31)
     * @param motorGearRatio            motor-output-to-turret ratio (176/64)
     */
    public TurretTracker(AbsoluteAngleSource source, DcMotorEx motor,
                         double encoderRevsPerTurretRev, double encoderARevsPerTurretRev,
                         double motorGearRatio) {
        this.absoluteSource = source;
        this.motor = motor;
        this.revsPerTurretB = encoderRevsPerTurretRev;
        this.revsPerTurretA = encoderARevsPerTurretRev;
        this.motorGearRatio = motorGearRatio;
        this.feedbackSourceName = source.describe();
    }

    // =========================================================================
    // Boot-time API
    // =========================================================================

    /**
     * Boot-time absolute-stability check: {@code ABS_BOOT_READS} reads spaced
     * ~12 ms apart, all CRC-valid and in-window (empty = fail) with spread
     * under {@code ABS_BOOT_SPREAD_DEG}. The turret must NOT be moving during
     * init for this to be meaningful (it isn't — nothing drives it at init).
     * Returns false (and logs loudly) when the absolute path looks unhealthy.
     */
    public boolean checkAbsoluteStabilityAtBoot() {
        final int reads = ABS_BOOT_READS;
        final long gapMs = 12;
        double minDeg = Double.MAX_VALUE;
        double maxDeg = -Double.MAX_VALUE;
        int ok = 0;
        for (int i = 0; i < reads; i++) {
            try {
                Thread.sleep(gapMs);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
                return false;
            }
            OptionalDouble r = absoluteSource.readRevs();
            if (r != null && r.isPresent()) {
                double deg = r.getAsDouble() * 360.0;
                minDeg = Math.min(minDeg, deg);
                maxDeg = Math.max(maxDeg, deg);
                ok++;
            }
        }
        if (ok == 0) {
            Log.e("IronLog", "TurretTracker ABS BOOT UNSTABLE: no valid absolute reads at init"
                    + " (check the OctoQuad channel/wiring, and the config echo above)");
            return false;
        }
        double spread = maxDeg - minDeg;
        if (spread > ABS_BOOT_SPREAD_DEG) {
            Log.e("IronLog", "TurretTracker ABS BOOT UNSTABLE: " + ok + "/" + reads
                    + " valid reads spread " + String.format(Locale.US, "%.2f", spread)
                    + " deg (limit " + ABS_BOOT_SPREAD_DEG
                    + ") — ABS PWM path suspect (cable mod? unseated connector?)");
            return false;
        }
        Log.i("IronLog", "TurretTracker ABS boot stability OK: " + ok + "/" + reads
                + " reads, spread " + String.format(Locale.US, "%.2f", spread) + " deg");
        return true;
    }

    /**
     * Anchor the tracked stream after a boot solve: the tracked stream starts
     * at the raw absolute reading (offset-free), then the reference is aligned
     * so position() == bootDeg at this instant. Also (re)primes the hybrid
     * motor-encoder path — it is delta-based, so it restarts cleanly from the
     * current count and re-runs WATCH (direction confirmation at the first
     * powered motion).
     *
     * @param bootDeg the boot angle from the CRT solve (0 = assumed forward)
     * @param absRevs the flipped encoder-B reading at this instant; empty = no
     *                valid absolute read (the stream stays at 0)
     */
    public void anchorBoot(double bootDeg, OptionalDouble absRevs) {
        if (absRevs.isPresent()) {
            unwrappedDeg = absRevs.getAsDouble() * 360.0 / revsPerTurretB;
        }
        zeroOffsetDeg = unwrappedDeg - bootDeg;
        lastAbsRevs = absRevs.isPresent() ? absRevs.getAsDouble() : Double.NaN;
        absPrimed = absRevs.isPresent();
        absHist0 = lastAbsRevs;
        absHist1 = lastAbsRevs;

        motorEncPrimed = false;
        motorEncReadFails = 0;
        motorEncConsecGlitches = 0;
        watchStartPos = Double.NaN;
        watchMotNetDeg = 0;
        watchDeadStreak = 0;
        motNetSinceFresh = 0;
        deadStreak = 0;
        dirStreak = 0;
        lastAnchoredRevs = Double.NaN;
        lastAnchorCorrDeg = Double.NaN;
        consecutiveAnchorRejects = 0;
        motorEncMode = resolveMotorEncMode();
    }

    /** Declare "the turret points at 0 RIGHT NOW" (soft re-zero, no twitch). */
    public void zeroHere() {
        zeroOffsetDeg = unwrappedDeg;
    }

    /**
     * Call from an OpMode's init_loop(): boot "is the encoder alive" check —
     * the humans hand-move the turret a little and we look for the feedback to
     * respond. Latches {@link #isBootEncoderVerified()}. (Deliberately a
     * human-powered check — the turret is not auto-wiggled at init.)
     */
    public void pollBootEncoderCheck() {
        try {
            double sig = bootFeedbackSignature();
            // The same absolute read feeds the direction guard — zero extra I2C traffic.
            pollBootDirectionGuard(sig / 360.0);
            if (Double.isNaN(bootCheckSig)) {
                bootCheckSig = sig;
                return;
            }
            if (Math.abs(sig - bootCheckSig) >= bootCheckThreshold()) {
                if (!bootEncoderVerified) {
                    Log.i("IronLog", "TurretTracker boot check PASSED — encoder responds to motion");
                }
                bootEncoderVerified = true;
                faultEncoderDeadAtBoot = false;
            }
        } catch (Exception e) {
            faultEncoderDeadAtBoot = true;
            Log.e("IronLog", "TurretTracker boot check: encoder read failed", e);
        }
    }

    /** Feedback signature for the boot check: the absolute reading in degrees. */
    private double bootFeedbackSignature() {
        OptionalDouble abs = absoluteSource.readRevs();
        if (!abs.isPresent()) {
            throw new IllegalStateException("no absolute reading for boot check");
        }
        return abs.getAsDouble() * 360.0;
    }

    /** "Did it move" threshold matching {@link #bootFeedbackSignature()}'s units. */
    private double bootCheckThreshold() {
        return 1.0; // ~3 PWM steps (1 deg)
    }

    /**
     * Boot-time DIRECTION cross-guard. Fed by the SAME absolute read the boot
     * check just did ({@code bFlippedRevs} = tracking encoder B in its FLIPPED
     * frame — the frame the PID actually consumes). While the humans hand-move
     * the turret for the boot check, it accumulates B's motion against encoder
     * A's solve-frame motion. Both frames are CCW-positive when the direction
     * flags are right, so over the same physical motion their signs must agree.
     * A disagreement means the tracking encoder's flip is not what the stream
     * is doing (stale runtime override, wiring change, swapped channels) — the
     * exact failure that mirrored the dashboard and made the PID a
     * positive-feedback runaway on 2026-10-23. Latches
     * {@link #isBootDirectionMismatch()} after ~15 turret degrees of hand
     * motion on BOTH channels. A THIRD vote — the motor's built-in encoder,
     * whose turret-frame sign is independent of BOTH Through-Bore flip flags —
     * catches the "two wrong flips look unanimous" blind spot (assuming
     * MOTOR_ENC_POLARITY was bench-verified; the log says which suspect first).
     */
    private void pollBootDirectionGuard(double bFlippedRevs) {
        if (bootDirMismatch || !(absoluteSource instanceof OctoQuadAbsoluteSource)) {
            return;
        }
        OctoQuadAbsoluteSource src = (OctoQuadAbsoluteSource) absoluteSource;
        OptionalDouble aOpt = src.readCrossRevs(); // cached from the same transaction
        if (!aOpt.isPresent()) {
            bootDirLastB = bootDirLastA = Double.NaN; // stale pair — restart accumulation
            return;
        }
        double b = bFlippedRevs;
        double a = TurretCrt.solveFrameRevs(aOpt.getAsDouble(),
                TurretCrt.CRT_ZERO_OFFSET_A_REV, TurretCrt.ENCODER_A_REVERSED);
        if (Double.isNaN(bootDirLastB) || Double.isNaN(bootDirLastA)) {
            bootDirLastB = b;
            bootDirLastA = a;
            return;
        }
        // Both sums accumulate FORWARD motion (from last reading to now) so
        // their signs are directly comparable (§12.20: the old code had two
        // helpers with opposite argument orders; forwardDiffRevs makes the
        // intent explicit).
        bootDirSumBRevs += TurretCrt.forwardDiffRevs(bootDirLastB, b);
        bootDirSumARevs += TurretCrt.forwardDiffRevs(bootDirLastA, a);
        bootDirLastB = b;
        bootDirLastA = a;
        double bDeg = bootDirSumBRevs * 360.0 / revsPerTurretB;
        double aDeg = bootDirSumARevs * 360.0 / revsPerTurretA;
        if (Math.abs(bDeg) >= 15.0 && Math.abs(aDeg) >= 15.0) {
            bootDirChecked = true;
            if ((bDeg > 0) != (aDeg > 0)) {
                bootDirMismatch = true;
                faultEncoderDirection = true;
                Log.e("IronLog", "TurretTracker boot DIRECTION MISMATCH: B(flipped) " + bDeg
                        + " deg vs A solve-frame " + aDeg + " deg — ENCODER_REVERSED is not what the stream sees");
            } else {
                Log.i("IronLog", "TurretTracker boot direction check PASSED (B " + bDeg
                        + " deg / A " + aDeg + " deg agree)");
            }
        }

        // ---- Third vote — the motor's built-in encoder ----
        if (!bootDirMotChecked && !bootDirMismatch) {
            try {
                int raw = motor.getCurrentPosition();
                if (!bootDirMotPrimed) {
                    bootDirMotPrimed = true;
                    bootDirMotLastRaw = raw;
                } else {
                    bootDirSumMotDeg += (raw - bootDirMotLastRaw) * MOTOR_ENC_POLARITY
                            * 360.0 / Math.max(1.0, motorEncTicksPerTurretRev());
                    bootDirMotLastRaw = raw;
                    double bDegForMot = bootDirSumBRevs * 360.0 / revsPerTurretB;
                    if (Math.abs(bootDirSumMotDeg) >= 15.0 && Math.abs(bDegForMot) >= 15.0) {
                        bootDirMotChecked = true;
                        if ((bDegForMot > 0) != (bootDirSumMotDeg > 0)) {
                            bootDirMismatch = true;
                            faultEncoderDirection = true;
                            Log.e("IronLog", "TurretTracker boot DIRECTION MISMATCH (motor vote): B(flipped) "
                                    + String.format(Locale.US, "%+.1f", bDegForMot)
                                    + " deg vs motor-frame "
                                    + String.format(Locale.US, "%+.1f", bootDirSumMotDeg)
                                    + " deg — suspect ENCODER_REVERSED (or MOTOR_ENC_POLARITY if unverified)");
                        } else {
                            Log.i("IronLog", "TurretTracker boot direction motor vote PASSED (B "
                                    + String.format(Locale.US, "%+.1f", bDegForMot)
                                    + " deg / motor "
                                    + String.format(Locale.US, "%+.1f", bootDirSumMotDeg) + " deg agree)");
                        }
                    }
                }
            } catch (Exception ignored) {
                // Motor encoder read trouble at init — watch mode will judge
                // it at the first powered motion instead.
            }
        }
    }

    // =========================================================================
    // Per-loop update
    // =========================================================================

    /**
     * Feedback pipeline — call once per loop BEFORE the PID runs. In hybrid
     * ACTIVE mode the motor encoder integrates per-loop motion (read FIRST, so
     * the PWM anchor's prediction includes this loop's motion) while the
     * OctoQuad absolute anchors and cross-checks; in every other mode the
     * encoder-B PWM path owns tracking and the motor path only watches. Either
     * stream failing alone is tolerated (the other keeps tracking); both
     * failing counts as feedback loss (see {@link #feedbackLost()}).
     */
    public void update(double dtSec) {
        boolean hybridCandidate = motorEncMode == MotEncMode.WATCH
                || motorEncMode == MotEncMode.ACTIVE;
        boolean motorOk = true;
        if (hybridCandidate) {
            try {
                motorOk = updateFromMotorEncoder(dtSec);
            } catch (Exception e) {
                if (motorEncReadFails == 0) {
                    Log.e("IronLog", "TurretTracker motor-encoder read failed", e);
                }
                motorOk = false;
            }
        }
        boolean absOk;
        try {
            absOk = updateFromAbsolute(dtSec);
        } catch (Exception e) {
            if (feedbackFailures == 0) {
                Log.e("IronLog", "TurretTracker feedback read failed (1)", e);
            }
            absOk = false;
        }

        // ---- Absolute-stream liveness (driver-station failure board) ----
        // updateFromAbsolute() returns false ONLY when the OctoQuad read came
        // back empty (dead device / unplugged ABS wire). In hybrid ACTIVE the
        // motor encoder keeps tracking, so this is NOT feedbackLost() — but
        // the at-rest anchor, the A cross-check and the boot-correct position
        // are all gone, and the driver needs the red banner. Glitch-gate
        // rejections do NOT count (the stream is alive; see absGlitchEvents).
        if (absOk) {
            absLostStreak = 0;
            if (faultAbsStreamLost && ++absGoodStreak >= ABS_STREAM_LOST_LOOPS) {
                faultAbsStreamLost = false;
                Log.i("IronLog", "TurretTracker absolute stream recovered ("
                        + ABS_STREAM_LOST_LOOPS + " good reads)");
            }
        } else {
            absGoodStreak = 0;
            if (!faultAbsStreamLost && ++absLostStreak >= ABS_STREAM_LOST_LOOPS) {
                faultAbsStreamLost = true;
                Log.e("IronLog", "TurretTracker ABS STREAM LOST for "
                        + ABS_STREAM_LOST_LOOPS + " loops — OctoQuad/encoder-B reads empty"
                        + " (hybrid still integrating on the motor encoder)");
            }
        }

        // ---- WATCH-mode direction confirmation ----
        // The first powered motion must show the motor encoder integrating the
        // SAME direction the (PWM-tracked) turret actually moved, before
        // per-loop integration is trusted. Until then the PWM-only tracker
        // owns position and PWM_ONLY_MAX_POWER caps the speed.
        if (motorEncMode == MotEncMode.WATCH && absOk) {
            if (Double.isNaN(watchStartPos)) {
                watchStartPos = position();
            }
            double pwmNet = position() - watchStartPos;
            if (Math.abs(pwmNet) >= MOTOR_ENC_CONFIRM_PWM_DEG) {
                if (Math.abs(watchMotNetDeg) >= MOTOR_ENC_CONFIRM_MOT_DEG
                        && (pwmNet > 0) == (watchMotNetDeg > 0)) {
                    motorEncMode = MotEncMode.ACTIVE;
                    Log.i("IronLog", "TurretTracker hybrid ACTIVE: motor encoder confirmed (PWM "
                            + String.format(Locale.US, "%+.1f", pwmNet) + " deg vs motor "
                            + String.format(Locale.US, "%+.1f", watchMotNetDeg) + " deg)");
                } else if (Math.abs(watchMotNetDeg) < MOTOR_ENC_DEAD_MOT_DEG) {
                    watchDeadStreak++;
                    Log.e("IronLog", "TurretTracker mot-enc dead evidence " + watchDeadStreak
                            + "/" + MOTOR_ENC_STREAK_LIMIT + " (turret moved "
                            + String.format(Locale.US, "%+.1f", pwmNet)
                            + " deg, motor encoder saw nothing)");
                    watchStartPos = position();
                    watchMotNetDeg = 0;
                    if (watchDeadStreak >= MOTOR_ENC_STREAK_LIMIT) {
                        faultMotorEnc = true;
                        motorEncMode = MotEncMode.OFF;
                        motorEncOffReason = "motor encoder dead (no counts while turret moves)";
                        Log.e("IronLog", "TurretTracker HYBRID OFF -> motor encoder dead;"
                                + " PWM-only tracking resumes (speed re-capped)");
                    }
                } else {
                    faultMotorEncDir = true;
                    motorEncMode = MotEncMode.OFF;
                    motorEncOffReason = "direction mismatch — check MOTOR_ENC_POLARITY";
                    Log.e("IronLog", "TurretTracker HYBRID OFF -> motor integrated "
                            + String.format(Locale.US, "%+.1f", watchMotNetDeg)
                            + " deg while the turret moved "
                            + String.format(Locale.US, "%+.1f", pwmNet)
                            + " deg — MOTOR_ENC_POLARITY (or an encoder flip) is wrong");
                }
            }
        }

        // Only the ACTIVE mode can keep tracking without the absolute stream
        // (motor quadrature is a real feedback source); everywhere else the
        // PWM path IS the tracker, so its health is the feedback health.
        boolean ok = motorEncMode == MotEncMode.ACTIVE ? (absOk || motorOk) : absOk;
        if (ok) {
            feedbackFailures = 0;
        } else {
            feedbackFailures++;
        }
        if (!ok) {
            velocityDegPerSec = 0;
            velEmaDegPerSec = 0;
        }
    }

    /**
     * The OctoQuad absolute path — the production tracker AND the encoder-B
     * fallback. Unwraps the absolute PWM angle into the tracked stream with
     * glitch defenses (the wild-heading capture showed single-sample ±150°
     * garbage reads during motion that the tracker used to integrate
     * faithfully):
     * <ol>
     *   <li><b>Range validation</b> happens inside the source (out-of-window
     *       raw µs = empty read + OOR counter).</li>
     *   <li><b>Median-of-3</b> on the raw stream kills single-sample outliers
     *       between two good reads (one-tick lag on real motion).</li>
     *   <li><b>Implausibility gate</b>: a step bigger than
     *       {@link #ABS_MAX_PLAUSIBLE_DPS} × dt is not integrated — counted
     *       and skipped; after {@link #ABS_REJECT_REANCHOR} consecutive
     *       rejects the tracker re-anchors to the raw reading (recovery, so a
     *       persistent step can't freeze position forever).</li>
     * </ol>
     * In hybrid ACTIVE mode the integration/gate is SKIPPED (motion is owned
     * by the motor encoder) and this path only anchors + cross-checks.
     */
    private boolean updateFromAbsolute(double dtSec) {
        OptionalDouble abs = absoluteSource.readRevs();
        if (abs == null || !abs.isPresent()) {
            return false;
        }
        // Raw (pre-median) value for telemetry/calibration: prefer the source's
        // raw microseconds — the would-be angle is informative even while the
        // glitch filters are doing their job.
        if (absoluteSource instanceof OctoQuadAbsoluteSource) {
            OctoQuadAbsoluteSource src = (OctoQuadAbsoluteSource) absoluteSource;
            lastAbsRawDeg = src.getLastRawUs() * 360.0 / Math.max(1, OctoQuadAbsoluteSource.ABS_PWM_MAX_US);
        } else {
            lastAbsRawDeg = abs.getAsDouble() * 360.0;
        }
        double revs = abs.getAsDouble();

        // ---- Median-of-3 glitch filter ----
        if (ABS_MEDIAN_WINDOW >= 3) {
            double med = median3(absHist0, absHist1, revs);
            absHist0 = absHist1;
            absHist1 = revs;
            revs = med;
        }

        if (!absPrimed) {
            // Boot already anchored the stream at the raw reading; just finish priming.
            lastAbsRevs = revs;
            absPrimed = true;
            velocityDegPerSec = 0;
            absLastChangeTime = Double.NaN;
            return true;
        }

        // Shortest-path delta in revolutions, seam-crossing safe.
        // forwardDiffRevs(last, now) = now - last = the FORWARD motion
        // (fracDiff's arguments are the other way around — §12.20).
        double deltaRevs = TurretCrt.forwardDiffRevs(lastAbsRevs, revs);

        if (motorEncMode == MotEncMode.ACTIVE) {
            // ---- HYBRID: motion is owned by the motor encoder; this stream
            // only ANCHORS (see the anchor helpers below). The implausibility
            // gate / direct integration below do NOT run: at full hybrid speed
            // a fresh PWM step can legitimately exceed half an encoder-B wrap,
            // and the wrap-resolved anchor handles it.
            lastAbsRevs = revs;
            if (deltaRevs != 0.0) {
                absLastChangeTime = timer.seconds();
            }
            if (Math.abs(velocityDegPerSec) <= MOTOR_ENC_REST_ANCHOR_VEL_DPS) {
                anchorAtRest(revs);
            } else if (deltaRevs != 0.0) {
                anchorWhileMoving(revs);
            }
            crossCheckAgainstPartner();
            return true;
        }

        // ---- ENCODER-B-ONLY (PWM) path — also active during hybrid WATCH ----
        //
        // ---- Implausibility gate ----
        // deltaRevs is in ENCODER-B revs: convert the turret-speed limit into
        // encoder revs, and clamp at 0.5 enc-rev/loop — encoder B repeats
        // every 65.5 deg of turret travel, so a bigger per-loop step is
        // indistinguishable from a PWM wrap alias and can never be trusted as
        // motion (the re-anchor path handles sustained overspeed; the
        // PWM_ONLY_MAX_POWER cap keeps real speeds under the ceiling anyway —
        // the binding ceiling is the ~10 Hz PWM refresh, i.e. half a wrap per
        // FRESH reading (~every 2nd loop), ~273-310 deg/s).
        double maxStepRevs = Math.min(0.5,
                Math.abs(ABS_MAX_PLAUSIBLE_DPS) * Math.max(dtSec, 1e-3) / 360.0
                        * revsPerTurretB);
        if (Math.abs(deltaRevs) > maxStepRevs) {
            consecutiveRejects++;
            absGlitchEvents++;
            if (consecutiveRejects >= ABS_REJECT_REANCHOR) {
                // Persistent implausible steps: re-anchor (accept the raw's
                // word) rather than freeze forever — and say so loudly.
                Log.e("IronLog", "TurretTracker ABS re-anchor after " + consecutiveRejects
                        + " implausible steps (wiring/config suspicion)");
                consecutiveRejects = 0;
                lastAbsRevs = revs;
            }
            // Skip the velocity update too — a rejected step is not motion.
            return true;
        }
        consecutiveRejects = 0;

        // A quarter-encoder-rev step that PASSED the gate is still noteworthy
        // (should not happen with the gate above — keep the counter for
        // telemetry).
        if (Math.abs(deltaRevs) > 0.25) {
            wrapEvents++;
            Log.w("IronLog", "TurretTracker suspicious absolute step: "
                    + String.format(Locale.US, "%+.3f revs (%+.1f deg turret)",
                    deltaRevs, deltaRevs * 360.0 * revsPerTurretB));
        }
        double deltaDeg = deltaRevs * 360.0 / revsPerTurretB;
        unwrappedDeg += deltaDeg;
        lastAbsRevs = revs;
        // The OctoQuad's PWM width only REFRESHES at ~10 Hz — at a ~20 Hz loop
        // the raw value is identical on every other loop, so the delta on a
        // fresh loop covers ~2 loops of motion. Dividing it by the single-loop
        // dt doubled the velocity and turned the D term into a 0<->2v square
        // wave. Divide by the time since the reading actually changed instead;
        // between changes the last measurement is HELD, and after
        // ABS_VELOCITY_STALE_S without any change the turret has (effectively)
        // stopped -> relax to 0.
        double now = timer.seconds();
        if (deltaRevs != 0.0) {
            double effDt = Double.isNaN(absLastChangeTime)
                    ? dtSec : clamp(now - absLastChangeTime, Math.max(dtSec, 1e-3), 0.5);
            computeVelocity(deltaDeg, effDt);
            absLastChangeTime = now;
        } else if (!Double.isNaN(absLastChangeTime)
                && (now - absLastChangeTime) > ABS_VELOCITY_STALE_S) {
            computeVelocity(0, dtSec);
        }
        crossCheckAgainstPartner();
        return true;
    }

    /** Median of three values. */
    private static double median3(double a, double b, double c) {
        return Math.max(Math.min(a, b), Math.min(Math.max(a, b), c));
    }

    /**
     * Every-loop cross-check: the tracked turret angle predicts encoder A's
     * reading; persistent disagreement (a slipped magnet / drifting chain)
     * latches CRT_CROSSCHECK. The tolerance is speed-aware: encoder A's PWM
     * refreshes at ~10 Hz, so while moving it is up to ~0.1 s stale vs the
     * tracked stream — legitimate disagreement grows with speed
     * (TurretCrt.CRT_CROSSCHECK_STALE_S).
     */
    private void crossCheckAgainstPartner() {
        if (!(absoluteSource instanceof OctoQuadAbsoluteSource)) {
            return;
        }
        OctoQuadAbsoluteSource src = (OctoQuadAbsoluteSource) absoluteSource;
        OptionalDouble aOpt = src.readCrossRevs();
        if (aOpt.isPresent()) {
            // Same raw-frame convention as the boot solve — zero offset off
            // the RAW reading first, direction flip after.
            double aFrac = TurretCrt.solveFrameRevs(aOpt.getAsDouble(),
                    TurretCrt.CRT_ZERO_OFFSET_A_REV, TurretCrt.ENCODER_A_REVERSED);
            double turretTurns = position() / 360.0;
            crtCrosscheckErrDeg = TurretCrt.crosscheckDiffDeg(aFrac, turretTurns, revsPerTurretA);
            // Speed-aware tolerance (see TurretCrt.CRT_CROSSCHECK_STALE_S).
            double tol = TurretCrt.CRT_CROSSCHECK_TOL_DEG
                    + TurretCrt.CRT_CROSSCHECK_STALE_S * Math.abs(velocityDegPerSec);
            if (TurretCrt.CRT_CHECK_MAX_SPEED_DPS <= 0
                    || Math.abs(velocityDegPerSec) <= TurretCrt.CRT_CHECK_MAX_SPEED_DPS) {
                if (Math.abs(crtCrosscheckErrDeg) > tol) {
                    crtCrosscheckFails++;
                    if (crtCrosscheckFails >= TurretCrt.CRT_CROSSCHECK_FAIL_LIMIT
                            && !faultCrtCrosscheck) {
                        faultCrtCrosscheck = true;
                        Log.e("IronLog", "TurretTracker CRT cross-check FAILED: tracked-vs-A error "
                                + String.format(Locale.US, "%+.2f deg (speed-aware limit %.2f)",
                                crtCrosscheckErrDeg, tol)
                                + " after " + crtCrosscheckFails
                                + " consecutive loops — check the A-encoder pinion/magnet.");
                    }
                } else {
                    crtCrosscheckFails = 0;
                    // A pass clears the counter only — latched faults stay
                    // until clearFaults().
                }
            }
        } else {
            crtCrosscheckErrDeg = Double.NaN;
        }
    }

    /**
     * HYBRID per-loop motion source: the turret motor's built-in quadrature
     * encoder. The hub counts the edges in hardware between loops, so the
     * delta over a loop is exact at any speed — this is what removes the
     * ~10 Hz PWM-refresh ceiling. Integrates into the tracked stream and feeds
     * the velocity filter ONLY in ACTIVE mode; in WATCH mode it only
     * accumulates direction-confirmation evidence (the PWM path owns
     * integration until the motor encoder has proven its polarity).
     */
    private boolean updateFromMotorEncoder(double dtSec) {
        int pos;
        try {
            pos = motor.getCurrentPosition();
        } catch (Exception e) {
            motorEncReadFails++;
            if (motorEncMode == MotEncMode.ACTIVE
                    && motorEncReadFails >= MOTOR_ENC_READ_FAIL_LIMIT) {
                faultMotorEnc = true;
                motorEncMode = MotEncMode.OFF;
                motorEncOffReason = "motor encoder read failures";
                Log.e("IronLog", "TurretTracker HYBRID OFF -> " + motorEncReadFails
                        + " consecutive motor-encoder read failures;"
                        + " PWM-only tracking resumes (speed re-capped)");
            }
            return false;
        }
        motorEncReadFails = 0;
        if (!motorEncPrimed) {
            // First read after boot/re-zero: anchor the delta tracker — the
            // absolute boot already aligned the tracked stream.
            motorEncLastPos = pos;
            motorEncPrimed = true;
            return true;
        }
        int deltaTicks = pos - motorEncLastPos;
        motorEncLastPos = pos;
        double deltaDeg = deltaTicks * MOTOR_ENC_POLARITY * 360.0
                / Math.max(1.0, motorEncTicksPerTurretRev());
        // Plausibility: hub quadrature counters don't lose counts; a per-loop
        // step beyond the physical ceiling is a glitch/counter reset, not
        // motion. Reject it; after ABS_REJECT_REANCHOR consecutive rejects,
        // re-prime from the current count (recovery, same philosophy as the
        // absolute gate).
        if (Math.abs(deltaDeg) > Math.abs(ABS_MAX_PLAUSIBLE_DPS) * Math.max(dtSec, 1e-3)) {
            motorEncGlitchEvents++;
            motorEncConsecGlitches++;
            if (motorEncConsecGlitches >= ABS_REJECT_REANCHOR) {
                motorEncConsecGlitches = 0;
                motorEncLastPos = pos;
                Log.w("IronLog", "TurretTracker mot-enc re-primed after "
                        + ABS_REJECT_REANCHOR + " implausible steps");
            }
            return true;
        }
        motorEncConsecGlitches = 0;
        if (motorEncMode == MotEncMode.ACTIVE) {
            unwrappedDeg += deltaDeg;
            computeVelocity(deltaDeg, dtSec);
            motNetSinceFresh += deltaDeg;
        } else { // WATCH: PWM owns integration; the motor path only accumulates evidence
            watchMotNetDeg += deltaDeg;
        }
        return true;
    }

    /**
     * HYBRID at-rest anchor: with the turret stationary the PWM width is fresh
     * by definition (nothing moved since the chip latched it), so the tracked
     * position is hard-snapped to the wrap-resolved absolute reading. This is
     * what erases backlash and any motor-integration drift between motions —
     * right before the shot, which is exactly when absolute accuracy matters.
     */
    private void anchorAtRest(double revsFresh) {
        double predRevs = unwrappedDeg * revsPerTurretB / 360.0;
        double anchoredRevs = revsFresh + Math.round(predRevs - revsFresh);
        double corrDeg = (anchoredRevs - predRevs) * 360.0 / revsPerTurretB;
        if (Math.abs(corrDeg) > MOTOR_ENC_ANCHOR_REJECT_DEG) {
            consecutiveAnchorRejects++;
            anchorRejectEvents++;
            if (consecutiveAnchorRejects >= ABS_REJECT_REANCHOR) {
                Log.e("IronLog", "TurretTracker hybrid: absurd at-rest anchor ("
                        + String.format(Locale.US, "%+.2f", corrDeg)
                        + " deg) persists — re-anchoring to the PWM absolute");
                consecutiveAnchorRejects = 0;
                lastAnchorCorrDeg = corrDeg;
                unwrappedDeg = anchoredRevs * 360.0 / revsPerTurretB;
                lastAnchoredRevs = anchoredRevs;
            }
            return;
        }
        consecutiveAnchorRejects = 0;
        lastAnchorCorrDeg = corrDeg;
        lastAnchoredRevs = anchoredRevs;
        unwrappedDeg = anchoredRevs * 360.0 / revsPerTurretB;
    }

    /**
     * HYBRID moving monitor: a fresh PWM reading while moving is up to one
     * ~106 ms refresh interval STALE, so correcting position with it would
     * rewind real motion — it is wrap-resolved, recorded for telemetry, and
     * NOT applied. What IS acted on: the wrap-resolved INTERVAL between
     * consecutive fresh readings vs the motor-integrated motion over the same
     * span — a dead motor encoder (absolute path moves, motor integrates
     * nothing) or a wrong polarity (motor integrates the wrong way) is
     * detectable regardless of staleness and latches after
     * {@link #MOTOR_ENC_STREAK_LIMIT} evidences.
     */
    private void anchorWhileMoving(double revsFresh) {
        double predRevs = unwrappedDeg * revsPerTurretB / 360.0;
        double anchoredRevs = revsFresh + Math.round(predRevs - revsFresh);
        lastAnchorCorrDeg = (anchoredRevs - predRevs) * 360.0 / revsPerTurretB;
        if (!Double.isNaN(lastAnchoredRevs)) {
            double pwmIntervalDeg = (anchoredRevs - lastAnchoredRevs) * 360.0
                    / revsPerTurretB;
            if (Math.abs(pwmIntervalDeg) >= MOTOR_ENC_DEAD_PWM_DEG) {
                if (Math.abs(motNetSinceFresh) < MOTOR_ENC_DEAD_MOT_DEG) {
                    deadStreak++;
                    if (deadStreak >= MOTOR_ENC_STREAK_LIMIT) {
                        faultMotorEnc = true;
                        demoteHybrid(anchoredRevs,
                                "motor encoder saw no motion the absolute path confirms");
                        return;
                    }
                } else {
                    deadStreak = 0;
                    if ((pwmIntervalDeg > 0) != (motNetSinceFresh > 0)) {
                        dirStreak++;
                        if (dirStreak >= MOTOR_ENC_STREAK_LIMIT) {
                            faultMotorEncDir = true;
                            demoteHybrid(anchoredRevs,
                                    "motor encoder moves OPPOSITE the absolute path"
                                            + " — check MOTOR_ENC_POLARITY");
                            return;
                        }
                    } else {
                        dirStreak = 0;
                    }
                }
            }
        }
        lastAnchoredRevs = anchoredRevs;
        motNetSinceFresh = 0;
        if (Math.abs(lastAnchorCorrDeg) > MOTOR_ENC_ANCHOR_REJECT_DEG) {
            consecutiveAnchorRejects++;
            anchorRejectEvents++;
            if (consecutiveAnchorRejects >= ABS_REJECT_REANCHOR) {
                Log.e("IronLog", "TurretTracker hybrid: " + consecutiveAnchorRejects
                        + " absurd anchor corrections in a row — re-anchoring to the PWM absolute");
                consecutiveAnchorRejects = 0;
                unwrappedDeg = anchoredRevs * 360.0 / revsPerTurretB;
                lastAnchoredRevs = anchoredRevs;
            }
        } else {
            consecutiveAnchorRejects = 0;
        }
    }

    /** Demote the hybrid to PWM-only, anchoring to the absolute reading that just won the argument. */
    private void demoteHybrid(double anchoredRevs, String reason) {
        motorEncMode = MotEncMode.OFF;
        motorEncOffReason = reason;
        motNetSinceFresh = 0;
        // Trust the absolute reading: re-anchor so the legacy tracker resumes
        // from the truth, not from the suspect motor-integrated estimate.
        unwrappedDeg = anchoredRevs * 360.0 / revsPerTurretB;
        lastAnchoredRevs = anchoredRevs;
        Log.e("IronLog", "TurretTracker HYBRID OFF -> " + reason
                + "; PWM-only tracking resumes (speed re-capped)");
    }

    /**
     * Live hybrid scale: motor-encoder ticks per ONE turret revolution
     * (28 x 19.2032 x 2.75 ≈ 1478.7). Computed live so Panels edits to the
     * gearing constants carry into the hybrid immediately.
     */
    private double motorEncTicksPerTurretRev() {
        return Math.max(1.0, MOTOR_ENC_TICKS_PER_MOTOR_REV)
                * Math.max(1e-6, MOTOR_PLANETARY_RATIO)
                * Math.max(1e-6, motorGearRatio);
    }

    /** Mode selection at boot/re-zero; latched faults keep the hybrid OFF. */
    private MotEncMode resolveMotorEncMode() {
        if (!MOTOR_ENC_HYBRID) {
            motorEncOffReason = "MOTOR_ENC_HYBRID=false (encoder-B-only mode)";
            return MotEncMode.OFF;
        }
        if (faultMotorEnc || faultMotorEncDir) {
            motorEncOffReason = "fault latched (re-zero or clearFaults to re-arm)";
            return MotEncMode.OFF;
        }
        return MotEncMode.WATCH;
    }

    /**
     * Self-computed turret velocity with a light EMA — the OctoQuad PWM angle
     * is quantized to 1024 steps (0.35 deg), so a raw per-loop derivative
     * steps ~17 deg/s per PWM step; the EMA smooths that into the D term
     * without adding much lag. {@code VELOCITY_EMA_ALPHA = 1.0} disables it.
     */
    private void computeVelocity(double deltaDeg, double dtSec) {
        if (dtSec <= 0) {
            return;
        }
        double measured = deltaDeg / dtSec;
        // Sane-loop clamp: a single garbage delta can't become a 1e4 deg/s
        // velocity spike into the D term.
        measured = clamp(measured, -3600.0, 3600.0);
        double a = clamp(VELOCITY_EMA_ALPHA, 0.05, 1.0);
        velEmaDegPerSec += a * (measured - velEmaDegPerSec);
        velocityDegPerSec = velEmaDegPerSec;
    }

    // =========================================================================
    // Readings for the subsystem (TurretPivot)
    // =========================================================================

    /** The tracked turret angle: robot-oriented degrees, 0 = forward, +CCW, continuous multi-turn. */
    public double position() {
        return unwrappedDeg - zeroOffsetDeg;
    }

    /** The tracked continuous angle in raw form (degrees since the boot zero) — bench/telemetry tool. */
    public double unwrapped() {
        return unwrappedDeg;
    }

    /** Measured turret velocity (deg/s, +CCW). */
    public double velocityDegPerSec() {
        return velocityDegPerSec;
    }

    /** True when the hybrid motor encoder owns per-loop motion (drives the power-cap choice). */
    public boolean hybridActive() {
        return motorEncMode == MotEncMode.ACTIVE;
    }

    /** Feedback health: true unless reads have been failing for FEEDBACK_FAIL_LIMIT loops straight. */
    public boolean feedbackLost() {
        return feedbackFailures >= FEEDBACK_FAIL_LIMIT;
    }

    /** Consecutive failed feedback reads (used by the subsystem for its BRAKE log line). */
    public int feedbackFailureCount() {
        return feedbackFailures;
    }

    /** The OctoQuad source identity (for telemetry). */
    public String feedbackSourceName() {
        return feedbackSourceName;
    }

    /** Wrap/glitch absorption events since boot (expect ~0). */
    public int wrapEvents() {
        return wrapEvents;
    }

    /** Latest raw absolute reading in degrees [0,360) (NaN when unavailable) — calibration helper. */
    public double lastAbsRawDeg() {
        return lastAbsRawDeg;
    }

    /** Diagnostic counters {oor, crcFail, glitch} from the absolute path. */
    public int[] absoluteDiagCounters() {
        int oor = 0;
        int crc = 0;
        if (absoluteSource instanceof OctoQuadAbsoluteSource) {
            OctoQuadAbsoluteSource src = (OctoQuadAbsoluteSource) absoluteSource;
            oor = src.oorEvents;
            crc = src.crcFailEvents;
        }
        return new int[]{oor, crc, absGlitchEvents};
    }

    /** Boot check: true once the boot hand-move check saw the encoder actually counting. */
    public boolean isBootEncoderVerified() {
        return bootEncoderVerified;
    }

    /** True once the boot direction guard completed (~15+ deg of hand motion on both channels). */
    public boolean isBootDirectionChecked() {
        return bootDirChecked;
    }

    /** True when the boot direction guard caught a flip/direction mismatch. */
    public boolean isBootDirectionMismatch() {
        return bootDirMismatch;
    }

    /** True when the encoder-A cross-check latched a failure (slipped magnet / drifting chain). */
    public boolean isCrtCrosscheckFaulted() {
        return faultCrtCrosscheck;
    }

    /** Last cross-check error in turret degrees (NaN = not checked yet / skipped / no partner). */
    public double crtCrosscheckErrDeg() {
        return crtCrosscheckErrDeg;
    }

    /** Consecutive cross-check failures (telemetry). */
    public int crtCrosscheckFails() {
        return crtCrosscheckFails;
    }

    /** Latched motor-encoder faults (hybrid demotion reasons). */
    public boolean isMotorEncFaulted() {
        return faultMotorEnc;
    }

    /** Latched motor-encoder direction fault (polarity/flip suspect). */
    public boolean isMotorEncDirFaulted() {
        return faultMotorEncDir;
    }

    /** True when the boot hand-move check found the encoder dead. */
    public boolean isEncoderDeadAtBoot() {
        return faultEncoderDeadAtBoot;
    }

    /** Latched: hand-motion direction disagrees with encoder A / the motor (flip suspect). */
    public boolean isEncoderDirectionFaulted() {
        return faultEncoderDirection;
    }

    /** Latched when the OctoQuad/encoder-B stream stopped being read (auto-clears on recovery). */
    public boolean isAbsStreamLost() {
        return faultAbsStreamLost;
    }

    // ---- One-line telemetry texts (keep TurretPivot.periodic readable) ----

    /** One-line hybrid status for telemetry ("Turret mot-enc"). */
    public String modeText() {
        switch (motorEncMode) {
            case ACTIVE:
                return String.format(Locale.US, "ACTIVE (pol %+.0f, %.1f ticks/turret-rev)",
                        MOTOR_ENC_POLARITY, motorEncTicksPerTurretRev());
            case WATCH:
                return "WATCH — confirming direction at first powered motion";
            default:
                return "OFF — " + motorEncOffReason;
        }
    }

    /** The motor encoder's live tick count ("Turret mot-enc ticks"). */
    public String motorEncTicksText() {
        return motorEncPrimed ? String.valueOf(motorEncLastPos) : "n/a";
    }

    /** The last anchor correction ("Turret anchor corr"). */
    public String anchorCorrText() {
        return Double.isNaN(lastAnchorCorrDeg) ? "n/a"
                : String.format(Locale.US, "%+.2f deg (%d rej)", lastAnchorCorrDeg, anchorRejectEvents);
    }

    /** The motor's boot direction-guard vote ("Turret boot dir (motor)"). */
    public String bootDirMotText() {
        return !bootDirMotChecked
                ? String.format(Locale.US, "pending hand-move (mot %+.1f deg)", bootDirSumMotDeg)
                : String.format(Locale.US, "done (mot %+.1f deg)", bootDirSumMotDeg);
    }

    /** The tracker's slice of the fault string (CRT crosscheck + direction + motor-encoder). */
    public String faultFragment() {
        StringBuilder sb = new StringBuilder();
        if (faultEncoderDeadAtBoot) sb.append("ENCODER_DEAD_AT_BOOT ");
        if (faultCrtCrosscheck) sb.append("CRT_CROSSCHECK ");
        if (faultEncoderDirection) sb.append("B_DIR_MISMATCH ");
        if (faultMotorEnc) sb.append("MOTOR_ENC ");
        if (faultMotorEncDir) sb.append("MOTOR_ENC_DIR ");
        if (faultAbsStreamLost) sb.append("ABS_STREAM_LOST ");
        return sb.toString();
    }

    /** Clear the tracker's latched faults and re-arm the hybrid. NOTE: the boot-wiring faults
     *  (ENCODER_DEAD_AT_BOOT, B_DIR_MISMATCH) are deliberately NOT clearable — re-zeroing
     *  cannot fix wiring; only an OpMode restart re-runs the boot checks. */
    public void clearFaults() {
        faultCrtCrosscheck = false;
        crtCrosscheckFails = 0;
        faultMotorEnc = false;
        faultMotorEncDir = false;
        faultAbsStreamLost = false;
        absLostStreak = 0;
        absGoodStreak = 0;
        deadStreak = 0;
        dirStreak = 0;
        watchDeadStreak = 0;
        absGlitchEvents = 0;
        feedbackFailures = 0;
        if (absoluteSource instanceof OctoQuadAbsoluteSource) {
            ((OctoQuadAbsoluteSource) absoluteSource).oorEvents = 0;
            ((OctoQuadAbsoluteSource) absoluteSource).crcFailEvents = 0;
        }
        motorEncMode = resolveMotorEncMode();
    }

    /**
     * What a re-zero does to the tracker: clear the CRT-crosscheck latch and the
     * hybrid motor-encoder faults (a fresh absolute anchor re-arms the hybrid),
     * but keep the diagnostic glitch counters — they describe the session, not
     * the current anchor.
     */
    public void resetForRezero() {
        faultCrtCrosscheck = false;
        crtCrosscheckFails = 0;
        crtCrosscheckErrDeg = Double.NaN;
        faultMotorEnc = false;
        faultMotorEncDir = false;
        motorEncMode = resolveMotorEncMode();
    }

    /** Tiny local timer (android.os.SystemClock-free, ElapsedTime-style). */
    private static final class ElapsedTimeTimer {
        private final long startNs = System.nanoTime();
        public double seconds() {
            return (System.nanoTime() - startNs) / 1.0e9;
        }
    }
}
