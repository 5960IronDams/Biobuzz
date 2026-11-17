package org.firstinspires.ftc.teamcode.Subsystems.turret;

import static org.firstinspires.ftc.teamcode.WattageLib.lib.killaUtils.clamp;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Locale;
import java.util.OptionalDouble;

/**
 * Fuses the turret's three encoders into one continuous angle (deg, 0 = forward, +CCW)
 * and velocity (deg/s) for {@link TurretPivot}:
 * <ul>
 *   <li>Motor quadrature encoder — exact per-loop motion deltas in {@code ACTIVE} mode.</li>
 *   <li>Encoder B (32T) PWM absolute — anchors at rest and provides {@code WATCH}/{@code OFF} tracking.</li>
 *   <li>Encoder A (31T) PWM absolute — CRT boot solving and per-loop cross-check.</li>
 * </ul>
 */
@Configurable
public class TurretTracker {

    // =========================================================================
    // Tuning (Panels -> TurretTracker)
    // =========================================================================

    /** Median filter window on raw absolute reads (3 drops single-sample glitches; 1 = off). */
    public static int ABS_MEDIAN_WINDOW = 3;

    /** Maximum plausible turret angular speed (deg/s) for filtering single-loop PWM glitches. */
    public static double ABS_MAX_PLAUSIBLE_DPS = 2000.0;

    /** Consecutive rejected PWM steps before force-reanchoring to the raw reading. */
    public static int ABS_REJECT_REANCHOR = 8;

    /** Number of boot reads (12 ms apart) used to verify absolute encoder stability. */
    public static int ABS_BOOT_READS = 8;

    /** Maximum allowed spread (deg) across boot stability reads. */
    public static double ABS_BOOT_SPREAD_DEG = 3.0;

    /** Velocity EMA smoothing weight (0.05..1.0; lower smooths more, 1.0 = raw derivative). */
    public static double VELOCITY_EMA_ALPHA = 1.0;

    /** Time (s) without a PWM change before held velocity relaxes to 0. */
    public static double ABS_VELOCITY_STALE_S = 0.30;

    // ---- Hybrid motor-encoder tracking ----

    /** 5203 built-in encoder counts per motor-shaft revolution (before gearbox). */
    public static double MOTOR_ENC_TICKS_PER_MOTOR_REV = 28.0;
    /** 5203 planetary gearbox ratio (19.2:1 = 3591/187). */
    public static double MOTOR_PLANETARY_RATIO = 3591.0 / 187.0;

    /** True enables hybrid motor+PWM tracking; false forces PWM-only fallback mode. */
    public static boolean MOTOR_ENC_HYBRID = true;

    /** Motor encoder polarity (+1 or -1) so positive tick deltas match CCW turret motion. */
    public static double MOTOR_ENC_POLARITY = 1.0;

    /** Rejects at-rest anchor corrections larger than this (deg). */
    public static double MOTOR_ENC_ANCHOR_REJECT_DEG = 25.0;
    /** Maximum turret speed (deg/s) for applying at-rest PWM hard re-anchoring. */
    public static double MOTOR_ENC_REST_ANCHOR_VEL_DPS = 8.0;
    /** Minimum time (s) PWM must stay unchanged before at-rest re-anchoring snaps position. */
    public static double MOTOR_ENC_REST_SETTLE_S = 0.12;
    /** Consecutive motor-encoder read exceptions before demoting hybrid tracking to PWM-only. */
    public static int MOTOR_ENC_READ_FAIL_LIMIT = 3;

    /** Consecutive failed feedback reads before declaring feedback lost and braking. */
    public static int FEEDBACK_FAIL_LIMIT = 10;

    /**
     * Consecutive loops without an absolute PWM read before latching ABS_STREAM_LOST
     * on HardwareHealth (auto-clears after the same number of good reads).
     */
    public static int ABS_STREAM_LOST_LOOPS = 50;

    /** Maximum physically plausible turret speed (deg/s) for detecting lost encoder wraps. */
    public static double CONTINUITY_MAX_DPS = 1200.0;
    /** Consecutive over-speed steps before latching position divergence. */
    public static int CONTINUITY_LIMIT = 2;

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
     * motion and PWM anchors at rest. OFF = PWM-only tracking fallback.
     */
    private enum MotEncMode { OFF, ACTIVE }
    private MotEncMode motorEncMode = MotEncMode.OFF;
    /** Why the hybrid is OFF (telemetry; empty when ACTIVE). */
    private String motorEncOffReason = "";
    /** False until the first motor-encoder read after boot/re-zero (delta baseline). */
    private boolean motorEncPrimed = false;
    private int motorEncLastPos = 0;
    /** Motor-encoder implausible steps rejected (counter glitch / mode stomp). */
    private int motorEncGlitchEvents = 0;
    private int motorEncConsecGlitches = 0;
    private int motorEncReadFails = 0;
    /** Last anchor correction (deg, prediction-vs-absolute) — telemetry/slip visibility. */
    private double lastAnchorCorrDeg = Double.NaN;
    private int consecutiveAnchorRejects = 0;
    private int anchorRejectEvents = 0;
    /** Latched: motor encoder failed repeated hardware reads. */
    private boolean faultMotorEnc = false;
    /** Latched: the absolute (OctoQuad/encoder-B) stream stopped being read — DS failure-board alarm. */
    private boolean faultAbsStreamLost = false;
    private boolean positionDiverged = false;
    private int continuityStreak = 0;
    /** Consecutive loops without an absolute read (drives faultAbsStreamLost). */
    private int absLostStreak = 0;
    /** Consecutive good absolute reads (auto-clears faultAbsStreamLost). */
    private int absGoodStreak = 0;

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
     * Checks during init hand-motion that B (flipped), A, and the motor encoder
     * all move in the same direction.
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
    }

    // =========================================================================
    // Per-loop update
    // =========================================================================

    /**
     * Feedback pipeline — call once per loop BEFORE the PID runs. In hybrid
     * ACTIVE mode the motor encoder integrates per-loop motion while the
     * OctoQuad absolute anchors at rest and cross-checks; in OFF mode the
     * encoder-B PWM path owns tracking.
     */
    public void update(double dtSec) {
        boolean motorOk = true;
        if (motorEncMode == MotEncMode.ACTIVE) {
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
        double deltaRevs = TurretCrt.forwardDiffRevs(lastAbsRevs, revs);

        if (motorEncMode == MotEncMode.ACTIVE) {
            lastAbsRevs = revs;
            double now = timer.seconds();
            if (deltaRevs != 0.0) {
                absLastChangeTime = now;
            }
            boolean pwmSettled = Double.isNaN(absLastChangeTime)
                    || (now - absLastChangeTime) >= MOTOR_ENC_REST_SETTLE_S;
            if (Math.abs(velocityDegPerSec) <= MOTOR_ENC_REST_ANCHOR_VEL_DPS && pwmSettled) {
                anchorAtRest(revs);
            } else if (deltaRevs != 0.0) {
                recordMovingAnchorCorr(revs);
            }
            crossCheckAgainstPartner();
            return true;
        }

        // PWM-only tracking (also active during WATCH): reject steps > 0.5 enc-rev or implausible speed.
        double maxStepRevs = Math.min(0.5,
                Math.abs(ABS_MAX_PLAUSIBLE_DPS) * Math.max(dtSec, 1e-3) / 360.0
                        * revsPerTurretB);
        if (Math.abs(deltaRevs) > maxStepRevs) {
            consecutiveRejects++;
            absGlitchEvents++;
            if (consecutiveRejects >= ABS_REJECT_REANCHOR) {
                Log.e("IronLog", "TurretTracker ABS re-anchor after " + consecutiveRejects
                        + " implausible steps (wiring/config suspicion)");
                consecutiveRejects = 0;
                lastAbsRevs = revs;
            }
            return true;
        }
        consecutiveRejects = 0;

        if (Math.abs(deltaRevs) > 0.25) {
            wrapEvents++;
            Log.w("IronLog", "TurretTracker suspicious absolute step: "
                    + String.format(Locale.US, "%+.3f revs (%+.1f deg turret)",
                    deltaRevs, deltaRevs * 360.0 * revsPerTurretB));
        }
        double deltaDeg = deltaRevs * 360.0 / revsPerTurretB;
        {
            double spanDt = Double.isNaN(absLastChangeTime)
                    ? dtSec : clamp(timer.seconds() - absLastChangeTime, Math.max(dtSec, 1e-3), 0.5);
            checkPhysicalContinuity(deltaDeg, spanDt);
        }
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
     * Checks each integrated step against {@link #CONTINUITY_MAX_DPS} to catch
     * lost encoder wraps; latches positionDiverged after {@link #CONTINUITY_LIMIT} hits.
     */
    private void checkPhysicalContinuity(double deltaDeg, double dtSec) {
        if (Double.isNaN(deltaDeg) || dtSec <= 0) {
            return;
        }
        double impliedDps = Math.abs(deltaDeg) / dtSec;
        if (impliedDps > CONTINUITY_MAX_DPS) {
            continuityStreak++;
            if (continuityStreak >= CONTINUITY_LIMIT) {
                if (!positionDiverged) {
                    Log.e("IronLog", "TurretTracker POSITION DISCONTINUITY: step "
                            + String.format(Locale.US, "%+.1f deg", deltaDeg)
                            + " in " + String.format(Locale.US, "%.0f ms", dtSec * 1000.0)
                            + " implies " + String.format(Locale.US, "%.0f", impliedDps)
                            + " deg/s, beyond the mechanism's "
                            + String.format(Locale.US, "%.0f", CONTINUITY_MAX_DPS)
                            + " deg/s ceiling — whole wraps were lost (braking)");
                }
                positionDiverged = true;
            }
        } else {
            continuityStreak = 0;
            positionDiverged = false;
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
        }
        return true;
    }

    /**
     * HYBRID at-rest anchor: once the turret has settled and the ~10 Hz PWM
     * reading has stabilized, snaps the tracked position to the wrap-resolved
     * absolute reading to erase backlash and integration drift before firing.
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
            }
            return;
        }
        consecutiveAnchorRejects = 0;
        lastAnchorCorrDeg = corrDeg;
        unwrappedDeg = anchoredRevs * 360.0 / revsPerTurretB;
    }

    /**
     * Records the moving PWM-vs-motor residual for telemetry only. Does not
     * demote to PWM-only mode, because ~10 Hz PWM staleness during quick
     * reversals makes interval comparisons unreliable.
     */
    private void recordMovingAnchorCorr(double revsFresh) {
        double predRevs = unwrappedDeg * revsPerTurretB / 360.0;
        double anchoredRevs = revsFresh + Math.round(predRevs - revsFresh);
        lastAnchorCorrDeg = (anchoredRevs - predRevs) * 360.0 / revsPerTurretB;
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

    /** Mode selection at boot/re-zero; read-failure latches keep hybrid OFF until cleared. */
    private MotEncMode resolveMotorEncMode() {
        if (!MOTOR_ENC_HYBRID) {
            motorEncOffReason = "MOTOR_ENC_HYBRID=false (encoder-B-only mode)";
            return MotEncMode.OFF;
        }
        if (faultMotorEnc) {
            motorEncOffReason = "read fault latched (re-zero or clearFaults to re-arm)";
            return MotEncMode.OFF;
        }
        return MotEncMode.ACTIVE;
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

    /** Latched motor-encoder read fault (hybrid demotion reason). */
    public boolean isMotorEncFaulted() {
        return faultMotorEnc;
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

    /**
     * True when tracked position has exceeded physical continuity limits (lost wraps);
     * auto-clears once steps return within normal speed bounds.
     */
    public boolean isPositionDiverged() {
        return positionDiverged;
    }

    // ---- One-line telemetry texts (keep TurretPivot.periodic readable) ----

    /** One-line hybrid status for telemetry ("Turret mot-enc"). */
    public String modeText() {
        if (motorEncMode == MotEncMode.ACTIVE) {
            return String.format(Locale.US, "ACTIVE (pol %+.0f, %.1f ticks/turret-rev)",
                    MOTOR_ENC_POLARITY, motorEncTicksPerTurretRev());
        }
        return "OFF — " + motorEncOffReason;
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

    /** The tracker's slice of the fault string (CRT crosscheck + direction + motor-encoder). */
    public String faultFragment() {
        StringBuilder sb = new StringBuilder();
        if (faultEncoderDeadAtBoot) sb.append("ENCODER_DEAD_AT_BOOT ");
        if (faultCrtCrosscheck) sb.append("CRT_CROSSCHECK ");
        if (faultEncoderDirection) sb.append("B_DIR_MISMATCH ");
        if (faultMotorEnc) sb.append("MOTOR_ENC ");
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
        faultAbsStreamLost = false;
        absLostStreak = 0;
        absGoodStreak = 0;
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
     * hybrid motor-encoder read fault (a fresh absolute anchor re-arms the hybrid),
     * but keep the diagnostic glitch counters — they describe the session, not
     * the current anchor.
     */
    public void resetForRezero() {
        faultCrtCrosscheck = false;
        crtCrosscheckFails = 0;
        crtCrosscheckErrDeg = Double.NaN;
        faultMotorEnc = false;
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
