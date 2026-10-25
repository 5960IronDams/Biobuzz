package org.firstinspires.ftc.teamcode.Subsystems.turret;

import static org.firstinspires.ftc.teamcode.WattageLib.lib.killaUtils.clamp;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareHealth;
import org.firstinspires.ftc.teamcode.IronConstants;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.Locale;
import java.util.OptionalDouble;

/**
 * TurretPivot — the turret rotation subsystem, ported from the 2026-IndianaJoe
 * FRC {@code TurretPivot} (TalonFX + dual-CANcoder EasyCRT) to the FTC REV
 * Control Hub. Full porting history lives in TURRET_PORTING_NOTES.md.
 *
 * <p><b>THE SPLIT:</b> the turret is two halves. This class is the ACTING half
 * — setpoint selection, the PID, the power stage, watchdogs, telemetry. The
 * SENSING half (where the turret actually is, fused from three encoders)
 * lives in {@link TurretTracker}; the pure CRT math lives in
 * {@link TurretCrt}; the OctoQuad PWM reads live in
 * {@link OctoQuadAbsoluteSource}. Ask the tracker for a position, ask this
 * class to move.
 *
 * <p><b>HARDWARE (what this class talks to):</b>
 * <ul>
 *   <li><b>Drive:</b> one goBILDA 5203 motor (312 RPM, 19.2:1) whose output
 *       carries a 64T gear meshing the 176T turret ring — 2.75:1 overall.
 *       Positive power rotates the turret CCW (looking down).</li>
 *   <li><b>Feedback:</b> an OctoQuad (I2C) reading TWO REV Through Bore
 *       absolute encoders: encoder B (32T pinion, the tracking encoder) and
 *       encoder A (31T pinion, the CRT partner) — see
 *       {@link OctoQuadAbsoluteSource}. The 5203's built-in encoder supplies
 *       per-loop motion for the hybrid tracker.</li>
 * </ul>
 *
 * <p><b>HOW THE TURRET KNOWS WHERE IT IS:</b> summarized in the
 * {@link TurretTracker} javadoc — motor encoder integrates motion every loop
 * (hybrid ACTIVE), PWM absolutes anchor at rest / cross-check / boot-solve
 * (CRT), and any fault drops back to PWM-only encoder-B tracking at a reduced
 * power cap. This class only consumes the tracker's answers.
 *
 * <p><b>SAFETY NETS (this class):</b> software soft limits ±180° (the cable
 * routing is built for one revolution — this is INTENTIONAL, and a setpoint on
 * the far side of the ±180 seam is reached the long way around by design),
 * wrap-aware setpoint choice, an edge slow zone near the limits, a stuck
 * watchdog, a motor over-current alert, brake-on-feedback-loss, and latched
 * fault flags surfaced on the dashboard.
 *
 * <p>Hardware + its own single-subsystem commands ({@code C_GotoPositon} /
 * {@code C_Stop}, FRC names kept for the humans) — gamepad bindings live in
 * {@code KeyBindings}, multi-subsystem compositions in {@code CommandFactory},
 * and the bench bring-up OpMode is {@code TurretPivotDebug}.
 *
 * <p>Panels-tunable (Panels -&gt; TurretPivot): gains, limits, gearing, caps.
 * Hardware NAME changes need an OpMode restart; everything else applies live.
 *
 * <p>Construct once in the OpMode (auto-registers with SubsystemBase):
 * <pre>
 * turret = TurretPivot.tryCreate(this);   // null when not in the RC config
 * </pre>
 */
@Configurable
public class TurretPivot extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    public enum SubSystemState {
        IDLE,
        MOVINGTOPOSITION,
        INPOSITION,
        BRAKED,
    }

    // =========================================================================
    // Geometry
    // =========================================================================

    /** Teeth on the turret RING gear (the big gear the pinions/gear mesh). */
    public static final double TURRET_GEAR_TEETH = 176.0;
    /** Teeth on the gear mounted on the 5203 motor's output shaft. */
    public static final double MOTOR_GEAR_TEETH = 64.0;
    /** Teeth on the TRACKING encoder's (encoder B) pinion. */
    public static final double ENCODER_GEAR_TEETH = 32.0;

    /**
     * Encoder-B revolutions per ONE turret revolution (176/32 = 5.5 exactly).
     * The CRT partner ratio (176/31) lives in {@link TurretCrt}.
     */
    public static final double ENCODER_REVS_PER_TURRET_REV = TURRET_GEAR_TEETH / ENCODER_GEAR_TEETH;

    // =========================================================================
    // Drive: the goBILDA 5203 motor
    // =========================================================================

    /** 5203 motor gearbox: 19.2:1 planetary (3591/187) x external 2.75:1 (64T:176T). */
    public static double MOTOR_GEAR_RATIO = TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH;
    /** 5203 no-load speed at 12 V (RPM) — only used for the "est max speed" sanity number. */
    public static double MOTOR_NO_LOAD_RPM = 312.0;
    /**
     * Software sign flip for the MOTOR: positive POWER must rotate the turret
     * CCW. The motor encoder's own polarity is
     * {@link TurretTracker#MOTOR_ENC_POLARITY}.
     */
    public static boolean MOTOR_REVERSED = false;
    /** Lynx current alert threshold for the 5203 channel (amps). */
    public static double CURRENT_ALERT_AMPS = 20.0;

    // =========================================================================
    // Motion limits + power caps
    // =========================================================================

    /**
     * Power cap in hybrid ACTIVE mode. STAGED ON PURPOSE (bring-up plan in
     * TURRET_PORTING_NOTES.md §12.11): raise in steps 0.65 -> 0.8 -> 1.0 only
     * after a clean capture at each stage.
     */
    public static double MAX_POWER = 0.50;

    /**
     * Power cap whenever the hybrid is NOT active (WATCH, OFF, or demoted):
     * the PWM-only tracker rides a ~10 Hz PWM refresh and cannot unwrap
     * faster than ~273-310 deg/s, so full power would outrun its own
     * feedback and fold the angle.
     */
    public static double PWM_ONLY_MAX_POWER = 0.40;

    /** Soft limits (deg). The ±180 window is INTENTIONAL — one cable-safe revolution. */
    public static double MAX_ANGLE_DEG = 180.0;
    public static double MIN_ANGLE_DEG = -180.0;

    /** "At target" tolerance (deg) for the state machine and the settled-hold. */
    public static double POSITION_TOLERANCE_DEG = 2.0;

    /** Edge slow zone: within this many degrees of a soft limit, cap power (MotionMagic analog). */
    public static double EDGE_SLOW_ZONE_DEG = 20.0;
    /** Power cap while inside the slow zone. */
    public static double EDGE_MAX_POWER = 0.35;

    // =========================================================================
    // PID (input: degrees + deg/s, output: servo power) — FRC Slot0 analog
    // =========================================================================

    /** Power per degree of error. Bench-tuned (captures 024811/030728/031455). */
    public static double kP = 0.040;
    /** Power per (deg*second) of accumulated error, inside I_ZONE only. */
    public static double kI = 0.000;
    /** Power per deg/s of measured motion (derivative on measurement — no setpoint-step kick). */
    public static double kD = 0.001;
    /** Static-friction feedforward (power), added in the sign of the PID output. FRC analog: kS. */
    public static double kS = 0.060;
    /**
     * kS taper — full static-friction feedforward beyond this many degrees of
     * error, fading linearly to 0 at the target. A full-size kS kick at EVERY
     * output sign reversal around the setpoint is a sustained limit cycle.
     */
    public static double KS_RAMP_DEG = 4.0;
    /**
     * kS engages on |error| beyond this epsilon (deg), decoupled from the PID
     * output magnitude (see §12.14 — the old |out| gate silently scaled with
     * kP). The taper already fades kS to 0 at the target; the epsilon only
     * prevents direction churn on numeric noise.
     */
    public static double KS_ENGAGE_DEG = 0.25;
    /**
     * Settled-hold: inside {@link #POSITION_TOLERANCE_DEG} AND slower than
     * this (deg/s), the output goes to 0 and the 5203's BRAKE zero-power
     * behavior holds position. Chasing the last couple of degrees through the
     * loop + filter lag is what made the turret bounce after arriving.
     */
    public static double SETTLE_VEL_DPS = 15.0;
    /** Enable/disable the settled-hold cutoff (see {@link #SETTLE_VEL_DPS}). */
    public static boolean IN_BAND_HOLD = true;

    // ---- Profile-lite approach cap (§12.16 fix A) ----
    /**
     * Error-scaled power cap: the position output is additionally clamped to
     * {@code |error| * PROFILE_GAIN_PER_DEG}, floored at
     * {@link #PROFILE_MIN_POWER} and capped by the edge/mode caps. Without it
     * the turret arrives at the setpoint at full power, overshoots, and rocks
     * on the backlash. Full power beyond ~25 deg of error.
     */
    public static double PROFILE_GAIN_PER_DEG = 0.020;
    /**
     * Floor on the profile cap (power) — must exceed the mesh's breakaway
     * friction so small corrections still move: 0.060 (= kS) stalled the
     * turret 2.7 deg off target (capture 030728); 0.10 breaks away.
     */
    public static double PROFILE_MIN_POWER = 0.100;
    /** Enable/disable the profile-lite cap (true = shaped approach). */
    public static boolean PROFILE_LITE_ENABLED = true;

    /** Velocity feedforward, power per deg/s of DESIRED turret velocity (dormant in position mode). */
    public static double kV = 0.000;
    /** Integral only accumulates within this error (deg). */
    public static double I_ZONE_DEG = 5.0;
    /** Integral contribution clamp (power). */
    public static double I_MAX = 0.15;

    // =========================================================================
    // Watchdogs
    // =========================================================================

    /** Stuck watchdog arms when |applied power| exceeds this. */
    public static double STUCK_POWER_THRESHOLD = 0.15;
    /** ...and stays armed this long without meaningful motion. */
    public static double STUCK_TIME_S = 0.75;
    /** "Meaningful motion" floor for the stuck watchdog (deg/s). */
    public static double STUCK_VEL_DEG_PER_S = 2.0;
    // NOTE: FEEDBACK_FAIL_LIMIT and every tracker tuning constant (ABS_*,
    // MOTOR_ENC_*, VELOCITY_EMA_ALPHA, ...) moved to {@link TurretTracker}.

    // =========================================================================
    // Live state
    // =========================================================================

    /** The 5203 motor (the only drive since the servo was retired). */
    private final DcMotorEx motor;
    /** Tracks the last written direction so refreshMotorDirection only writes on change. */
    private boolean lastReversed = !MOTOR_REVERSED; // force one direction write at init
    /** The absolute PWM source (encoder B + CRT partner A on one OctoQuad). */
    private final AbsoluteAngleSource absoluteSource;
    /** The SENSING half: fuses the three encoders into one position estimate. */
    private final TurretTracker tracker;

    private final ElapsedTimeTimer timer = new ElapsedTimeTimer();
    private double lastLoopTime = Double.NaN;

    /** FRC analog: currentPosition — robot-oriented DEGREES (0 = forward, +CCW), continuous multi-turn. Cached from the tracker each loop. */
    private double currentPosition = 0;
    /** Raw requested angle (pre-clamp) — FRC analog: requestedPosition. */
    private double requestedPosition = 0;
    /** Safe (clamped) setpoint actually tracked — FRC analog: setPointPosition. */
    private double setPointPosition = 0;
    /** Desired turret velocity feedforward for the active setpoint (deg/s, +CCW). */
    private double desiredVelocityDps = 0;
    /** True once a goto has been commanded and should keep being held (until BRAKE). */
    private boolean setpointActive = false;
    private double iAccum = 0;

    /** FRC parity fields. */
    public SubSystemState State = SubSystemState.IDLE;
    public boolean TurretPivot_OK = false;
    public boolean ENCODER_OK = false;

    // ---- boot / sanity state (facts about HOW the boot went) ----
    private boolean bootAssumedForward = true;
    /** True once the CRT solve produced the boot angle (the two encoders agreed). */
    private boolean crtBootUsed = false;
    /** True once the absolute path delivered a boot angle (false = assumed forward). */
    private boolean absBootUsed = false;
    /** Match diff (encoder revolutions, ±0.5-centered) of the winning CRT candidate; NaN if unsolved. */
    private double crtSolveDiff = Double.NaN;
    /** Number of turret-turn candidates the boot solver considered (telemetry/debug). */
    private int crtSolveCandidates = 0;

    // ---- latched fault flags OWNED by this class (the tracker owns its own) ----
    /** Latched: commanded power with no meaningful motion for STUCK_TIME_S. */
    private boolean faultEncoderStuck = false;
    /** Latched: tracked angle escaped [MIN-eps, MAX+eps] (boot window was wrong, or limits are). */
    private boolean faultOutOfScope = false;
    /** Latched: the absolute path failed the boot stability check (wiring/config suspicion). */
    private boolean faultAbsBootUnstable = false;
    /** Latched: 5203 motor channel reported an over-current condition (Lynx alert latch). */
    private boolean faultOverCurrent = false;

    private double stuckTimer = 0;
    private double lastAppliedPower = 0;
    /** Last 5203 channel current (NaN on failed read) — telemetry. */
    private double lastMotorAmps = Double.NaN;

    /**
     * Builds the turret from the RC config. REQUIRES the 5203 motor
     * ({@code IronConstants.TurretMotorName}) and the OctoQuad
     * ({@code IronConstants.TurretOctoQuadName}) — use {@link #tryCreate} to
     * get null instead of a throw on rigs without them.
     */
    public TurretPivot(OpMode opMode) {
        // ---- Drive: the goBILDA 5203 ----
        DcMotorEx m;
        try {
            m = opMode.hardwareMap.get(DcMotorEx.class, IronConstants.TurretMotorName);
        } catch (Exception e) {
            throw new IllegalArgumentException("TurretPivot needs the 5203 motor '"
                    + IronConstants.TurretMotorName + "' in the RC config", e);
        }
        motor = m;
        // RUN_WITHOUT_ENCODER: no hub-side velocity PID — power comes straight
        // from the software PID. BRAKE zero-power behavior = active holding.
        // The built-in encoder is still counted in hardware between loops and
        // read via getCurrentPosition() (the 4-pin JST-XH cable must be seated
        // in the motor port's encoder header — an unseated cable is what
        // caused the 020255 thrash).
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        try {
            motor.setCurrentAlert(CURRENT_ALERT_AMPS, CurrentUnit.AMPS);
        } catch (Exception e) {
            Log.w("IronLog", "TurretPivot: could not arm the 5203 current alert", e);
        }

        // ---- Feedback: the OctoQuad (encoder B tracking + encoder A CRT) ----
        OctoQuad octoquad;
        try {
            octoquad = opMode.hardwareMap.get(OctoQuad.class, IronConstants.TurretOctoQuadName);
        } catch (Exception e) {
            throw new IllegalArgumentException("TurretPivot needs the OctoQuad '"
                    + IronConstants.TurretOctoQuadName + "' (I2C) in the RC config", e);
        }
        OctoQuadAbsoluteSource src;
        try {
            src = new OctoQuadAbsoluteSource(octoquad);
        } catch (Exception e) {
            throw new IllegalArgumentException("TurretPivot: OctoQuad found but init failed", e);
        }
        absoluteSource = src;
        tracker = new TurretTracker(absoluteSource, motor,
                ENCODER_REVS_PER_TURRET_REV, TurretCrt.encoderARevsPerTurretRev(), MOTOR_GEAR_RATIO);

        refreshMotorDirection();
        motor.setPower(0);

        bootZero();
        Log.i("IronLog", "TurretPivot created (gear " + ENCODER_REVS_PER_TURRET_REV
                + " enc/turret, drive: " + getDriveName()
                + ", feedback: " + tracker.feedbackSourceName()
                + ", mot-enc: " + tracker.modeText()
                + ", boot: " + (crtBootUsed ? "CRT dual-encoder solve" : "ASSUMED FORWARD") + ")");
    }

    /**
     * Null-safe constructor for OpModes that must also run without the turret
     * hardware configured (same pattern as Vision.tryCreate). Returns null when
     * the motor or the OctoQuad is missing from the RC config.
     */
    public static TurretPivot tryCreate(OpMode opMode) {
        try {
            return new TurretPivot(opMode);
        } catch (Exception e) {
            Log.w("IronLog", "TurretPivot skipped — needs '" + IronConstants.TurretMotorName
                    + "' (5203 motor) and '" + IronConstants.TurretOctoQuadName
                    + "' (OctoQuad on I2C) in the RC config", e);
            // Driver-station failure board: on the comp bot a missing turret
            // IS a match-critical condition (no aiming) — the DS gets the red
            // banner before PLAY. Bench rigs (CompBot=false) legitimately run
            // without the turret and stay silent.
            if (IronConstants.CompBot) {
                String detail = e.getMessage();
                HardwareHealth.get().report("TURRET", "absent at init ("
                        + e.getClass().getSimpleName()
                        + (detail == null ? ")" : ": " + detail + ")"));
            }
            return null;
        }
    }

    // =========================================================================
    // Boot logic
    // =========================================================================

    /**
     * Boot zero: run the CRT dual-encoder solve (TurretCrt) to find the true
     * multi-turn turret angle at init. If the solve is unavailable or fails,
     * fall back to "the turret points forward right now" (accurate to a few
     * degrees) and log loudly — the dashboard shows what to check.
     *
     * <p>This is the ORCHESTRATION only: the stability check, the anchor and
     * the hybrid re-prime live in the tracker ({@link TurretTracker}).
     */
    private void bootZero() {
        OptionalDouble abs = absoluteSource.readRevs();
        double bootDeg = 0;
        bootAssumedForward = true;
        absBootUsed = false;
        crtBootUsed = false;
        crtSolveDiff = Double.NaN;
        crtSolveCandidates = 0;

        // Boot stability check: a handful of reads must agree — the cheapest
        // possible "the absolute path is healthy" proof.
        faultAbsBootUnstable = !tracker.checkAbsoluteStabilityAtBoot();

        // ---- CRT boot solve (both channels from the SAME bulk read) ----
        OptionalDouble crossRevs = OptionalDouble.empty();
        OptionalDouble primaryRaw = OptionalDouble.empty();
        if (absoluteSource instanceof OctoQuadAbsoluteSource) {
            OctoQuadAbsoluteSource octoSrc = (OctoQuadAbsoluteSource) absoluteSource;
            crossRevs = octoSrc.readCrossRevs();
            primaryRaw = octoSrc.readPrimaryRawRevs();
        }
        if (abs.isPresent() && crossRevs.isPresent() && primaryRaw.isPresent()
                && TurretCrt.CRT_ZERO_CALIBRATED) {
            // Both channels go in RAW (pre-reversal): the solve subtracts the
            // zero offsets in the raw magnet frame and applies the direction
            // flips itself — the flip flags can be changed at will without
            // invalidating the zero calibration.
            TurretCrt.SolveResult solve = TurretCrt.solve(
                    crossRevs.getAsDouble(), primaryRaw.getAsDouble(),
                    OctoQuadAbsoluteSource.ENCODER_REVERSED,
                    ENCODER_REVS_PER_TURRET_REV,
                    MIN_ANGLE_DEG / 360.0, MAX_ANGLE_DEG / 360.0);
            crtSolveDiff = solve.matchDiffEncRevs;
            crtSolveCandidates = solve.candidates;
            if (solve.solved) {
                bootDeg = wrapTo180(solve.turretTurns * 360.0);
                bootAssumedForward = false;
                absBootUsed = true;
                crtBootUsed = true;
                Log.i("IronLog", "TurretPivot CRT boot solve: turret "
                        + String.format(Locale.US, "%+.2f", bootDeg)
                        + " deg (match diff " + String.format(Locale.US, "%.4f", crtSolveDiff)
                        + " enc-revs over " + crtSolveCandidates + " candidates)");
            } else {
                Log.e("IronLog", "TurretPivot CRT boot solve FAILED (best diff "
                        + String.format(Locale.US, "%.4f", Double.isNaN(crtSolveDiff) ? -1 : crtSolveDiff)
                        + " enc-revs, tol " + TurretCrt.CRT_MATCH_TOL_ENC_REVS + ", "
                        + crtSolveCandidates + " candidates) — encoders disagree beyond tolerance."
                        + " Falling back to assumed-forward; check CRT zero offsets / encoder health.");
            }
        } else {
            Log.i("IronLog", "TurretPivot boot: CRT solve unavailable (missing read or"
                    + " CRT_ZERO_CALIBRATED=false) — assuming forward. Park the turret at"
                    + " forward, note the two raw dash values (Turret CRT A raw us / Turret abs"
                    + " raw us), enter them as TurretCrt.CRT_ZERO_OFFSET_A_REV / _B_REV (us/1024),"
                    + " keep TurretCrt.CRT_ZERO_CALIBRATED = true.");
        }

        // Prime the tracked stream: start at the raw reading (offset-free; the
        // offset only enters through the boot angle below), then align the
        // reference so position() == bootDeg at this instant. Also re-primes
        // the hybrid motor-encoder path (it re-runs watch / direction
        // confirmation at the first powered motion; latched MOTOR_ENC faults
        // keep the hybrid OFF until clearFaults() / re-zero).
        tracker.anchorBoot(bootDeg, abs);

        resetPidState();
        currentPosition = getPosition();
        setPointPosition = 0;
        requestedPosition = 0;
        desiredVelocityDps = 0;
        setpointActive = false;
        State = SubSystemState.IDLE;
    }

    /**
     * Re-run the boot anchor at PLAY. Between init and start the periodic loop
     * is not running, so the turret can be hand-spun in that gap without the
     * tracked stream seeing any of it — the init-time CRT answer would then be
     * stale by however far it was moved. The CRT solve is cheap (~6 candidates)
     * and re-derives the ABSOLUTE position from the two raw encoders, so
     * re-running it at start re-anchors to wherever the turret actually is
     * when the match begins. Also clears the crosscheck fail counter/fault so
     * an init-period latch can't ride into the match. Safe to call repeatedly;
     * reads encoders only, drives nothing.
     */
    public void rezeroAtStart() {
        bootZero();
        // A fresh absolute anchor re-arms the hybrid too.
        tracker.resetForRezero();
        Log.i("IronLog", "TurretPivot re-zeroed at start: boot "
                + (crtBootUsed ? "CRT re-solved " + String.format(Locale.US, "%+.2f", getPosition()) + " deg"
                        : "ASSUMED FORWARD"));
    }

    /**
     * Soft re-zero: declare "the turret points forward RIGHT NOW" without a
     * hardware counter reset (no twitch, works mid-run). This is the recovery
     * for "we booted with the turret half-turned" — re-point forward by hand
     * and call this (bound to a button in TurretPivotDebug). The log suggests
     * the RAW readings to enter as the permanent CRT zero offsets.
     */
    public void reZeroAtForward() {
        tracker.zeroHere();
        resetPidState();
        requestedPosition = 0;
        setPointPosition = 0;
        desiredVelocityDps = 0;
        setpointActive = false;
        State = SubSystemState.IDLE;
        faultOutOfScope = false;
        bootAssumedForward = true;
        Log.i("IronLog", "TurretPivot re-zeroed at current position (assumed forward)");
        // Calibration helper: whatever the two encoders read right now IS the
        // "forward" pair — suggest both so this re-zero can be made permanent.
        if (absoluteSource instanceof OctoQuadAbsoluteSource) {
            OctoQuadAbsoluteSource src = (OctoQuadAbsoluteSource) absoluteSource;
            OptionalDouble rawB = src.readPrimaryRawRevs();
            OptionalDouble rawA = src.readCrossRevs();
            if (rawB.isPresent() && rawA.isPresent()) {
                Log.i("IronLog", "TurretPivot: to make this zero permanent set"
                        + " TurretCrt.CRT_ZERO_OFFSET_B_REV = "
                        + String.format(Locale.US, "%.4f", rawB.getAsDouble())
                        + " and TurretCrt.CRT_ZERO_OFFSET_A_REV = "
                        + String.format(Locale.US, "%.4f", rawA.getAsDouble())
                        + " (Panels -> TurretCrt)");
            }
        }
    }

    /**
     * Call from an OpMode's init_loop(): boot "is the encoder alive" check —
     * the humans hand-move the turret a little and we look for the feedback to
     * respond. (Deliberately a human-powered check — the turret is not
     * auto-wiggled at init.) Delegates to the tracker.
     */
    public void pollBootEncoderCheck() {
        tracker.pollBootEncoderCheck();
    }

    // =========================================================================
    // Public control API (FRC names kept where the humans already know them)
    // =========================================================================

    /** Get the current angle in robot-oriented degrees. 0 is forward, positive is CCW, negative is CW. Continuous multi-turn. */
    public double getPosition() {
        return tracker.position();
    }

    /** FRC parity alias of {@link #getPosition()}. */
    public double getRawAngle() {
        return getPosition();
    }

    /** Measured turret velocity (deg/s, +CCW). */
    public double getVelocityDegPerSec() {
        return tracker.velocityDegPerSec();
    }

    /**
     * The tracked continuous TURRET angle in raw form (degrees since the boot
     * zero, before the soft zero is subtracted) — bench/telemetry tool for
     * watching the stream itself move.
     */
    public double getUnwrappedDeg() {
        return tracker.unwrapped();
    }

    /** True once the boot hand-move check saw the encoder actually counting. */
    public boolean isBootEncoderVerified() {
        return tracker.isBootEncoderVerified();
    }

    /** True once the boot direction guard completed (~15+ deg of hand motion on both channels). */
    public boolean isBootDirectionChecked() {
        return tracker.isBootDirectionChecked();
    }

    /** True when the boot direction guard caught a flip/direction mismatch (see TurretTracker). */
    public boolean isBootDirectionMismatch() {
        return tracker.isBootDirectionMismatch();
    }

    /** Concatenated latched fault flags (or "none") — telemetry/debug OpModes. */
    public String getFaults() {
        return faultString();
    }

    /** Latest raw absolute reading in degrees [0,360) (NaN when unavailable) — zero-calibration helper. */
    public double getAbsoluteRawDeg() {
        return tracker.lastAbsRawDeg();
    }

    /** Latest raw pulse width in microseconds from the OctoQuad (Integer.MIN_VALUE = never read). */
    public int getAbsoluteRawUs() {
        return absoluteSource instanceof OctoQuadAbsoluteSource
                ? ((OctoQuadAbsoluteSource) absoluteSource).getLastRawUs() : Integer.MIN_VALUE;
    }

    /** The OctoQuad's actual applied config echo (bank, pulse window, wrap tracking), or null. */
    public String getAbsoluteConfigEcho() {
        return absoluteSource instanceof OctoQuadAbsoluteSource
                ? ((OctoQuadAbsoluteSource) absoluteSource).getConfigEcho() : null;
    }

    /** Diagnostic counters {oor, crcFail, glitch} from the absolute path. */
    public int[] getAbsoluteDiagCounters() {
        return tracker.absoluteDiagCounters();
    }

    // ---- CRT dual-encoder bench helpers ----

    /** Latest raw pulse width (µs) from the CRT partner channel (encoder A); Integer.MIN_VALUE = never read. */
    public int getCrossRawUs() {
        return absoluteSource instanceof OctoQuadAbsoluteSource
                ? ((OctoQuadAbsoluteSource) absoluteSource).getCrossRawUs() : Integer.MIN_VALUE;
    }

    /** True when the boot used the CRT dual-encoder solve (both encoders agreed). */
    public boolean isCrtBootUsed() {
        return crtBootUsed;
    }

    /** Match diff (encoder revolutions) of the winning CRT candidate; NaN when unsolved/not attempted. */
    public double getCrtSolveDiff() {
        return crtSolveDiff;
    }

    /** Turret-turn candidates the boot solver considered (telemetry/debug). */
    public int getCrtSolveCandidates() {
        return crtSolveCandidates;
    }

    /** Last cross-check error in turret degrees (NaN = not checked yet / skipped / no partner). */
    public double getCrtCrosscheckErrDeg() {
        return tracker.crtCrosscheckErrDeg();
    }

    /** True when the encoder-A cross-check latched a failure (slipped magnet / drifting chain). */
    public boolean isCrtCrosscheckFaulted() {
        return tracker.isCrtCrosscheckFaulted();
    }

    /** Which feedback path is active ("OctoQuad ch6 PWM abs..."). */
    public String getFeedbackSourceName() {
        return tracker.feedbackSourceName();
    }

    /** Which drive is active (the 5203 description). */
    public String getDriveName() {
        return "goBILDA 5203 '" + IronConstants.TurretMotorName
                + "' (gear " + MOTOR_GEAR_RATIO + ":1)";
    }

    /** Last 5203 channel current in amps (NaN on failed read) — bench telemetry. */
    public double getMotorAmps() {
        return lastMotorAmps;
    }

    /**
     * Nominal no-load turret speed estimate at full power:
     * {@code MOTOR_NO_LOAD_RPM / MOTOR_GEAR_RATIO}, in deg/s. Under load and
     * at MAX_POWER it will be lower — treat it as an upper bound / sanity
     * number when choosing {@link #MOTOR_GEAR_RATIO}.
     */
    public double maxTurretSpeedDps() {
        return Math.abs(MOTOR_NO_LOAD_RPM) * 360.0 / 60.0 / Math.max(1e-6, Math.abs(MOTOR_GEAR_RATIO));
    }

    public double getSetpoint() {
        return setPointPosition;
    }

    public double getRequestedPosition() {
        return requestedPosition;
    }

    /** True if the turret is at the setpoint within tolerance. Also updates {@link #State} (FRC behavior). BRAKED always reads true. */
    public boolean atSetpoint() {
        if (State == SubSystemState.BRAKED) {
            return true;
        }
        boolean at = Math.abs(setPointPosition - currentPosition) <= POSITION_TOLERANCE_DEG;
        State = at ? SubSystemState.INPOSITION : SubSystemState.MOVINGTOPOSITION;
        return at;
    }

    /**
     * Move the turret to a safe angle within the soft limits — FRC GotoSafeAngle
     * verbatim: if the requested angle is out of bounds, the wrap-aware
     * {@link #chooseTurretSetpointDeg} picks the equivalent/nearest valid angle
     * (this is the "knowing when to turn back around" logic).
     */
    public void GotoSafeAngle(double angleDeg) {
        GotoSafeAngle(angleDeg, 0.0);
    }

    /**
     * FRC parity: move to a safe angle with a velocity feedforward (deg/s,
     * +CCW) so the turret can track a MOVING target with near-zero lag — the
     * PID then only corrects disturbances instead of generating all the torque.
     * Pass 0 for stationary shots.
     */
    public void GotoSafeAngle(double angleDeg, double velocityDps) {
        double safeAngle = chooseTurretSetpointDeg(angleDeg, getPosition(), MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        GotoPositionDeg(safeAngle, velocityDps);
    }

    /** Relative jog of the tracked setpoint, clamped to the soft limits (debug OpMode + KeyBindings helper). */
    public void NudgeDeg(double deltaDeg) {
        // When position mode is not engaged, the stored setpoint is a stale
        // default (0) — jogging from it while the turret sits elsewhere
        // commands a huge move at full power (the 2026-10-23 spin: boot at
        // +88 deg, first dpad tap aimed at -4 deg = a 92-degree blast-off).
        // Jog from wherever the turret actually is instead.
        double base = setpointActive ? setPointPosition : getPosition();
        GotoSafeAngle(base + deltaDeg);
    }

    /**
     * FRC GotoPosition / GotoPositionWithFeedForward analog: command an
     * (already safety-selected) angle and engage the PID hold. The hold keeps
     * running in periodic() until BRAKE — same as the TalonFX holding its last
     * MotionMagic setpoint.
     */
    private void GotoPositionDeg(double wantedDeg, double velocityDps) {
        State = SubSystemState.MOVINGTOPOSITION;
        requestedPosition = wantedDeg;
        setPointPosition = wantedDeg;
        desiredVelocityDps = velocityDps;
        setpointActive = true;
        // Re-engage active holding: BRAKE() left the drive in FLOAT (neutral),
        // so the first goto after a release has to put BRAKE back on.
        if (motor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * FRC BRAKE (kept name): release the PID and return the drive to NEUTRAL —
     * 0 power AND the 5203's FLOAT zero-power behavior, so nothing is
     * electrically holding the turret any more. This is what runs when the aim
     * command ends (trigger release): before, the command ended but the motor
     * was left in BRAKE with the PID still latched, so the turret stayed
     * actively locked wherever the driver had been aiming and had to be
     * pushed to move. The next goto switches back to BRAKE + PID.
     */
    public void BRAKE() {
        State = SubSystemState.BRAKED;
        setpointActive = false;
        iAccum = 0;
        lastAppliedPower = 0;
        desiredVelocityDps = 0;
        stuckTimer = 0;
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
    }

    public void clearFaults() {
        faultEncoderStuck = false;
        faultOutOfScope = false;
        faultOverCurrent = false;
        faultAbsBootUnstable = false;
        // Clear the tracker's faults and re-arm the hybrid.
        tracker.clearFaults();
        Log.i("IronLog", "TurretPivot faults cleared");
    }

    // ---- Commands (FRC parity names; single-subsystem per team convention) ----

    /** FRC parity: hold the requirement until the turret arrives at the angle. */
    public Command C_GotoPositon(double positionDeg) {
        return new FunctionalCommand(
                // OnInit
                () -> GotoSafeAngle(positionDeg),
                // OnExecute (PID hold runs in periodic())
                () -> { },
                // OnEnd
                interrupted -> { },
                // IsFinished
                () -> atSetpoint(),
                // Requirement
                this
        ).setName("TurretGotoPosition");
    }

    /** FRC parity: stop the drive and return the motor to neutral (PID released). */
    public Command C_Stop() {
        return new InstantCommand(this::BRAKE).setName("StopTurret");
    }

    // =========================================================================
    // Core math (ported from FRC TurretPivot, verbatim where noted)
    // =========================================================================

    /**
     * FRC parity: wrap the shortest-path error into [-180, 180].
     * Used ONLY for setpoint choice — the PID itself runs on the plain
     * (unwrapped) error, exactly like the TalonFX did.
     */
    private static double angleErrorDeg(double setpointDeg, double currentDeg) {
        return inputModulus(setpointDeg - currentDeg, -180.0, 180.0);
    }

    /**
     * FRC chooseTurretSetpointDeg, ported verbatim. Will move the turret to a
     * safe angle within the defined limits: takes the shortest-path candidate
     * toward the requested angle and, if it is beyond a limit, tries the
     * +/-360 equivalent before falling back to a clamp — so an out-of-range
     * request turns the turret back around instead of commanding it into the
     * stop. (This is also why a setpoint on the far side of the ±180 seam is
     * reached the long way — the short path would leave the allowed window.)
     */
    private static double chooseTurretSetpointDeg(double requestedDeg, double currentDeg,
                                                  double minDeg, double maxDeg) {
        double candidate = currentDeg + angleErrorDeg(requestedDeg, currentDeg);

        if (candidate < minDeg) {
            double alt = candidate + 360.0;
            if (alt <= maxDeg) {
                candidate = alt;
            }
        } else if (candidate > maxDeg) {
            double alt = candidate - 360.0;
            if (alt >= minDeg) {
                candidate = alt;
            }
        }
        return clamp(candidate, minDeg, maxDeg);
    }

    /** WPILib MathUtil.inputModulus, ported (no WPILib on the Control Hub). */
    private static double inputModulus(double input, double minimum, double maximum) {
        double modulus = maximum - minimum;
        if (modulus <= 0) {
            throw new IllegalArgumentException("inputModulus: maximum must be greater than minimum");
        }
        double result = (input - minimum) % modulus;
        if (result < 0) {
            result += modulus;
        }
        return result + minimum;
    }

    /** Wrap any angle to [-180, 180) — used by the absolute boot solve. */
    private static double wrapTo180(double deg) {
        return inputModulus(deg, -180.0, 180.0);
    }

    private void resetPidState() {
        iAccum = 0;
        stuckTimer = 0;
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void periodic() {
        double now = timer.seconds();
        double rawDt = Double.isNaN(lastLoopTime) ? 0.02 : Math.max(0, now - lastLoopTime);
        double dtSec = Math.min(rawDt, 0.25); // Panels pause shouldn't slingshot the PID
        lastLoopTime = now;

        refreshMotorDirection(); // live-tune support: Panels edits apply without restart
        tracker.update(dtSec);   // SENSING: the tracker advances the position estimate

        // Feedback-loss reaction stays HERE (the tracker never drives):
        // sustained loss of both streams = no motion. Same philosophy as the
        // FRC encoder checks — but single I2C hiccups are tolerated.
        if (tracker.feedbackLost() && setpointActive) {
            Log.e("IronLog", "TurretPivot feedback lost for " + tracker.feedbackFailureCount()
                    + " consecutive reads — braking");
            BRAKE();
        }

        currentPosition = tracker.position();
        ENCODER_OK = !tracker.feedbackLost();

        // ---- Out-of-scope sanity (FRC OutofScopeAlert analog) ----
        // The continuous angle escaping the soft limits means the boot window
        // was wrong (turret wasn't actually forward) or the limits are — flag
        // it loudly; the output guard below still refuses to push outward.
        if (currentPosition > MAX_ANGLE_DEG + POSITION_TOLERANCE_DEG
                || currentPosition < MIN_ANGLE_DEG - POSITION_TOLERANCE_DEG) {
            if (!faultOutOfScope) {
                Log.e("IronLog", "TurretPivot OUT OF SCOPE: " + String.format(Locale.US, "%.2f", currentPosition)
                        + " deg outside [" + MIN_ANGLE_DEG + ", " + MAX_ANGLE_DEG + "]"
                        + (bootAssumedForward ? " — boot 'forward' assumption was wrong; re-zero!" : ""));
            }
            faultOutOfScope = true;
        }

        atSetpoint(); // keep State fresh every loop (FRC SetCurrentState analog)
        driveOutput(dtSec);
        monitorDrive();

        TurretPivot_OK = ENCODER_OK && !faultEncoderStuck && !tracker.isEncoderDeadAtBoot();

        // Driver-station failure board: one heartbeat or one fault line per
        // loop; the full per-loop diagnostics stay on the Panels telemetry
        // below. The latched fault string is exactly the "critical" set.
        String faultStr = faultString();
        HardwareHealth.get().report("TURRET",
                faultStr.equals("none") ? null : faultStr);

        // NOTE: addData only, no update() here. Panels TelemetryManager sends AND
        // clears its line buffer on every update(), so >1 update per loop sends
        // partial frames (flicker) and drops lines. RobotMain.flushTelemetry()
        // does the single end-of-loop flush.
        telemetry.addData("Turret heading (robot rel)", String.format(Locale.US, "%+.2f deg", currentPosition));
        telemetry.addData("Turret setpoint deg", String.format(Locale.US, "%+.2f (req %+.2f)", setPointPosition, requestedPosition));
        telemetry.addData("Turret state", State);
        telemetry.addData("Turret velocity deg/s", String.format(Locale.US, "%+.1f", tracker.velocityDegPerSec()));
        telemetry.addData("Turret power", String.format(Locale.US, "%+.3f", lastAppliedPower));
        telemetry.addData("Turret drive", getDriveName());
        telemetry.addData("Turret motor current", Double.isNaN(lastMotorAmps)
                ? "n/a" : String.format(Locale.US, "%.2f A (alert %.0f)", lastMotorAmps, CURRENT_ALERT_AMPS));
        telemetry.addData("Turret est max speed", String.format(Locale.US, "%.0f deg/s", maxTurretSpeedDps()));
        telemetry.addData("Turret feedback", tracker.feedbackSourceName());
        if (absoluteSource instanceof OctoQuadAbsoluteSource) {
            OctoQuadAbsoluteSource src = (OctoQuadAbsoluteSource) absoluteSource;
            telemetry.addData("Turret abs raw us", src.getLastRawUs() == Integer.MIN_VALUE
                    ? "n/a" : String.valueOf(src.getLastRawUs()));
            int[] diag = tracker.absoluteDiagCounters();
            telemetry.addData("Turret abs OOR/crc/glitch", diag[0] + " / " + diag[1] + " / " + diag[2]);
        }
        telemetry.addData("Turret abs raw deg", Double.isNaN(tracker.lastAbsRawDeg())
                ? "n/a" : String.format(Locale.US, "%.1f", tracker.lastAbsRawDeg()));
        // ---- CRT telemetry ----
        if (absoluteSource instanceof OctoQuadAbsoluteSource) {
            OctoQuadAbsoluteSource src = (OctoQuadAbsoluteSource) absoluteSource;
            telemetry.addData("Turret CRT A raw us", src.getCrossRawUs() == Integer.MIN_VALUE
                    ? "n/a" : String.valueOf(src.getCrossRawUs()));
            telemetry.addData("Turret CRT A OOR/crc",
                    src.oorEventsA + " / " + src.crcFailEventsA);
        }
        telemetry.addData("Turret CRT boot", crtBootUsed
                ? String.format(Locale.US, "SOLVED (diff %.4f enc-revs, %d candidates)",
                crtSolveDiff, crtSolveCandidates)
                : (TurretCrt.CRT_ZERO_CALIBRATED
                        ? "not used (solve failed or unavailable — assumed forward)"
                        : "NOT CALIBRATED (park turret forward, set TurretCrt.CRT_ZERO_OFFSET_A/B_REV)"));
        telemetry.addData("Turret CRT crosscheck", Double.isNaN(tracker.crtCrosscheckErrDeg())
                ? (TurretCrt.CRT_ZERO_CALIBRATED ? "n/a (no A read)" : "off (no CRT calibration)")
                : String.format(Locale.US, "%+.2f deg (%d fails)",
                        tracker.crtCrosscheckErrDeg(), tracker.crtCrosscheckFails()));
        telemetry.addData("Turret boot", bootAssumedForward ? "ASSUMED FORWARD"
                : (crtBootUsed ? "CRT dual-encoder solve" : "absolute (calibrated)"));
        telemetry.addData("Turret wrap/glitch events", tracker.wrapEvents());
        telemetry.addData("Turret encoder live (boot)", tracker.isBootEncoderVerified()
                ? "VERIFIED" : "not yet (hand-move turret at init)");
        telemetry.addData("Turret enc B flip (runtime)", OctoQuadAbsoluteSource.ENCODER_REVERSED);
        telemetry.addData("Turret boot direction", !tracker.isBootDirectionChecked()
                ? "pending hand-move (~15 deg, both encoders)"
                : (tracker.isBootDirectionMismatch()
                        ? "MISMATCH — ENCODER_REVERSED not applied to stream!" : "OK (B and A agree)"));
        // ---- Hybrid tracker telemetry ----
        telemetry.addData("Turret mot-enc", tracker.modeText());
        telemetry.addData("Turret mot-enc ticks", tracker.motorEncTicksText());
        telemetry.addData("Turret anchor corr", tracker.anchorCorrText());
        telemetry.addData("Turret boot dir (motor)", tracker.bootDirMotText());
        telemetry.addData("Turret faults", faultStr);
        telemetry.addData("Turret OK", TurretPivot_OK);
    }

    /**
     * The power cap that actually applies. Hybrid ACTIVE lifts the ~10 Hz
     * PWM-refresh ceiling (the motor encoder tracks per loop, the PWM absolute
     * is wrap-resolved against it); every other mode puts the encoder-B
     * PWM-only tracker back in charge, so the refresh ceiling —
     * {@link #PWM_ONLY_MAX_POWER} — returns.
     */
    private double effectiveMaxPower() {
        return tracker.hybridActive()
                ? MAX_POWER : Math.min(MAX_POWER, PWM_ONLY_MAX_POWER);
    }

    /**
     * Software PID -> motor power, with the FRC output-stage behaviors:
     * soft limits (refuse to push outward), edge slow zone, velocity/static
     * feedforward, the profile-lite approach cap, settled-hold, and the stuck
     * watchdog.
     */
    private void driveOutput(double dtSec) {
        double out = 0;

        if (setpointActive && ENCODER_OK) {
            double error = setPointPosition - currentPosition; // plain error, FRC/TalonFX semantics
            double velocityDegPerSec = tracker.velocityDegPerSec();

            // Integral, zone-limited + clamped (anti-windup).
            if (Math.abs(error) <= I_ZONE_DEG) {
                iAccum += error * dtSec * kI;
            } else {
                iAccum = 0;
            }
            iAccum = clamp(iAccum, -I_MAX, I_MAX);

            // Derivative on measurement (d_err via velocity), FRC kD analog.
            double dTerm = -kD * velocityDegPerSec;
            // Velocity feedforward for tracking moving targets (FRC kV analog),
            // plus static-friction feedforward in the direction of commanded motion.
            double ff = kV * desiredVelocityDps;

            out = kP * error + iAccum + dTerm + ff;
            // Fade kS out near the target — a full-size static-friction kick at
            // every output sign reversal around the setpoint is a sustained
            // limit cycle ("bouncing after arriving"). Full kS for real moves,
            // tapered to 0 at the target.
            double ksEff = KS_RAMP_DEG > 0
                    ? kS * clamp(Math.abs(error) / KS_RAMP_DEG, 0, 1) : kS;
            // Engage kS on real commanded motion (error-based), not on PID
            // output magnitude — the old |out| gate made kS vanish whenever
            // kP was small (§12.14). At target the taper has already faded
            // kS to ~0, so no chatter is re-introduced.
            if (Math.abs(error) > KS_ENGAGE_DEG) {
                double ksSign = Math.signum(out);
                if (ksSign == 0.0) {
                    ksSign = Math.signum(error);
                }
                out += ksSign * ksEff;
            }
            // ---- Edge slow zone (MotionMagic replacement) ----
            // Hybrid-ACTIVE lifts the PWM-refresh cap; every other mode
            // re-applies PWM_ONLY_MAX_POWER (encoder-B-only tracker).
            double cap = effectiveMaxPower();
            if (EDGE_SLOW_ZONE_DEG > 0) {
                double toMax = MAX_ANGLE_DEG - currentPosition;
                double toMin = currentPosition - MIN_ANGLE_DEG;
                double dist = error > 0 ? toMax : toMin;
                if (dist < EDGE_SLOW_ZONE_DEG) {
                    cap = Math.min(cap, EDGE_MAX_POWER);
                }
            }

            // ---- Profile-lite approach cap ----
            // Error-scaled power limit (see PROFILE_GAIN_PER_DEG): bounds the
            // arrival speed so the backlash+stiction stage is never hit at
            // full power. The edge-zone cap above feeds in as the ceiling.
            if (PROFILE_LITE_ENABLED) {
                double profCap = Math.abs(error) * PROFILE_GAIN_PER_DEG;
                if (profCap < PROFILE_MIN_POWER) {
                    profCap = PROFILE_MIN_POWER;
                }
                if (profCap < cap) {
                    cap = profCap;
                }
            }

            // ---- Soft limit output guard (TalonFX Forward/ReverseSoftLimit analog) ----
            // Refuse to push beyond the limits; turning back toward range (or
            // holding) is always allowed — this is the "knowing when to turn
            // back around" stage at the boundary itself.
            if (currentPosition >= MAX_ANGLE_DEG && out > 0) {
                out = 0;
            }
            if (currentPosition <= MIN_ANGLE_DEG && out < 0) {
                out = 0;
            }

            out = clamp(out, -cap, cap);

            // ---- Settled-hold ----
            // Inside the position tolerance AND no longer moving: stop driving
            // and let the 5203's BRAKE zero-power behavior hold. Chasing the
            // last couple of degrees through the loop + filter lag is what
            // made the turret bounce around the setpoint after arriving. Leave
            // the band or speed up and the PID re-engages (kS ramp gives
            // breakaway torque for the correction).
            if (IN_BAND_HOLD && Math.abs(error) <= POSITION_TOLERANCE_DEG
                    && Math.abs(velocityDegPerSec) <= SETTLE_VEL_DPS) {
                out = 0;
            }
        }

        lastAppliedPower = out;
        motor.setPower(out);

        // ---- Stuck watchdog (encoder/chain sanity) ----
        boolean pushing = Math.abs(lastAppliedPower) > STUCK_POWER_THRESHOLD;
        boolean moving = Math.abs(tracker.velocityDegPerSec()) > STUCK_VEL_DEG_PER_S;
        if (pushing && !moving) {
            stuckTimer += dtSec;
        } else {
            stuckTimer = 0;
        }
        if (stuckTimer > STUCK_TIME_S) {
            if (!faultEncoderStuck) {
                Log.e("IronLog", "TurretPivot STUCK: commanded power with no motion — braking "
                        + "(drive jam, gearing, or encoder not counting)");
            }
            faultEncoderStuck = true;
            BRAKE();
        }
    }

    /**
     * Drive health/current monitoring: read the 5203 channel current, arm the
     * Lynx over-current alert, and latch an OVER_CURRENT fault when the
     * controller flags it (alert only — no auto-brake; the soft limits are
     * what keep the turret out of the stops). FRC analog: stator/supply
     * current telemetry.
     */
    private void monitorDrive() {
        try {
            double amps = motor.getCurrent(CurrentUnit.AMPS);
            lastMotorAmps = amps;
            if (motor.isOverCurrent() && !faultOverCurrent) {
                Log.e("IronLog", "TurretPivot OVER CURRENT on the 5203 channel ("
                        + String.format(Locale.US, "%.2f A", amps)
                        + ", alert at " + CURRENT_ALERT_AMPS + " A) — check the mechanism");
            }
            if (motor.isOverCurrent()) {
                faultOverCurrent = true;
            }
        } catch (Exception ignored) {
            lastMotorAmps = Double.NaN;
        }
    }

    /** Re-apply the MOTOR_REVERSED flag (Panels live-tune support) — only writes on change. */
    private void refreshMotorDirection() {
        if (MOTOR_REVERSED != lastReversed) {
            motor.setDirection(MOTOR_REVERSED
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            lastReversed = MOTOR_REVERSED;
        }
    }

    /** Latched fault string — own faults + the tracker's, in the established order. */
    private String faultString() {
        StringBuilder sb = new StringBuilder();
        if (tracker.isEncoderDeadAtBoot()) sb.append("ENCODER_DEAD_AT_BOOT ");
        if (faultEncoderStuck) sb.append("STUCK ");
        if (faultOutOfScope) sb.append("OUT_OF_SCOPE ");
        if (faultOverCurrent) sb.append("OVER_CURRENT ");
        if (faultAbsBootUnstable) sb.append("ABS_BOOT_UNSTABLE ");
        if (tracker.isCrtCrosscheckFaulted()) sb.append("CRT_CROSSCHECK ");
        if (tracker.isEncoderDirectionFaulted()) sb.append("B_DIR_MISMATCH ");
        if (tracker.isMotorEncFaulted()) sb.append("MOTOR_ENC ");
        if (tracker.isMotorEncDirFaulted()) sb.append("MOTOR_ENC_DIR ");
        if (tracker.isAbsStreamLost()) sb.append("ABS_STREAM_LOST ");
        if (sb.length() == 0) return "none";
        return sb.toString();
    }

    /** Tiny local timer (android.os.SystemClock-free, ElapsedTime-style). */
    private static final class ElapsedTimeTimer {
        private final long startNs = System.nanoTime();
        public double seconds() {
            return (System.nanoTime() - startNs) / 1.0e9;
        }
    }
}
