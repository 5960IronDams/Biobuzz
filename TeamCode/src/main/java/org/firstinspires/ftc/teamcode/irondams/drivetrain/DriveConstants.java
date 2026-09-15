package org.firstinspires.ftc.teamcode.irondams.drivetrain;

import com.acmerobotics.dashboard.config.Config;

/**
 * Single Dashboard-tunable home for drivetrain feel + future closed-loop gains.
 *
 * <p>Surface presets: carpet needs more stiction breakaway than comp tile.
 * Flip {@link #USE_COMP_SURFACE} at competition instead of retuning everything.
 *
 * <p>Signal chain (teleop, open-loop default):
 * <pre>
 *   stick -> conditionStick (deadband + rescale + expo, NO min jump)
 *         -> slew ramp (FIELD frame) -> field-centric rotate
 *         -> per-axis gain in ROBOT frame (STRAFE on robot-lateral, FWD on
 *            robot-forward) -> mecanum mix -> normalize
 *         -> per-wheel kS boost + per-wheel scale -> setPower()
 * </pre>
 * Gains MUST live post-rotate: with field-centric drive the stick's "strafe"
 * is field-lateral, but the inefficiency is robot-lateral (roller scrub).
 * At yaw=90deg, pushing the stick forward drives the robot sideways -- so the
 * boost has to follow the chassis, not the stick. No per-axis clamp: the mix
 * normalize preserves the boosted ratio and caps magnitude without distorting
 * diagonal direction.
 *
 * <p>The kS jump lives per-wheel (not per-stick) so strafe/turn mixing that
 * attenuates individual wheels still breaks stiction correctly.
 *
 * <p>Future velocity-PID path (closed loop) reuses the same chain, only the
 * final stage changes to {@code targetTps = shaped * MAX_TPS} with
 * RUN_USING_ENCODER + PIDF. See {@link FourWheelDriveTrain}. Both can be
 * active in code: stick conditioning always applies; power vs velocity is
 * selected by {@link #USE_VELOCITY}.
 */
@Config
public final class DriveConstants {

    private DriveConstants() {}

    // ---- Stick feel (surface-independent) ----
    public static double DEADBAND = 0.08;
    public static double EXPO = 0.35;

    // ---- Per-axis gains: mecanum strafe is ~1.2-1.5x less efficient than forward ----
    // Applied AFTER the field-centric rotate, in ROBOT frame (rotX = robot-lateral
    // scrub axis, rotY = robot-forward). This is the standard "lateralMultiplier"
    // fix. Tune: drive full-forward, time 6ft; drive full-strafe (same heading),
    // raise STRAFE_GAIN until times match. Start ~1.3.
    public static double STRAFE_GAIN = 1.3;
    public static double FWD_GAIN = 1.0;
    public static double TURN_GAIN = 1.0;

    // ---- Stiction breakaway (kS), per surface ----
    // Tune: raise from 0.05 until the robot just creeps.
    public static double MIN_POWER_CARPET = 0.20;
    public static double MIN_POWER_TILE = 0.12;
    /** Flip to true on the smooth competition rubber. */
    public static boolean USE_COMP_SURFACE = false;

    /** Slew rate: power units per second. 3.0 = 0-to-full in ~0.33s. Raised from legacy 3.0. */
    public static double RAMP_RATE = 4.0;

    // ---- Per-wheel trims (added to global kS / multiplied as scale) ----
    // Use these when one wheel drags or one motor is weaker. +/-0.03 is usually plenty.
    public static double KS_TRIM_FL = 0.0;
    public static double KS_TRIM_FR = 0.0;
    public static double KS_TRIM_BL = 0.0;
    public static double KS_TRIM_BR = 0.0;

    public static double SCALE_FL = 1.0;
    public static double SCALE_FR = 1.0;
    public static double SCALE_BL = 1.0;
    public static double SCALE_BR = 1.0;

    // ---- Future velocity-PID mode (CLOSED LOOP, currently OFF) ----
    /** When true, FourWheelDriveTrain outputs setVelocity() instead of setPower(). */
    public static boolean USE_VELOCITY = false;
    public static double TICKS_PER_REV = 537.7; // goBILDA 5203 312rpm
    public static double MAX_RPM = 312.0;
    // Shared RUN_USING_ENCODER gains (same pattern as Intake). Tune these first;
    // per-motor overrides can be added later via FourWheelDriveTrain.applyPerMotorPIDF().
    public static double V_KP = 15.0;
    public static double V_KI = 0.0;
    public static double V_KD = 0.0;
    public static double V_KF = 12.0;

    /** Effective global kS for the current surface. */
    public static double getMinPower() {
        return USE_COMP_SURFACE ? MIN_POWER_TILE : MIN_POWER_CARPET;
    }

    public static double getMaxTicksPerSec() {
        return MAX_RPM * TICKS_PER_REV / 60.0;
    }

    /**
     * Deadband + rescale to 0..1 + expo shaping. No minimum jump here on purpose:
     * the kS jump is applied per-wheel in FourWheelDriveTrain so mixing math
     * stays proportional.
     */
    public static double conditionStick(double in) {
        double a = Math.abs(in);
        if (a < DEADBAND) return 0.0;
        double lin = (a - DEADBAND) / (1.0 - DEADBAND); // 0..1
        double shaped = (1.0 - EXPO) * lin + EXPO * lin * lin * lin;
        return Math.signum(in) * shaped;
    }
}
