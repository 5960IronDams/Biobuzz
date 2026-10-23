package org.firstinspires.ftc.teamcode.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Subsystems.turret.TurretPivot;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PPAssets;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PPFile;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.WattageLib.Ballistics.NoFireZones;

import java.util.function.Supplier;

/**
 * Hold-to-aim command: SPINS THE TURRET to face the closest shooting target
 * for the current alliance while the driver keeps full chassis control
 * (translation AND rotation stay on the sticks — the robot no longer aims
 * itself, the turret does).
 *
 * <p>Targets come from {@code assets/pathfiles/AimPoints.pp}, parsed with
 * {@link PPFile}. Points are filtered by alliance name prefix
 * ({@code "Red*"} / {@code "Blue*"}), and the closest one (by XY distance to
 * the follower pose) is picked fresh every loop, so the lock follows you
 * around the field.
 *
 * <p>SESSION 5 (turret-aim port, per the FRC IndianaJoe pattern): the aim
 * member no longer touches the follower. Each loop it computes the target's
 * robot-relative bearing (field bearing minus chassis heading), hands it to
 * {@link TurretPivot#GotoSafeAngle(double, double)} together with a
 * chassis-yaw-rate feedforward so the turret tracks the field aim while the
 * chassis turns, and lets the turret's own soft limits + PID do the motion.
 * On release the turret HOLDS its last angle (no park) — the driver's right
 * stick takes over manually the moment the trigger drops.
 *
 * <p>Requirements: the turret subsystem ONLY. The follower is read (pose) but
 * never required, so driving is never interrupted.
 */
@Configurable
public class AimAtGoal extends CommandBase {

    // ---- Panels-tunable aim constants (Panels -> AimAtGoal) ----
    /** Trigger value above which "aim is held" (TeleOp wiring helper). */
    public static double TRIGGER_THRESHOLD = 0.3;
    /**
     * Chassis-yaw-rate feedforward to the turret. SESSION 5g UNITS FIX: the
     * lead added to the setpoint is ONE LOOP of predicted chassis rotation,
     * in DEGREES: {@code ffDeg = -yawRateDps * loopDtSec * YAW_FF_SCALE},
     * so YAW_FF_SCALE is now "loops of lead" (1.0 = predict the heading one
     * loop ahead). The old code added {@code -yawRateDps * 1.0} — a VELOCITY
     * injected as a POSITION, 1/dt ≈ 25–50x too strong: every 0.5–0.9 deg of
     * per-loop Pinpoint heading noise/rock became a ±25–45 deg setpoint
     * jump, the turret slammed chasing it, and its own reaction shake
     * re-excited the heading noise. Proven in capture 20261023-025055
     * i=470–525: bearing + yaw was constant at −2.8 deg (true aim 2.8 deg
     * away) while the setpoint thrashed ±30–60 deg — every degree of it was
     * this term. 0 disables.
     */
    public static double YAW_FF_SCALE = 1.0;
    /**
     * EMA smoothing (new-sample weight) on the chassis yaw-rate estimate.
     * Pinpoint heading jitters ~±0.5–0.9 deg per loop under turret-slam
     * vibration, so the raw dHeading/dt swings ±25–45 deg/s loop-to-loop;
     * 0.3 keeps real chassis spins responsive while averaging the
     * alternating noise down. 1.0 = no smoothing.
     */
    public static double YAW_RATE_EMA_ALPHA = 0.3;
    /**
     * |chassis yaw rate| below this (deg/s) is treated as zero for the
     * feedforward — damps Pedro pose micro-jitter into the turret setpoint.
     * Raised 2 → 5 in session 5g: with the units fix the residual lead is
     * tiny either way, and a stationary-but-shaking robot should produce
     * ZERO ff.
     */
    public static double YAW_FF_DEADBAND_DPS = 5.0;
    /**
     * Aim-point hysteresis: keep the locked target unless another alliance
     * point is closer than the current one by more than this margin (in).
     * Selection was PROVEN stable while stationary (capture 20261023-025055
     * logged RedFront on every aim loop) — this is hardening for driving
     * between hives so range noise at a near-tie can't flip the aim.
     */
    public static double AIM_SWITCH_MARGIN_IN = 6.0;
    /** |error| below this (degrees) counts as "on target" for telemetry. */
    public static double TOLERANCE_DEG = 2.0;

    /** Asset path of the aim-point file. Already contains BOTH alliances' points. */
    public static final String ASSET_PATH = "pathfiles/AimPoints.pp";

    private final TurretPivot turret;
    private final Follower follower;
    private final PPFile aimPoints;
    private final Supplier<ALLIANCE_COLOR> alliance;

    // Per-run state (also surfaced for telemetry / flywheel compositions).
    private int targetIndex = -1;
    private Pose targetPose = null;
    private String targetName = "";
    private boolean usingFallback = false;
    private double headingErrorDeg = 0.0;
    private double turretSetpointDeg = 0.0;
    private double yawRateDps = 0.0;
    private double prevHeadingRad = Double.NaN;
    /** SESSION 5g: last execute() timestamp for the real loop period (ns). */
    private long lastExecNs = 0L;
    /** SESSION 5g: measured command loop period (s) — no hard-coded 0.02. */
    private double loopDtSec = 0.02;

    /** Load aim points from assets and aim the TURRET with the driver in full control. */
    public AimAtGoal(TurretPivot turret, Follower follower, HardwareMap hardwareMap,
                     Supplier<ALLIANCE_COLOR> alliance) {
        this(turret, follower, loadAimPoints(hardwareMap), alliance);
    }

    /** Inject an already-loaded PPFile (tests, or a shared copy). */
    public AimAtGoal(TurretPivot turret, Follower follower, PPFile aimPoints,
                     Supplier<ALLIANCE_COLOR> alliance) {
        if (turret == null) {
            throw new IllegalArgumentException("AimAtGoal needs a non-null TurretPivot");
        }
        if (aimPoints == null) {
            throw new IllegalArgumentException("AimAtGoal: " + ASSET_PATH + " has no points");
        }
        // Own the turret while running — manual right-stick spin must yield.
        addRequirements(turret);
        this.turret = turret;
        this.follower = follower; // read-only: pose source for target bearing
        this.aimPoints = aimPoints;
        this.alliance = alliance != null ? alliance : () -> ALLIANCE_COLOR.ALLIANCE_RED;
    }

    /**
     * Load the aim file with a plain degrees factory (NO Blue mirroring).
     * The file already stores both alliances' points in absolute field
     * coordinates, so mirroring would fling the Blue points to the wrong side.
     * Alliance is handled by filtering point names ({@code "Red*"}/{@code "Blue*"}) instead.
     */
    public static PPFile loadAimPoints(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException(
                    "AimAtGoal needs a non-null HardwareMap to load " + ASSET_PATH);
        }
        try {
            return PPAssets.fromAsset(hardwareMap, ASSET_PATH, PoseFactory.degrees());
        } catch (Exception e) {
            throw new RuntimeException("AimAtGoal: failed to load asset " + ASSET_PATH, e);
        }
    }

    /** Name prefix used to filter points per alliance (matches AimPoints.pp). */
    public static String prefixFor(ALLIANCE_COLOR alliance) {
        return alliance == ALLIANCE_COLOR.ALLIANCE_BLUE ? "Blue" : "Red";
    }

    /** TeleOp helper: is the trigger held past {@link #TRIGGER_THRESHOLD}? */
    public static boolean held(double triggerValue) {
        return triggerValue > TRIGGER_THRESHOLD;
    }

    // ---- Command lifecycle (runs until cancelled : hold trigger to aim) ----

    @Override
    public void initialize() {
        prevHeadingRad = Double.NaN;
        lastExecNs = 0L;
        pickClosestTarget();
    }

    @Override
    public void execute() {
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null) {
            return;
        }
        pickClosestTarget();
        if (targetPose == null) {
            return;
        }

        // Shots originate from the SHOOTER pivot, not the robot center — rotate
        // the robot pose through its heading to apply the pivot offset, then
        // aim from that point.
        Pose shooter = getShooterPose(robot);

        // Field bearing to the target from the SHOOTER, converted to a TURRET
        // angle: desired robot-relative bearing (deg, +CCW, 0 = chassis forward)
        double desiredBearingDeg =
                Math.toDegrees(Angle.normalizeSigned(
                        Math.atan2(targetPose.y() - shooter.y(), targetPose.x() - shooter.x())
                                - robot.heading()));
        headingErrorDeg = normDeg180(desiredBearingDeg - turret.getPosition());

        // SESSION 5g: measure the REAL command loop period. The old
        // hard-coded 0.02 s fed both the yaw-rate estimate and (through the
        // ff units bug) the setpoint noise amplitude.
        long nowNs = System.nanoTime();
        if (lastExecNs != 0L) {
            double dt = (nowNs - lastExecNs) * 1e-9;
            if (dt > 0.002 && dt < 0.2) {
                loopDtSec = dt;
            }
        }
        lastExecNs = nowNs;

        // Chassis yaw-rate feedforward (deg/s, +CCW) — EMA-smoothed, then
        // sign-flipped into a ONE-LOOP-OF-LEAD angle (see YAW_FF_SCALE) so
        // the turret holds the field aim while the chassis turns (same idea
        // as FRC GotoSafeAngle(angle, velocityDps)).
        double headingRad = robot.heading();
        if (Double.isNaN(prevHeadingRad)) {
            prevHeadingRad = headingRad;
            yawRateDps = 0.0;
        } else {
            double dHeading = Angle.normalizeSigned(headingRad - prevHeadingRad);
            prevHeadingRad = headingRad;
            double rawYawDps = Math.toDegrees(dHeading / loopDtSec);
            yawRateDps = YAW_RATE_EMA_ALPHA * rawYawDps
                    + (1.0 - YAW_RATE_EMA_ALPHA) * yawRateDps;
        }
        double ffDeg = Math.abs(yawRateDps) < YAW_FF_DEADBAND_DPS
                ? 0.0
                : -yawRateDps * loopDtSec * YAW_FF_SCALE;

        turretSetpointDeg = desiredBearingDeg + ffDeg;
        // GotoSafeAngle clamps to soft limits + picks the nearest ±360 twin,
        // exactly like FRC's chooseTurretSetpointDeg. The velocity argument
        // is a true deg/s feedforward rate (used by the turret's kV path
        // when position-mode kV is enabled).
        turret.GotoSafeAngle(turretSetpointDeg, -yawRateDps);

        RobotMain.DashTelemetry.addData("Aim target", "%s (%.1f, %.1f)%s",
                targetName, targetPose.x(), targetPose.y(),
                usingFallback ? " [fallback: no alliance match]" : "");
        RobotMain.DashTelemetry.addData("Aim err (deg)", "%.2f", headingErrorDeg);
        RobotMain.DashTelemetry.addData("Aim turret set", "%.1f deg", turretSetpointDeg);
        RobotMain.DashTelemetry.addData("Aim chassis yaw", "%+.1f dps", yawRateDps);
        RobotMain.DashTelemetry.addData("Aim dt (ms)", "%.1f", loopDtSec * 1000.0);
        RobotMain.DashTelemetry.addData("Aim dist (in)", "%.1f", getDistanceInches());
        RobotMain.DashTelemetry.addData("Aim on target", isOnTarget());
    }

    @Override
    public boolean isFinished() {
        return false; // hold-to-aim: cancel on trigger release or compose with .until(...)
    }

    @Override
    public void end(boolean interrupted) {
        // Turret HOLDS its last commanded angle (GotoSafeAngle's setpoint stays
        // latched in the subsystem) — no park; the driver's right stick takes
        // over manually the moment the trigger drops. Chassis is untouched
        // (never required) so driving continues without a blip.
    }

    // ---- Target selection ----

    /**
     * Re-pick the closest alliance point to the current pose.
     */
    private void pickClosestTarget() {
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null) {
            return;
        }
        // Closest target as measured from the SHOOTER (pivot offset applied),
        // not the robot center.
        Pose shooter = getShooterPose(robot);
        String prefix = prefixFor(alliance.get());

        boolean fallbackNow = false;
        int best = -1;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (int i = 0; i < aimPoints.pointCount(); i++) {
            String name = aimPoints.pointName(i);
            if (name == null || !name.regionMatches(true, 0, prefix, 0, prefix.length())) {
                continue;
            }
            double d2 = dist2(shooter, aimPoints.point(i));
            if (d2 < bestD2) {
                bestD2 = d2;
                best = i;
            }
        }
        if (best < 0) {
            // No alliance-prefixed points (file renamed?) - fall back to all
            // target points (skipping index 0, the "Robot" start point).
            for (int i = 1; i < aimPoints.pointCount(); i++) {
                double d2 = dist2(shooter, aimPoints.point(i));
                if (d2 < bestD2) {
                    bestD2 = d2;
                    best = i;
                }
            }
            fallbackNow = true;
        }
        if (best < 0) {
            return; // no points at all; execute() guards on targetPose == null
        }
        // SESSION 5g hysteresis: keep the locked target unless the challenger
        // is meaningfully closer (AIM_SWITCH_MARGIN_IN). Only applies within
        // the same candidate set (alliance vs fallback) so a set change still
        // switches immediately. Stationary selection was already proven
        // stable — this guards driving between hives against near-tie flips.
        if (best != targetIndex && targetIndex >= 0
                && targetIndex < aimPoints.pointCount()
                && fallbackNow == usingFallback) {
            double curDist = Math.sqrt(dist2(shooter, aimPoints.point(targetIndex)));
            double bestDist = Math.sqrt(bestD2);
            if (bestDist >= curDist - AIM_SWITCH_MARGIN_IN) {
                best = targetIndex;
            }
        }
        usingFallback = fallbackNow;
        if (best != targetIndex || targetPose == null) {
            targetIndex = best;
            targetPose = aimPoints.point(best);
            targetName = aimPoints.pointName(best);
        }
    }

    private static double dist2(Pose a, Pose b) {
        double dx = a.x() - b.x();
        double dy = a.y() - b.y();
        return dx * dx + dy * dy;
    }

    /**
     * Compute the SHOOTER's actual field-frame position: the robot's pose
     * rotated through its own heading to apply the physical pivot offset
     * ({@link NoFireZones#SHOOTER_OFFSET_X_IN}/{@code _Y_IN}). Shots originate
     * from here — the bearing and distance to targets must both be measured
     * from this point, not the robot center.
     */
    private static Pose getShooterPose(Pose robot) {
        double heading = robot.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double shooterX = robot.x() + NoFireZones.SHOOTER_OFFSET_X_IN * cos - NoFireZones.SHOOTER_OFFSET_Y_IN * sin;
        double shooterY = robot.y() + NoFireZones.SHOOTER_OFFSET_X_IN * sin + NoFireZones.SHOOTER_OFFSET_Y_IN * cos;
        return new Pose(shooterX, shooterY, heading);
    }

    // ---- Telemetry / composition accessors ----

    /** Name of the currently locked point (e.g. {@code "RedBack"}). */
    public String getTargetName() {
        return targetName;
    }

    /** Pose of the currently locked point. */
    public Pose getTargetPose() {
        return targetPose;
    }

    /** Signed turret aim error in degrees (+ = need CCW turret motion). */
    public double getHeadingErrorDeg() {
        return headingErrorDeg;
    }

    /** Last turret setpoint (deg, robot-relative, before soft-limit clamping). */
    public double getTurretSetpointDeg() {
        return turretSetpointDeg;
    }

    /** Last measured chassis yaw rate (deg/s, +CCW) used for the feedforward. */
    public double getYawRateDps() {
        return yawRateDps;
    }

    /** True when |error| is within {@link #TOLERANCE_DEG}. */
    public boolean isOnTarget() {
        return Math.abs(headingErrorDeg) <= TOLERANCE_DEG;
    }

    /**
     * Distance (inches) from the SHOOTER (robot pose + pivot offset) to the
     * locked point — the correct input to the flywheel RPM LUT; NaN if unknown.
     */
    public double getDistanceInches() {
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null || targetPose == null) {
            return Double.NaN;
        }
        Pose shooter = getShooterPose(robot);
        return Math.hypot(targetPose.x() - shooter.x(), targetPose.y() - shooter.y());
    }

    /** Number of points in the loaded aim file (expect 5: Robot + 4 targets). */
    public int getPointCount() {
        return aimPoints.pointCount();
    }

    /** Normalize degrees to [-180, 180). */
    private static double normDeg180(double deg) {
        double r = (deg + 180.0) % 360.0;
        if (r < 0) r += 360.0;
        return r - 180.0;
    }
}
