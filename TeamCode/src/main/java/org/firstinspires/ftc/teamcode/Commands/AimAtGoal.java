package org.firstinspires.ftc.teamcode.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PPAssets;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PPFile;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Hold-to-aim command: rotates the robot to face the closest shooting target
 * for the current alliance while leaving translation (XY) on the driver sticks.
 *
 * <p>Targets come from {@code assets/pathfiles/AimPoints.pp}, parsed with
 * {@link PPFile}. Points are filtered by alliance name prefix
 * ({@code "Red*"} vs {@code "Blue*"}), and the closest one (by XY distance to
 * the follower pose) is picked fresh every loop, so the lock follows you
 * around the field.
 *
 * <p>Only the turn axis is PID-controlled (tunable below via Panels
 * -&gt; AimAtGoal). Forward/strafe come from the supplied sticks, so the
 * driver can still translate while aiming.
 *
 * <p>This is a normal SolversLib {@link Command}
 * (a {@link CommandBase}): it runs until cancelled, so hold the left trigger
 * to aim and compose it with the flywheel, e.g.
 * <pre>
 * // TeleOp hold-to-aim wiring (see JowByTeleOp):
 * if (AimAtGoal.held(gamepad1.left_trigger) && aimCmd == null) {
 *     aimCmd = robot.CommandF.AimAtClosestGoal(fwdStick, strafeStick);
 *     aimCmd.schedule();
 * } else if (!AimAtGoal.held(gamepad1.left_trigger) && aimCmd != null) {
 *     aimCmd.cancel();
 *     aimCmd = null;
 * }
 * // ...and skip the default manual() drive while aimCmd != null.
 *
 * // Composition with the flywheel (deadline: aiming never finishes on its own):
 * schedule(deadline(
 *     robot.CommandF.AimAtClosestGoal(fwd, strafe)
 *         .until(() -&gt; gamepad1.left_trigger &lt; 0.3),
 *     robot.intake.runCmd()));
 * </pre>
 */
@Configurable
public class AimAtGoal extends CommandBase {

    // ---- Panels-tunable heading PID (Panels -> AimAtGoal) ----
    /** Proportional gain on heading error (radians) -> turn power. */
    public static double kP = 2.5;
    /** Integral gain (anti-windup via {@link #I_MAX}). */
    public static double kI = 0.0;
    /** Derivative gain on heading-error rate. */
    public static double kD = 0.1;
    /** Max |turn| power the PID may request. */
    public static double MAX_TURN = 1.0;//0.7;
    /** Integral clamp (anti-windup). */
    public static double I_MAX = 1.0;
    /** |error| below this (degrees) counts as "on target" for telemetry. */
    public static double TOLERANCE_DEG = 2.0;
    /** Trigger value above this means "aim held" (TeleOp wiring helper). */
    public static double TRIGGER_THRESHOLD = 0.3;

    /** Asset path of the aim-point file. Already contains BOTH alliances' points. */
    public static final String ASSET_PATH = "pathfiles/AimPoints.pp";

    private final Follower follower;
    private final PPFile aimPoints;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final Supplier<ALLIANCE_COLOR> alliance;

    // Per-run state (also surfaced for telemetry / flywheel compositions).
    private int targetIndex = -1;
    private Pose targetPose = null;
    private String targetName = "";
    private boolean usingFallback = false;
    private double headingErrorRad = 0.0;
    private double turnPower = 0.0;

    // PID state.
    private double integral = 0.0;
    private double prevError = 0.0;
    private long lastTimeNs = 0L;
    private boolean firstRun = true;

    /** Load aim points from assets and aim with driver-supplied translation. */
    public AimAtGoal(Follower follower, HardwareMap hardwareMap,
                     DoubleSupplier forward, DoubleSupplier strafe,
                     Supplier<ALLIANCE_COLOR> alliance) {
        this(follower, loadAimPoints(hardwareMap), forward, strafe, alliance, null);
    }

    /** Same, but with a SHARED follower requirement (see the full ctor). */
    public AimAtGoal(Follower follower, HardwareMap hardwareMap,
                     DoubleSupplier forward, DoubleSupplier strafe,
                     Supplier<ALLIANCE_COLOR> alliance, Subsystem sharedFollowerReqt) {
        this(follower, loadAimPoints(hardwareMap), forward, strafe, alliance,
                sharedFollowerReqt);
    }

    /** Inject an already-loaded PPFile (tests, or a shared copy). */
    public AimAtGoal(Follower follower, PPFile aimPoints,
                     DoubleSupplier forward, DoubleSupplier strafe,
                     Supplier<ALLIANCE_COLOR> alliance) {
        this(follower, aimPoints, forward, strafe, alliance, null);
    }

    /**
     * Full ctor. Pass a SHARED follower requirement {@link Subsystem} (e.g. the
     * one held by {@code CommandFactory}) so multiple AimAtGoal instances - a
     * bare aim command and one inside a compound like ShootAtGoal - map to the
     * SAME scheduler requirement and properly conflict/interrupt each other.
     * With distinct per-instance wrappers the scheduler treats them as
     * unrelated and two aims could fight over the follower. Null = private wrapper.
     */
    public AimAtGoal(Follower follower, PPFile aimPoints,
                     DoubleSupplier forward, DoubleSupplier strafe,
                     Supplier<ALLIANCE_COLOR> alliance, Subsystem sharedFollowerReqt) {
        // Own the follower while running (SolversLib requirement).
        this.followerSubsystem =
                sharedFollowerReqt != null ? sharedFollowerReqt : new SubsystemBase() {};
        addRequirements(followerSubsystem);
        if (follower == null) {
            throw new IllegalArgumentException("AimAtGoal needs a non-null Follower");
        }
        if (aimPoints == null) {
            throw new IllegalArgumentException("AimAtGoal needs a non-null PPFile");
        }
        if (aimPoints.pointCount() == 0) {
            throw new IllegalArgumentException("AimAtGoal: " + ASSET_PATH + " has no points");
        }
        this.follower = follower;
        this.aimPoints = aimPoints;
        this.forward = forward != null ? forward : () -> 0.0;
        this.strafe = strafe != null ? strafe : () -> 0.0;
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

    /** SolversLib requires Subsystem requirements; the Follower is wrapped here. */
    private final Subsystem followerSubsystem;

    // ---- Command lifecycle (runs until cancelled : hold trigger to aim) ----

    @Override
    public void initialize() {
        integral = 0.0;
        prevError = 0.0;
        firstRun = true;
        lastTimeNs = System.nanoTime();
        pickClosestTarget();
    }

    @Override
    public void execute() {
        Pose robot = follower.pose();
        if (robot == null) {
            return;
        }
        pickClosestTarget();
        if (targetPose == null) {
            return;
        }

        double desired = Math.atan2(targetPose.y() - robot.y(), targetPose.x() - robot.x());
        headingErrorRad = Angle.normalizeSigned(desired - robot.heading());

        long nowNs = System.nanoTime();
        double dt = firstRun ? 0.02 : (nowNs - lastTimeNs) / 1.0e9;
        dt = Math.min(Math.max(dt, 0.001), 0.25);
        lastTimeNs = nowNs;

        integral += headingErrorRad * dt;
        integral = Math.min(Math.max(integral, -I_MAX), I_MAX);
        double derivative = firstRun ? 0.0 : (headingErrorRad - prevError) / dt;
        firstRun = false;
        prevError = headingErrorRad;

        turnPower = kP * headingErrorRad + kI * integral + kD * derivative;
        turnPower = Math.min(Math.max(turnPower, -MAX_TURN), MAX_TURN);

        DrivePowers powers = ManualDrive.fieldCentric(
                forward.getAsDouble(), strafe.getAsDouble(), turnPower, robot.heading());
        follower.manual(powers);

        RobotMain.DashTelemetry.addData("Aim target", "%s (%.1f, %.1f)%s",
                targetName, targetPose.x(), targetPose.y(),
                usingFallback ? " [fallback: no alliance match]" : "");
        RobotMain.DashTelemetry.addData("Aim err (deg)", "%.2f", Math.toDegrees(headingErrorRad));
        RobotMain.DashTelemetry.addData("Aim turn", "%.3f", turnPower);
        RobotMain.DashTelemetry.addData("Aim dist (in)", "%.1f", getDistanceInches());
        RobotMain.DashTelemetry.addData("Aim on target", isOnTarget());
    }

    @Override
    public boolean isFinished() {
        return false; // hold-to-aim: cancel on trigger release or compose with .until(...)
    }

    @Override
    public void end(boolean interrupted) {
        integral = 0.0;
        firstRun = true;
        // Hand translation straight back to the driver with zero turn so the
        // robot doesn't keep spinning for a loop after release.
        Pose robot = follower.pose();
        double heading = robot != null ? robot.heading() : 0.0;
        follower.manual(ManualDrive.fieldCentric(
                forward.getAsDouble(), strafe.getAsDouble(), 0.0, heading));
    }

    // ---- Requirements are declared via addRequirements(followerSubsystem) above ----

    // ---- Target selection ----

    /**
     * Re-pick the closest alliance point to the current pose. Resets the
     * integral on a target switch so a fresh lock doesn't inherit windup.
     */
    private void pickClosestTarget() {
        Pose robot = follower.pose();
        if (robot == null) {
            return;
        }
        String prefix = prefixFor(alliance.get());

        int best = -1;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (int i = 0; i < aimPoints.pointCount(); i++) {
            String name = aimPoints.pointName(i);
            if (name == null || !name.regionMatches(true, 0, prefix, 0, prefix.length())) {
                continue;
            }
            double d2 = dist2(robot, aimPoints.point(i));
            if (d2 < bestD2) {
                bestD2 = d2;
                best = i;
            }
        }
        usingFallback = false;
        if (best < 0) {
            // No alliance-prefixed points (file renamed?) - fall back to all
            // target points (skipping index 0, the "Robot" start point).
            for (int i = 1; i < aimPoints.pointCount(); i++) {
                double d2 = dist2(robot, aimPoints.point(i));
                if (d2 < bestD2) {
                    bestD2 = d2;
                    best = i;
                }
            }
            usingFallback = true;
        }
        if (best < 0) {
            return; // no points at all; execute() guards on targetPose == null
        }
        if (best != targetIndex) {
            integral = 0.0;
            targetIndex = best;
            targetPose = aimPoints.point(best);
            targetName = aimPoints.pointName(best);
        } else if (targetPose == null) {
            targetPose = aimPoints.point(best);
            targetName = aimPoints.pointName(best);
        }
    }

    private static double dist2(Pose a, Pose b) {
        double dx = a.x() - b.x();
        double dy = a.y() - b.y();
        return dx * dx + dy * dy;
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

    /** Signed heading error in degrees (+ = need CCW turn). */
    public double getHeadingErrorDeg() {
        return Math.toDegrees(headingErrorRad);
    }

    /** Last turn power sent to the follower. */
    public double getTurnPower() {
        return turnPower;
    }

    /** True when |error| is within {@link #TOLERANCE_DEG}. */
    public boolean isOnTarget() {
        return Math.abs(headingErrorRad) <= Math.toRadians(TOLERANCE_DEG);
    }

    /** Distance (inches) from the robot to the locked point; NaN if unknown. */
    public double getDistanceInches() {
        Pose robot = follower.pose();
        if (robot == null || targetPose == null) {
            return Double.NaN;
        }
        return Math.hypot(targetPose.x() - robot.x(), targetPose.y() - robot.y());
    }

    /** Number of points in the loaded aim file (expect 5: Robot + 4 targets). */
    public int getPointCount() {
        return aimPoints.pointCount();
    }
}
