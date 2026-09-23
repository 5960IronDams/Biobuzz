package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AimAtGoal;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.killerwatts.PositionalServo;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

/**
 * Single home for every gamepad binding in TeleOp.
 *
 * <p>Owns, in order:
 * <ol>
 *   <li>Drive: field-centric sticks on gamepad1, or the {@link AimAtGoal} command
 *       while the left trigger is held (aim owns turn, sticks keep translation).</li>
 *   <li>Intake: right trigger press/release schedules
 *       {@code CommandF.RunIntake()} / {@code CommandF.StopIntake()}.</li>
 *   <li>Servo: X toggles presets via {@code CommandF.ServoTogglePos},
 *       bumpers nudge while held.</li>
 *   <li>Localize: start resets the Pedro pose to the alliance start.</li>
 * </ol>
 *
 * <p>Usage (see {@code JowByTeleOp}):
 * <pre>
 * keys = new KeyBindings(this, robot);   // init(), after RobotMain
 * robot.RobotRunPeriodic();              // loop(), first
 * keys.update();                         // loop(), right after (same spot the old
 *                                        // PedroFollowerFieldCentricDrivetrainloop() was)
 * </pre>
 * {@code update()} must run once per loop, AFTER {@code RobotRunPeriodic}, because
 * it applies the follower powers (aim PID output from the scheduler, or fresh stick
 * powers) with a trailing {@code follower.update()} + field draw.
 *
 * <p>Subsystems are hardware-only: they expose Panels-tunable state plus command-safe
 * APIs and never touch a gamepad. Every button/stick lives here. Gamepad2 is currently
 * unbound - add operator bindings in {@link #handleOperator}.
 */
@SuppressWarnings("unused") // public accessors (isAiming, drive sticks) are TeleOp/flywheel API
public class KeyBindings {

    // Alliance start poses for the start-button relocalize.
    private static final Pose RED_RESET_POSE = new Pose(10.5, 10.5, Math.toRadians(0));
    private static final Pose BLUE_RESET_POSE = new Pose(20.5, 20.5, Math.toRadians(180));

    private final RobotMain robot;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    /** Non-null while the left trigger is held and aim owns the follower turn axis. */
    private Command aimCmd = null;

    // Manual rising-edge latches (version-proof; no dependency on SDK *WasPressed APIs).
    private boolean prevServoTogglePressed = false;
    private boolean prevStartPressed = false;
    private long lastNudgeNs = -1L;

    public KeyBindings(OpMode opMode, RobotMain robot) {
        if (opMode == null || robot == null) {
            throw new IllegalArgumentException("KeyBindings needs a non-null OpMode and RobotMain");
        }
        this.robot = robot;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
    }

    /** Poll every binding once. Call once per loop, after {@code robot.RobotRunPeriodic()}. */
    public void update() {
        handleAimScheduling();
        handleIntake();
        handleServo();
        handleRelocalize();
        handleOperator();
        applyDrive();
        reportTelemetry();
    }

    /** Cancel a held aim (call from the OpMode's stop()). Safe when not aiming. */
    public void stop() {
        cancelAim();
    }

    /** True while the aim command owns the follower turn axis. */
    public boolean isAiming() {
        return aimCmd != null;
    }

    // ------------------------------------------------------------------ drive + aim

    /**
     * Left-trigger hold owns turn via {@link AimAtGoal}; translation stays on the sticks.
     * Scheduling happens here (before next loop's Scheduler. execute picks it up); the
     * actual follower powers are applied in {@link #applyDrive} below.
     */
    private void handleAimScheduling() {
        boolean wantAim = AimAtGoal.held(gamepad1.left_trigger);
        if (wantAim && aimCmd == null) {
            aimCmd = robot.CommandF.AimAtClosestGoal(this::driveForward, this::driveStrafe);
            aimCmd.schedule();
        } else if (!wantAim && aimCmd != null) {
            cancelAim();
        }
    }

    /**
     * Moved here from {@code JowByTeleOp.PedroFollowerFieldCentricDrivetrainloop()}.
     * When aiming, the AimAtGoal command already called {@code follower.manual()} during
     * {@code Scheduler.execute()} - just refresh the follower so its powers apply.
     * Otherwise , drive field-centric off the sticks.
     */
    private void applyDrive() {
        if (robot.follower == null || robot.follower.pose() == null) {
            return;
        }
        if (aimCmd == null) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    driveForward(),
                    driveStrafe(),
                    driveTurn(),
                    robot.follower.pose().heading());
            robot.follower.manual(powers);
        }
        // Trailing update: applies aim PID powers OR the fresh manual powers above.
        robot.follower.update();
        // Correct AFTER the trailing update (same predict->correct order as RobotMain).
        if (robot.visionFusion != null) robot.visionFusion.correct();
        robot.fieldRenderer.drawPedroPose(robot.follower.pose());
    }

    private void cancelAim() {
        if (aimCmd != null) {
            aimCmd.cancel();
            aimCmd = null;
        }
    }

    // Alliance-aware stick mapping (preserved from JowByTeleOp).
    private boolean isRed() {
        return RobotMain.CurrentAlliance != ALLIANCE_COLOR.ALLIANCE_BLUE;
    }

    /** Forward stick (+ = forward). Also fed to AimAtGoal so translation survives aiming. */
    public double driveForward() {
        return isRed() ? -gamepad1.left_stick_y : gamepad1.left_stick_y;
    }

    /** Strafe stick (+ = right). Also fed to AimAtGoal so translation survives aiming. */
    public double driveStrafe() {
        return isRed() ? -gamepad1.left_stick_x : gamepad1.left_stick_x;
    }

    /** Turn stick (+ = turn). Ignored while aiming (aim owns turn). */
    public double driveTurn() {
        return -gamepad1.right_stick_x;
    }

    // ------------------------------------------------------------------ subsystems

    /** Right trigger: press -> spin intake, release -> stop (edge-triggered commands). */
    private void handleIntake() {
        if (gamepad1.rightTriggerWasPressed()) {
            robot.CommandF.RunIntake().schedule();
        } else if (gamepad1.rightTriggerWasReleased()) {
            robot.CommandF.StopIntake().schedule();
        }
    }

    /**
     * X toggles the servo preset command; bumpers nudge it while held.
     * Nudge writes the target directly (rate * dt) - the slew + hardware write
     * still happen in {@code PositionalServo.Periodic()}.
     */
    private void handleServo() {
        boolean pressed = gamepad1.x;
        if (pressed && !prevServoTogglePressed) {
            robot.CommandF.ServoTogglePos.schedule();
        }
        prevServoTogglePressed = pressed;

        long nowNs = System.nanoTime();
        double dt = lastNudgeNs < 0 ? 0.02 : (nowNs - lastNudgeNs) / 1.0e9;
        dt = Math.min(Math.max(dt, 0.0), 0.25);
        lastNudgeNs = nowNs;

        if (robot.PosServ != null) {
            if (gamepad1.left_bumper) {
                robot.PosServ.nudge(-PositionalServo.NUDGE_RATE * dt);
            }
            if (gamepad1.right_bumper) {
                robot.PosServ.nudge(PositionalServo.NUDGE_RATE * dt);
            }
        }
    }

    /** Start button: teleport the Pedro pose to the alliance start (edge-triggered). */
    private void handleRelocalize() {
        boolean pressed = gamepad1.start;
        if (pressed && !prevStartPressed && robot.follower != null) {
            robot.follower.setPose(
                    RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE
                            ? BLUE_RESET_POSE : RED_RESET_POSE);
        }
        prevStartPressed = pressed;
    }

    /** Operator (gamepad2) bindings - nothing assigned yet; add flywheel/etc. here. */
    private void handleOperator() {
        // TODO: move operator controls here as mechanism commands land, e.g.
        // if (gamepad2.yWasPressed()) robot.CommandF.RunFlywheel().schedule();
    }

    // ------------------------------------------------------------------ telemetry

    private void reportTelemetry() {
        // addData only — flushed once via robot.flushTelemetry() at end of loop().
        RobotMain.DashTelemetry.addData("Drive mode",
                aimCmd != null ? "AIM (LT held, sticks = translate)" : "manual field-centric");
        if (robot.follower != null && robot.follower.pose() != null) {
            Pose p = robot.follower.pose();
            RobotMain.DashTelemetry.addData("Robot X", p.x());
            RobotMain.DashTelemetry.addData("Robot Y", p.y());
            RobotMain.DashTelemetry.addData("Robot Heading", Math.toDegrees(p.heading()));
        }
    }
}
