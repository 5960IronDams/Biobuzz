package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.Commands.AimAtGoal;
import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.WattageLib.lib.CommandGamepad;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

/**
 * Single home for every gamepad binding in TeleOp.
 *
 * <p>Declarative SolversLib bindings set up once in the constructor:
 * <ol>
 *   <li>Drive: field-centric sticks on gamepad1, or the {@link AimAtGoal} command
 *       while the left trigger is held (aim owns turn, sticks keep translation).</li>
 *   <li>Intake: right trigger press/release schedules
 *       {@code CommandF.RunIntake()} / {@code CommandF.StopIntake()}.</li>
 *   <li>Flywheel: left bumper press/release schedules
 *       {@code CommandF.SetFlywheelRpm(TARGET_RPM)} / {@code CommandF.StopFlywheel()}.</li>
 *   <li>Servo: X toggles presets via {@code CommandF.ServoTogglePos},
 *       dpad nudges while held.</li>
 *   <li>Localize: start resets the Pedro pose to the alliance start.</li>
 * </ol>
 *
 * <p>Usage (see {@code JowByTeleOp}):
 * <pre>
 * keys = new KeyBindings(this, robot);   // init(), after RobotMain
 * robot.RobotRunPeriodic();              // loop(), first (runs CommandScheduler)
 * keys.update();                         // loop(), right after
 * </pre>
 * {@code update()} refreshes the GamepadEx button readers and applies the follower
 * powers, AFTER {@code RobotRunPeriodic} (which runs the scheduler) — the same
 * scheduler-then-inputs ordering the Ivy scheduler had. The field draw happens
 * once per loop in {@code flushTelemetry()}, after this returns.
 */
public class KeyBindings {

    private final RobotMain robot;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    /** Fluent bindings facade over gamepad1 — see bindCommands(). */
    private final CommandGamepad gp1;

    /** Non-null while the aim command is scheduled; it owns the follower turn axis. */
    private Command aimCmd = null;

    // Manual rising-edge latch for the start-button relocalize (kept as a simple
    // poll; no dependency on SDK *WasPressed APIs).
    private boolean prevStartPressed = false;

    public KeyBindings(OpMode opMode, RobotMain robot) {
        if (opMode == null || robot == null) {
            throw new IllegalArgumentException("KeyBindings needs a non-null OpMode and RobotMain");
        }
        this.robot = robot;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        this.gp1 = new CommandGamepad(gamepad1);
        bindCommands();
    }

    /**
     * All gamepad bindings, declared once at init. SolversLib bindings are polled
     * by {@code CommandScheduler.run()} (called from RobotMain.RobotRunPeriodic()).
     */
    private void bindCommands() {
        // ---- Aim: left trigger hold-to-aim ----
        // The aim command is built ONCE here with the live stick suppliers; while
        // it is scheduled, applyDrive() skips manual drive (aim already wrote the
        // powers during scheduler execution). interruptible=true so a new command
        // can always take over if this one lingers.
        aimCmd = robot.CommandF.AimAtClosestGoal(this::driveForward, this::driveStrafe);
        gp1.leftTrigger(AimAtGoal.TRIGGER_THRESHOLD)
                .whileActiveOnce(aimCmd, true);

        // ---- Intake: right trigger press->run, release->stop ----
        // HoldIntakeTunable is a RunCommand: INTAKE_RPM is re-read every loop, so
        // live Panels retunes apply immediately while the trigger is held.
        gp1.rightTrigger(0.1)
                .whenActive(robot.CommandF.HoldIntakeTunable())
                .whenInactive(robot.CommandF.StopIntake());

        // ---- Flywheel: left bumper press->spin up, release->stop ----
        // HoldFlywheelTunable is a RunCommand: TARGET_RPM is re-asserted every
        // loop, so live Panels retunes apply immediately while the bumper is held.
        gp1.leftBumper()
                .whenActive(robot.CommandF.HoldFlywheelTunable())
                .whenInactive(robot.CommandF.StopFlywheel());

        // ---- Servo toggle: X, edge-triggered (persistent instance; reusable) ----
        gp1.x().whenActive(robot.CommandF.ServoTogglePos);

        // ---- Servo nudge: dpad_up/dpad_down, run-while-held ----
        // Each command computes its own dt so nudging rate is loop-time
        // independent; the slew + hardware write still happen in
        // ShootGateServo.periodic(). Requires PosServ so dpad nudges cancel
        // any servo preset command still running.
        gp1.dpadUp().whileActiveOnce(robot.CommandF.NudgeServo(true), true);
        gp1.dpadDown().whileActiveOnce(robot.CommandF.NudgeServo(false), true);
    }

    /** Poll every binding once. Call once per loop, after {@code robot.RobotRunPeriodic()}. */
    public void update() {
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
        return aimCmd != null && aimCmd.isScheduled();
    }

    // ------------------------------------------------------------------ drive + aim

    /**
     * When aiming, the AimAtGoal command already called {@code follower.manual()} during
     * {@code CommandScheduler.run()} - just refresh the follower so its powers apply.
     * Otherwise, drive field-centric off the sticks.
     */
    private void applyDrive() {
        if (robot.follower == null || robot.follower.pose() == null) {
            return;
        }
        if (aimCmd == null || !aimCmd.isScheduled()) {
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
        // Same-frame re-poll no-ops inside VisionFusion; a new LL frame still fuses.
        // No field draw here — flushTelemetry() paints once per loop (final pose).
        if (robot.visionFusion != null) robot.visionFusion.correct();
    }

    private void cancelAim() {
        if (aimCmd != null && aimCmd.isScheduled()) {
            aimCmd.cancel();
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

    /**
     * Start button: teleport the Pedro pose to the alliance start (edge-triggered).
     * Kept as a manual poll — simple edge detection is fine here.
     */
    private void handleRelocalize() {
        boolean pressed = gamepad1.start;
        if (pressed && !prevStartPressed && robot.follower != null) {
            robot.follower.setPose(
                    RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE
                            ? IronConstants.BLUE_RESET_POSE : IronConstants.RED_RESET_POSE);
        }
        prevStartPressed = pressed;
    }

    // ------------------------------------------------------------------ unused

    /** Operator (gamepad2) bindings - nothing assigned yet; add flywheel/etc. here. */
    private void handleOperator() {
        // TODO: move operator controls here as mechanism commands land, e.g.
        // new Trigger(() -> gamepad2.y).whenActive(robot.CommandF.RunFlywheel());
    }

    // ------------------------------------------------------------------ telemetry

    private void reportTelemetry() {
        // addData only — flushed once via robot.flushTelemetry() at end of loop().
        RobotMain.DashTelemetry.addData("Drive mode",
                isAiming() ? "AIM (LT held, sticks = translate)" : "manual field-centric");
        if (robot.follower != null && robot.follower.pose() != null) {
            Pose p = robot.follower.pose();
            RobotMain.DashTelemetry.addData("Robot X", p.x());
            RobotMain.DashTelemetry.addData("Robot Y", p.y());
            RobotMain.DashTelemetry.addData("Robot Heading", Math.toDegrees(p.heading()));
        }
    }
}
