package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Commands.AimAtGoal;
import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.WattageLib.lib.CommandGamepad;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PanelsGamepadDebug;

/**
 * TeleOp gamepad bindings for Driver (gamepad1) and Operator (gamepad2).
 *
 * <p>Driver (gamepad1):
 * <ul>
 *   <li>Sticks: field-centric drive (left = translate, right X = turn).</li>
 *   <li>Left Trigger (toggle): auto-aim turret + LUT flywheel + auto-fire when ready.</li>
 *   <li>Left Bumper: stop shooting / panic reset.</li>
 *   <li>Dpad Left/Right: manual turret jog (when auto-aim is off).</li>
 *   <li>Right Trigger (hold): intake forward; A (hold): intake reverse.</li>
 *   <li>Right Bumper (hold): transfer forward; B (hold): transfer reverse.</li>
 *   <li>Y: toggle gate servo; X + Dpad Left/Right: nudge gate servo.</li>
 *   <li>Dpad Down: lower ramp.</li>
 *   <li>Back: reset robot pose to alliance start.</li>
 * </ul>
 *
 * <p>Operator (gamepad2):
 * <ul>
 *   <li>Right Trigger (toggle): manual shoot at fixed RPM (no turret movement).</li>
 *   <li>Dpad Down: lower ramp; LB/RB: jog ramp; Y: zero ramp; Back: cycle jog servo.</li>
 *   <li>B/X: flywheel kF +/-; Dpad Right/Left: flywheel kP +/- (hold LT for fine step); Dpad Up: test spin.</li>
 * </ul>
 */
public class KeyBindings {

    private final RobotMain robot;
    private final CommandGamepad DriveController;
    private final CommandGamepad OperatorController;
    private final PanelsGamepadDebug gamepadDebug;

    /** Degrees per loop to jog the turret while holding Dpad Left/Right. */
    public static final double MANUAL_JOG_STEP_DEG = 4.0;

    /** Right-trigger threshold for running the intake. */
    public static final double INTAKE_TRIGGER_THRESHOLD = 0.1;

    private boolean prevBackPressed = false;
    private boolean prevOpB = false;
    private boolean prevOpX = false;
    private boolean prevOpDpadRight = false;
    private boolean prevOpDpadLeft = false;
    private boolean prevOpDpadUp = false;

    private final Command autoShoot;
    private final Command manualShoot;

    public KeyBindings(OpMode opMode, RobotMain robot) {
        if (opMode == null || robot == null) {
            throw new IllegalArgumentException("KeyBindings needs a non-null OpMode and RobotMain");
        }
        this.robot = robot;
        this.DriveController = new CommandGamepad(opMode.gamepad1);
        this.OperatorController = new CommandGamepad(opMode.gamepad2);
        this.gamepadDebug = new PanelsGamepadDebug(opMode.gamepad1, "GP1");
        this.autoShoot = robot.CommandF.getAutoShootCommand();
        this.manualShoot = robot.CommandF.getManualShootCommand();
        bindCommands();
    }

    private void bindCommands() {
        // Manual turret jog (only active when auto-aim is off)
        if (robot.turret != null) {
            new Trigger(() -> DriveController.dpadLeft().get() && !isAutoAimOn())
                    .whileActiveOnce(
                            new RunCommand(() -> robot.turret.NudgeDeg(MANUAL_JOG_STEP_DEG),
                                    robot.turret),
                            true);
            new Trigger(() -> DriveController.dpadRight().get() && !isAutoAimOn())
                    .whileActiveOnce(
                            new RunCommand(() -> robot.turret.NudgeDeg(-MANUAL_JOG_STEP_DEG),
                                    robot.turret),
                            true);
        }

        // Auto-shoot toggle (Driver LT)
        if (robot.turret != null && autoShoot != null) {
            DriveController.leftTrigger(AimAtGoal.TRIGGER_THRESHOLD)
                    .toggleWhenActive(autoShoot, true);
        }

        // Manual fixed-RPM shoot toggle (Operator RT)
        if (manualShoot != null) {
            OperatorController.rightTrigger(AimAtGoal.TRIGGER_THRESHOLD)
                    .toggleWhenActive(manualShoot, true);
        }

        // Stop shoot / panic reset (Driver LB)
        DriveController.leftBumper().whenActive(robot.CommandF.getStopShootCommand());

        // Intake (Driver RT = forward, A = reverse)
        DriveController.rightTrigger(INTAKE_TRIGGER_THRESHOLD)
                .whenActive(robot.intake.holdCmd())
                .whenInactive(robot.intake.stopCmd());
        DriveController.a()
                .whenActive(robot.intake.reverseCmd())
                .whenInactive(robot.intake.stopCmd());

        // Transfer / reservoir (Driver RB = forward, B = reverse; integrates with active shoot)
        new Trigger(() -> DriveController.rightBumper().get() && !isShooting())
                .whenActive(robot.transfer.holdCmd())
                .whenInactive(robot.transfer.stopCmd());
        new Trigger(() -> DriveController.b().get() && !isShooting())
                .whenActive(robot.transfer.reverseHoldCmd())
                .whenInactive(robot.transfer.stopCmd());

        // Gate servo (Driver Y = toggle, X + Dpad Left/Right = nudge)
        DriveController.y().whenActive(robot.PosServ.toggleCmd());
        DriveController.dpadLeft().and(DriveController.x()).whileActiveOnce(robot.PosServ.nudgeCmd(true), true);
        DriveController.dpadRight().and(DriveController.x()).whileActiveOnce(robot.PosServ.nudgeCmd(false), true);

        bindRamp();
    }

    private void bindRamp() {
        if (robot.ramp == null) {
            return; // ramp servos not in the RC config — every binding skipped
        }
        // Taps: dpad DOWN -> lowered, dpad UP -> parked at 0.
        DriveController.dpadDown().whenActive(robot.ramp.toggleDownCmd());
        //DriveController.dpadUp().whenActive(robot.ramp.toggleUpCmd());

        // Taps: dpad DOWN -> lowered, dpad UP -> parked at 0.
        OperatorController.dpadDown().whenActive(robot.ramp.toggleDownCmd());
        //OperatorController.dpadUp().whenActive(robot.ramp.toggleUpCmd());
        // Held fine jog for calibrating POS_DOWN (and un-sticking a ramp).
        OperatorController.leftBumper().whileActiveOnce(robot.ramp.jogCmd(true), true);
        OperatorController.rightBumper().whileActiveOnce(robot.ramp.jogCmd(false), true);
        // Bring-up zero: install-the-horns position / rescue.
        OperatorController.y().whenActive(robot.ramp.zeroCmd());
        // Cycle which servo the jog moves: LEFT -> RIGHT -> BOTH.
        OperatorController.back().whenActive(robot.ramp.cycleJogSelectCmd());
    }

    /** Updates manual driver intent, relocalize, operator PIDF nudges, and drive powers each loop. */
    public void update() {
        robot.CommandF.setManualIntakeHeld(robot.intake != null
                && DriveController.rightTrigger(INTAKE_TRIGGER_THRESHOLD).get());
        robot.CommandF.setManualTransferForwardHeld(robot.transfer != null
                && DriveController.rightBumper().get());
        robot.CommandF.setManualTransferReverseHeld(robot.transfer != null
                && DriveController.b().get());
        handleRelocalize();
        handleOperator();
        applyDrive();
    }

    /** Cancels any active shoot mode (call in OpMode.stop()). */
    public void stop() {
        if (autoShoot != null && autoShoot.isScheduled()) {
            autoShoot.cancel();
        }
        if (manualShoot != null && manualShoot.isScheduled()) {
            manualShoot.cancel();
        }
    }

    /** True while auto-shoot is toggled ON. */
    public boolean isAutoAimOn() {
        return autoShoot != null && autoShoot.isScheduled();
    }

    /** True while manual-shoot is toggled ON. */
    public boolean isManualShootOn() {
        return manualShoot != null && manualShoot.isScheduled();
    }

    /** True while either auto-shoot or manual-shoot is active. */
    public boolean isShooting() {
        return isAutoAimOn() || isManualShootOn();
    }

    private void applyDrive() {
        if (robot.follower == null || robot.follower.pose() == null) {
            return;
        }
        DrivePowers powers = ManualDrive.fieldCentric(
                driveForward(),
                driveStrafe(),
                driveTurn(),
                robot.follower.pose().heading());
        robot.follower.manual(powers);
    }

    private boolean isRed() {
        return RobotMain.CurrentAlliance != ALLIANCE_COLOR.ALLIANCE_BLUE;
    }

    /** Forward stick input (+ = forward). */
    public double driveForward() {
        return isRed() ? -DriveController.getLeftY() : DriveController.getLeftY();
    }

    /** Strafe stick input (+ = right). */
    public double driveStrafe() {
        return isRed() ? -DriveController.getLeftX() : DriveController.getLeftX();
    }

    /** Chassis turn stick input (+ = CCW). */
    public double driveTurn() {
        return -DriveController.getRightX();
    }

    /** Resets the robot pose and Pinpoint to the alliance start pose on Back button press. */
    private void handleRelocalize() {
        boolean pressed = DriveController.back().get();
        if (pressed && !prevBackPressed && robot.follower != null) {
            Pose resetPose = RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE
                    ? IronConstants.BLUE_RESET_POSE : IronConstants.RED_RESET_POSE;
            robot.follower.setPose(resetPose);
            robot.writePoseToPinpoint(resetPose);
        }
        prevBackPressed = pressed;
    }

    // ------------------------------------------------------------------ unused

    /**
     * Operator (gamepad2) controls for Flywheel PIDF dialing and autotuning:
     * - B: Nudge kF up (+0.5 coarse, +0.1 if leftTrigger held)
     * - X: Nudge kF down (-0.5 coarse, -0.1 if leftTrigger held)
     * - Dpad RIGHT: Nudge kP up (+1.0 coarse, +0.2 if leftTrigger held)
     * - Dpad LEFT: Nudge kP down (-1.0 coarse, -0.2 if leftTrigger held)
     * - A: Start / toggle Flywheel Autotune routine
     * - Dpad UP: Toggle Flywheel test spin to target RPM (pre-match check)
     */
    private void handleOperator() {
        if (robot.flywheel == null) {
            return;
        }

        boolean fine = OperatorController.getLeftTriggerAxis() > 0.2;

        boolean b = OperatorController.b().get();
        if (b && !prevOpB) {
            robot.flywheel.adjustKf(fine ? 0.10 : 0.50);
        }
        prevOpB = b;

        boolean x = OperatorController.x().get();
        if (x && !prevOpX) {
            robot.flywheel.adjustKf(fine ? -0.10 : -0.50);
        }
        prevOpX = x;

        boolean dpadRight = OperatorController.dpadRight().get();
        if (dpadRight && !prevOpDpadRight) {
            robot.flywheel.adjustKp(fine ? 0.20 : 1.0);
        }
        prevOpDpadRight = dpadRight;

        boolean dpadLeft = OperatorController.dpadLeft().get();
        if (dpadLeft && !prevOpDpadLeft) {
            robot.flywheel.adjustKp(fine ? -0.20 : -1.0);
        }
        prevOpDpadLeft = dpadLeft;

        boolean dpadUp = OperatorController.dpadUp().get();
        if (dpadUp && !prevOpDpadUp) {
            robot.flywheel.toggleTestSpin();
        }
        prevOpDpadUp = dpadUp;
    }

    // ------------------------------------------------------------------ telemetry

    /** Publishes TeleOp latch/pose telemetry (call after {@code RobotRunPeriodic()} so states match this loop). */
    public void reportTelemetry() {
        if (!RobotMain.DASHBOARD) {
            return;
        }
        // addData only — flushed once via robot.flushTelemetry() at end of loop().
        gamepadDebug.report(); // raw-vs-mapped gamepad axes (toggle in Panels > Configurables)
        RobotMain.DashTelemetry.addData("Shoot latch (LT toggle)", isAutoAimOn() ? "ON" : "OFF");
        RobotMain.DashTelemetry.addData("Manual shoot latch (RT toggle)", isManualShootOn() ? "ON" : "OFF");
        RobotMain.DashTelemetry.addData("Drive mode",
                isAutoAimOn() ? "SHOOT (latch ON, TURRET aims, chassis free)" :
                (isManualShootOn() ? "MANUAL SHOOT (latch ON, set RPM, turret fixed, chassis free)" : "manual field-centric"));

        if (robot.CommandF != null) {
            RobotMain.DashTelemetry.addData("Shoot readiness", robot.CommandF.getShootReadinessSummary());
        }
        if (robot.flywheel != null) {
            RobotMain.DashTelemetry.addData("Flywheel Tune", robot.flywheel.getStatusLine());
        }

        if (robot.turret != null) {
            RobotMain.DashTelemetry.addData("Turret (robot-rel)", "%+.1f deg",
                    robot.turret.getPosition());
        }
        if (robot.follower != null && robot.follower.pose() != null) {
            Pose p = robot.follower.pose();
            RobotMain.DashTelemetry.addData("Robot X", p.x());
            RobotMain.DashTelemetry.addData("Robot Y", p.y());
            RobotMain.DashTelemetry.addData("Robot Heading", Math.toDegrees(p.heading()));
        }
    }
}
