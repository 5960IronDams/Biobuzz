package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.gamepad.PanelsGamepad;
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
import org.firstinspires.ftc.teamcode.WattageLib.lib.CommandXboxController;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PanelsGamepadDebug;

/**
 * Single home for every gamepad binding in TeleOp.
 *
 * <p>Declarative SolversLib bindings set up once in the constructor:
 * <ol>
 *   <li>Drive: field-centric sticks on gamepad1 (Xbox axis remap via
 *       {@link CommandXboxController}; live raw-vs-mapped axes on Panels via
 *       {@link PanelsGamepadDebug}).</li>
 *   <li>Shoot: left trigger hold schedules {@code CommandF.ShootAtGoal} -
 *       aim at the closest hive + distance-to-hive LUT RPM + gate opens at
 *       speed; release closes the gate and coasts the flywheel to idle.</li>
 *   <li>Intake: right trigger press/release schedules
 *       {@code intake.holdCmd()} / {@code intake.stopCmd()}.</li>
 *   <li>Servo: X toggles presets via {@code PosServ.toggleCmd},
 *       dpad nudges while held.</li>
 *   <li>Localize: start resets the Pedro pose to the alliance start and
 *       re-anchors the Pinpoint to it.</li>
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
    /** Fluent bindings facade over gamepad1 — see bindCommands(). */
    private final CommandGamepad DriveController;
    /** Example-style Panels gamepad dump (PanelsGamepad.firstManager combined pad; Panels > Configurables > PanelsGamepadDebug). */
    private final PanelsGamepadDebug gamepadDebug;

    /** Non-null while the shoot compound is scheduled; it owns the follower turn axis. */
    private Command shootCmd = null;

    // Manual rising-edge latch for the start-button relocalize (kept as a simple
    // poll; no dependency on SDK *WasPressed APIs).
    private boolean prevStartPressed = false;

    public KeyBindings(OpMode opMode, RobotMain robot) {
        if (opMode == null || robot == null) {
            throw new IllegalArgumentException("KeyBindings needs a non-null OpMode and RobotMain");
        }
        this.robot = robot;
        this.DriveController = new CommandGamepad(opMode.gamepad1);
        this.gamepadDebug = new PanelsGamepadDebug(opMode.gamepad1, "GP1");
        bindCommands();
    }

    /**
     * All gamepad bindings, declared once at init. SolversLib bindings are polled
     * by {@code CommandScheduler.run()} (called from RobotMain.RobotRunPeriodic()).
     */
    private void bindCommands() {
        // ---- Shoot: left trigger hold-to-shoot ----
        // Compound (aim + LUT RPM + gate-at-speed) built ONCE here with the live
        // stick suppliers; while it is scheduled, applyDrive() skips manual drive
        // (the aim member already wrote the powers during scheduler execution).
        // interruptible=true so a new command can always take over if this one
        // lingers. On release the compound's end closes the gate + coasts the
        // flywheel to idle.
        shootCmd = robot.CommandF.ShootAtGoal(this::driveForward, this::driveStrafe);
        DriveController.leftTrigger(AimAtGoal.TRIGGER_THRESHOLD)
                .whileActiveOnce(shootCmd, true);

        // ---- Intake: right trigger press->run, release->stop ----
        // holdCmd is a RunCommand: INTAKE_RPM is re-read every loop, so
        // live Panels retunes apply immediately while the trigger is held.
        DriveController.rightTrigger(0.1)
                .whenActive(robot.intake.holdCmd())
                .whenInactive(robot.intake.stopCmd());

        // ---- Servo toggle: X, edge-triggered (persistent instance; reusable) ----
        DriveController.x().whenActive(robot.PosServ.toggleCmd());

        // ---- Servo nudge: dpad_up/dpad_down, run-while-held ----
        // Each command computes its own dt so nudging rate is loop-time
        // independent; the slew + hardware write still happen in
        // ShootGateServo.periodic(). Requires PosServ so dpad nudges cancel
        // any servo preset command still running.
        DriveController.dpadUp().whileActiveOnce(robot.PosServ.nudgeCmd(true), true);
        DriveController.dpadDown().whileActiveOnce(robot.PosServ.nudgeCmd(false), true);
    }

    /**
     * Refresh the controller's merged pad source for this tick: the Panels
     * Gamepad plugin's browser pad (which sees the Xbox controller's right
     * stick correctly, unlike the DS/Android HID lanes) layered over the DS
     * pad. MUST run before anything polls the controller (triggers in the
     * scheduler, drive reads). Called at the top of loop() via the OpMode and
     * again at the top of update() as a belt-and-suspenders.
     */
    public void refreshPad() {
        // DriveController is constructed as a CommandXboxController (Xbox remap
        // + browser-pad merge); the field keeps the parent type so any other
        // bindings stay drop-in compatible.
        if (DriveController instanceof CommandXboxController) {
            ((CommandXboxController) DriveController)
                    .refreshPad(PanelsGamepad.INSTANCE.getFirstManager());
        }
    }

    /** Poll every binding once. Call once per loop, after {@code robot.RobotRunPeriodic()}. */
    public void update() {
        refreshPad();
        handleRelocalize();
        handleOperator();
        applyDrive();
        reportTelemetry();
    }

    /** Cancel a held shot (call from the OpMode's stop()). Safe when not shooting. */
    public void stop() {
        cancelShoot();
    }

    /** True while the shoot compound owns the follower turn axis. */
    public boolean isShooting() {
        return shootCmd != null && shootCmd.isScheduled();
    }

    // ------------------------------------------------------------------ drive + shoot

    /**
     * When shooting, the aim member of the compound already called
     * {@code follower.manual()} during {@code CommandScheduler.run()} - just
     * refresh the follower so its powers apply. Otherwise, drive field-centric
     * off the sticks.
     */
    private void applyDrive() {
        if (robot.follower == null || robot.follower.pose() == null) {
            return;
        }
        if (shootCmd == null || !shootCmd.isScheduled()) {
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

    private void cancelShoot() {
        if (shootCmd != null && shootCmd.isScheduled()) {
            shootCmd.cancel();
        }
    }

    // Alliance-aware stick mapping (preserved from JowByTeleOp).
    private boolean isRed() {
        return RobotMain.CurrentAlliance != ALLIANCE_COLOR.ALLIANCE_BLUE;
    }

    /** Forward stick (+ = forward). Also fed to AimAtGoal so translation survives aiming. */
    public double driveForward() {
        return isRed() ? -DriveController.getLeftY() : DriveController.getLeftY();
    }

    /** Strafe stick (+ = right). Also fed to AimAtGoal so translation survives aiming. */
    public double driveStrafe() {
        return isRed() ? -DriveController.getLeftX() : DriveController.getLeftX();
    }

    /** Turn stick (+ = turn). Ignored while shooting (aim owns turn). */
    public double driveTurn() {
        return -DriveController.getRightX();
    }

    // ------------------------------------------------------------------ subsystems

    /**
     * Start button: teleport the Pedro pose to the alliance start (edge-triggered).
     * Kept as a manual poll — simple edge detection is fine here.
     */
    private void handleRelocalize() {
        boolean pressed = DriveController.back().get();
        if (pressed && !prevStartPressed && robot.follower != null) {
            Pose resetPose = RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE
                    ? IronConstants.BLUE_RESET_POSE : IronConstants.RED_RESET_POSE;
            robot.follower.setPose(resetPose);
            // Re-anchor the raw Pinpoint to the same pose. Without this the
            // dead reckoning keeps its old frame: the fusion history and the
            // next OpMode's Pinpoint seed would disagree with the pose this
            // button just set. (One-tick velocity transient is acceptable for
            // an emergency re-zero.)
            robot.writePoseToPinpoint(resetPose);
        }
        prevStartPressed = pressed;
    }

    // ------------------------------------------------------------------ unused

    /** Operator (gamepad2) bindings - nothing assigned yet; add flywheel/etc. here. */
    private void handleOperator() {
        // TODO: move operator controls here as mechanism commands land, e.g.
        // new Trigger(() -> gamepad2.y).whenActive(robot.flywheel.toRpm(2000));
    }

    // ------------------------------------------------------------------ telemetry

    private void reportTelemetry() {
        // addData only — flushed once via robot.flushTelemetry() at end of loop().
        gamepadDebug.report(); // raw-vs-mapped gamepad axes (toggle in Panels > Configurables)
        RobotMain.DashTelemetry.addData("Drive mode",
                isShooting() ? "SHOOT (LT held, sticks = translate)" : "manual field-centric");
        if (robot.follower != null && robot.follower.pose() != null) {
            Pose p = robot.follower.pose();
            RobotMain.DashTelemetry.addData("Robot X", p.x());
            RobotMain.DashTelemetry.addData("Robot Y", p.y());
            RobotMain.DashTelemetry.addData("Robot Heading", Math.toDegrees(p.heading()));
        }
    }
}
