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
 * Single home for every gamepad binding in TeleOp.
 *
 * <p>Declarative SolversLib bindings set up once in the constructor:
 * <ol>
 *   <li>Drive: field-centric sticks on gamepad1, read through
 *       {@link CommandGamepad} (the LOGITECH pad — plain SDK axes, no remap).
 *       Live raw axes on Panels via {@link PanelsGamepadDebug}. The chassis is
 *       ALWAYS driver-controlled — aiming never touches it. Right stick X =
 *       chassis turn.</li>
 *   <li><b>Aim / auto-fire: LEFT TRIGGER is a TOGGLE</b> (press to start,
 *       press again to stop) — {@code toggleWhenActive} on the latched
 *       auto-shoot compound. While ON the turret tracks the closest alliance
 *       target, the flywheel holds the LUT RPM, and the TRANSFER fires itself
 *       the moment {@code isReadyToShoot()} says the shot is good. This
 *       replaces the old hold-to-aim push-to-talk, which needed a second
 *       button to unlock; <b>LEFT BUMPER is now the unlock/panic</b> (latch
 *       off + gate closed + transfer stopped + shooter parked).</li>
 *   <li>Turret: normally FULLY AUTO while the latch is ON. Dpad LEFT/RIGHT is
 *       the manual/debug jog for when the latch is OFF (via
 *       {@code TurretPivot.NudgeDeg} so soft limits + PID safety stay
 *       active).</li>
 *   <li>Intake: right trigger press/release schedules
 *       {@code intake.holdCmd()} / {@code intake.stopCmd()}. While the shot
 *       is firing the transfer parks the intake itself and hands it back
 *       afterwards (see {@code CommandFactory.FireAtGoal}).</li>
 *   <li>Transfer: LEFT BUMPER moved to the panic/unlock, so both manual test
 *       directions are on the right cluster — hold RIGHT BUMPER to run
 *       forward, hold RIGHT STICK CLICK (R3) to run reverse; either release
 *       PID-brakes to zero.</li>
 *   <li>Servo: X toggles presets via {@code PosServ.toggleCmd},
 *       dpad up/down nudges while held.</li>
 *   <li>Localize: BACK resets the Pedro pose to the alliance start and
 *       re-anchors the Pinpoint to it.</li>
 * </ol>
 *
 * <p>Usage (see {@code JowByTeleOp}):
 * <pre>
 * keys = new KeyBindings(this, robot);   // init(), after RobotMain
 * robot.RobotRunPeriodic();              // loop(), runs CommandScheduler
 * keys.update();                         // loop(), right after
 * </pre>
 * {@code update()} reports the intake trigger intent, ticks the shoot-readiness
 * motion bookkeeping ({@code CommandFactory.readinessPeriodic()}) and applies
 * the follower powers. The field draw happens once per loop in
 * {@code flushTelemetry()}, after this returns.
 */
public class KeyBindings {

    private final RobotMain robot;
    /** Fluent bindings facade over gamepad1 — see bindCommands(). */
    private final CommandGamepad DriveController;
    /** Example-style Panels gamepad dump (Panels > Configurables > PanelsGamepadDebug). */
    private final PanelsGamepadDebug gamepadDebug;

    // ---- Manual turret jog (dpad left/right) ----
    /**
     * Jog step per loop while a dpad direction is held (deg). NudgeDeg
     * re-targets the PID every loop, and the subsystem's edge slow zone still
     * protects the soft limits.
     */
    public static final double MANUAL_JOG_STEP_DEG = 4.0;

    /** Right-trigger threshold that starts/stops the manual intake (binding + intent read use it). */
    public static final double INTAKE_TRIGGER_THRESHOLD = 0.1;

    // Manual rising-edge latch for the back-button relocalize (kept as a simple
    // poll; no dependency on SDK *WasPressed APIs).
    private boolean prevBackPressed = false;

    // The left-trigger toggle's latched state is owned by the COMMAND itself:
    // toggleWhenActive() schedules/cancels the same instance, so "is it on" is
    // simply isScheduled(). That keeps the button and the robot in lockstep
    // even if something else (an auto, the panic button) ends the command.
    private final Command autoShoot;

    public KeyBindings(OpMode opMode, RobotMain robot) {
        if (opMode == null || robot == null) {
            throw new IllegalArgumentException("KeyBindings needs a non-null OpMode and RobotMain");
        }
        this.robot = robot;
        this.DriveController = new CommandGamepad(opMode.gamepad1);
        this.gamepadDebug = new PanelsGamepadDebug(opMode.gamepad1, "GP1");
        // One instance, reused by the toggle below: its scheduled state IS the latch.
        this.autoShoot = robot.CommandF.getAutoShootCommand();
        bindCommands();
    }

    /**
     * All gamepad bindings, declared once at init. SolversLib bindings are polled
     * by {@code CommandScheduler.run()} (called from RobotMain.RobotRunPeriodic()).
     */
    private void bindCommands() {
        // ---- Manual turret jog: dpad LEFT / dpad RIGHT ----
        // Normally the turret is AUTO-aimed. The dpad is the manual/debug
        // path: hold dpad-left to jog CCW (+), dpad-right for CW (−), through
        // NudgeDeg so the subsystem's soft limits + PID + watchdogs stay in
        // the loop. GATED ON !isAutoAimOn(): while the auto-shoot latch is on
        // the aim is fully automatic and the dpad does nothing to the turret.
        // Guarded: turret is optional hardware (tryCreate may return null).
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

        // ---- SHOOT: LEFT TRIGGER = TOGGLE (press on, press again off) ----
        // toggleWhenActive schedules the latched auto-shoot compound on the
        // first press and cancels it on the next. One button owns the whole
        // shot cycle: aim + flywheel RPM + gate + TRANSFER, the transfer
        // feeding automatically whenever CommandFactory.isReadyToShoot() is
        // true. No second "unlock" press is needed to keep shooting — press
        // once to shoot, press once to stop. interruptible=true.
        if (robot.turret != null && autoShoot != null) {
            DriveController.leftTrigger(AimAtGoal.TRIGGER_THRESHOLD)
                    .toggleWhenActive(autoShoot, true);
        }

        // ---- PANIC / UNLOCK: LEFT BUMPER ----
        // The old "one extra button to unlock" job, now the ONE dedicated safe
        // control: turn the latch OFF and re-park the shooter (gate closed,
        // transfer stopped, flywheel coasts to idle, turret released).
        // getStopShootCommand() is idempotent, so hitting it when already off
        // just re-asserts the parked state. This is also what stops the
        // auto-transfer mid-shot.
        DriveController.leftBumper().whenActive(robot.CommandF.getStopShootCommand());

        // ---- Intake: right trigger press->run, release->stop ----
        // holdCmd is a RunCommand: INTAKE_RPM is re-read every loop, so
        // live Panels retunes apply immediately while the trigger is held.
        // While the shot is firing the transfer parks the intake itself
        // (see CommandFactory.FireAtGoal) and hands it back when it is done —
        // setManualIntakeHeld() below is what tells it whether the driver
        // still wants it running.
        DriveController.rightTrigger(INTAKE_TRIGGER_THRESHOLD)
                .whenActive(robot.intake.holdCmd())
                .whenInactive(robot.intake.stopCmd());
        DriveController.a()
                .whenActive(robot.intake.reverseCmd())
                .whenInactive(robot.intake.stopCmd());

        // ---- Transfer (bench test): RB = forward, R3 = reverse ----
        // LB moved to the panic button, so BOTH manual transfer directions
        // live on the right side: hold RB to run forward, hold the right
        // stick click (R3) to run reverse. Either release PID-brakes to zero.
        // A manual transfer press also OVERRIDES the auto-fire for a moment
        // (see CommandFactory.FIRE_OVERRIDE_HOLD_S) so a jam can be cleared.
        DriveController.rightBumper()
                .whenActive(robot.transfer.holdCmd())
                .whenInactive(robot.transfer.stopCmd());
        DriveController.b()
                .whenActive(robot.transfer.reverseHoldCmd())
                .whenInactive(robot.transfer.stopCmd());

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

    /** Poll every binding once. Call once per loop, after {@code robot.RobotRunPeriodic()}. */
    public void update() {
        // Tell the shoot factory whether the driver still wants the intake, so
        // the auto-fire can hand it back after it got it out of the way. Same
        // threshold as the binding above, so "held" means exactly what started
        // holdCmd().
        robot.CommandF.setManualIntakeHeld(robot.intake != null
                && DriveController.rightTrigger(INTAKE_TRIGGER_THRESHOLD).get());
        // Shoot-readiness sensing: advances the debounce/warmup timers and
        // samples the chassis motion from the follower pose. Home for both is
        // one call — the scheduler already polled this loop's triggers, so this
        // tick's sample is what the NEXT tick's isReadyToShoot()/FireAtGoal()
        // sees (the pose has already advanced by applyDrive() below).
        robot.CommandF.readinessPeriodic();
        handleRelocalize();
        handleOperator();
        applyDrive();
        reportTelemetry();
    }

    /** Cancel the latched shot (call from the OpMode's stop()). Safe when not shooting. */
    public void stop() {
        if (autoShoot != null && autoShoot.isScheduled()) {
            autoShoot.cancel();
        }
    }

    /** True while the latched auto-shoot (aim + transfer) is ON. */
    public boolean isAutoAimOn() {
        return autoShoot != null && autoShoot.isScheduled();
    }

    /** Legacy alias kept for anything that asked "are we shooting". */
    public boolean isShooting() {
        return isAutoAimOn();
    }

    // ------------------------------------------------------------------ drive

    /**
     * The chassis is ALWAYS manual field-centric: aiming spins the turret and
     * never touches the follower, so the driver keeps translation AND turn
     * even while the shot cycle runs.
     */
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
        robot.follower.update();
        // Correct AFTER the trailing update (same predict->correct order as RobotMain).
        // Same-frame re-poll no-ops inside VisionFusion; a new LL frame still fuses.
        // No field draw here — flushTelemetry() paints once per loop (final pose).
        if (robot.visionFusion != null) robot.visionFusion.correct();
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

    /**
     * Chassis turn stick (+ = CCW). The right stick is chassis turning
     * (standard drive layout); the turret's manual jog lives on dpad
     * LEFT/RIGHT instead. Normally the turret is auto-aimed and the stick is
     * just driving.
     */
    public double driveTurn() {
        return -DriveController.getRightX();
    }

    // ------------------------------------------------------------------ subsystems

    /**
     * Back button: teleport the Pedro pose to the alliance start
     * (edge-triggered). Kept as a manual poll — simple edge detection is fine
     * here.
     */
    private void handleRelocalize() {
        boolean pressed = DriveController.back().get();
        if (pressed && !prevBackPressed && robot.follower != null) {
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
        prevBackPressed = pressed;
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
        RobotMain.DashTelemetry.addData("Shoot latch (LT toggle)", isAutoAimOn() ? "ON" : "OFF");
        RobotMain.DashTelemetry.addData("Drive mode",
                isAutoAimOn() ? "SHOOT (latch ON, TURRET aims, chassis free)" : "manual field-centric");
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
