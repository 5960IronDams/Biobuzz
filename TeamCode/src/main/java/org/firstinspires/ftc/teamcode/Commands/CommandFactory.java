package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ShootGateServo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class CommandFactory {

    public final Follower follower;
    public final Intake intake;
    public final Flywheel flywheel;
    public final ShootGateServo PosServ;
    public final HardwareMap hardwareMap;
    /** Shared dt tracker for the nudge commands (rate * dt, loop-time independent). */
    private long lastNudgeNs = -1L;
    public CommandFactory(Follower _follower, Intake _intake, Flywheel _flywheel, ShootGateServo _posServ, HardwareMap _hardwareMap) {
        follower = _follower;
        intake = _intake;
        flywheel = _flywheel;
        PosServ = _posServ;
        hardwareMap = _hardwareMap;
    }

    /**
     * Fire-and-forget: spin the intake at INTAKE_RPM once (the motor keeps
     * spinning until StopIntake). For autos / sequencing - pairs with
     * {@link #StopIntake} in a sequence. NOT Panels-live-tunable.
     */
    public Command RunIntake() {
        return new InstantCommand(()->{intake.runIntake();}, intake);
    }
    /**
     * Continuous hold: re-asserts INTAKE_RPM every scheduler loop, so live
     * Panels retunes apply immediately while held. For TeleOp trigger
     * bindings - releases should bind {@link #StopIntake}.
     */
    public Command HoldIntakeTunable() {
        return new RunCommand(()->{intake.runIntake();}, intake);
    }
    public Command StopIntake() {
        return new InstantCommand(()->{intake.stop();}, intake);
    }

    /**
     * Spin the flywheel to an explicit RPM via velocity PID (fire-and-forget).
     * The PID hold runs in {@code Flywheel.periodic()}, so this finishes instantly -
     * sequence on {@code flywheel.isAtTargetRpm()} (e.g. {@code .until(...)}) if the
     * next step needs it up to speed first.
     */
    public Command SetFlywheelRpm(double rpm) {
        return new InstantCommand(()->{flywheel.setTargetRpm(rpm);}, flywheel);
    }
    /**
     * Hold the flywheel at TARGET_RPM, re-asserting every loop so live Panels
     * retunes of TARGET_RPM apply immediately (PID hold also runs in Periodic()).
     */
    public Command HoldFlywheelTunable() {
        return new RunCommand(()->{flywheel.setTargetRpm(Flywheel.TARGET_RPM);}, flywheel);
    }
    /** PID-brake the flywheel to zero. */
    public Command StopFlywheel() {
        return new InstantCommand(()->{flywheel.stop();}, flywheel);
    }

    /**
     * Nudge the servo target each loop the command runs (loop-time independent:
     * dt is measured internally). Rate and direction per ShootGateServo.
     *
     * @param up true nudges toward one end, false toward the other (matches the
     *           dpad_up/dpad_down binding directions)
     */
    public Command NudgeServo(boolean up) {
        return new RunCommand(
                () -> {
                    long nowNs = System.nanoTime();
                    double dt = lastNudgeNs < 0 ? 0.02 : (nowNs - lastNudgeNs) / 1.0e9;
                    dt = Math.min(Math.max(dt, 0.0), 0.25);
                    lastNudgeNs = nowNs;
                    PosServ.nudge((up ? -1 : 1) * ShootGateServo.NUDGE_RATE * dt);
                },
                PosServ);
    }
    /**
     * Cut drive (FLOAT) and free-spin toward idle, then re-engage the normal PID
     * to hold idle on arrival. Finishes when {@code flywheel.isHoldingIdle()} -
     * i.e. coast arrived AND PID is holding idle - so autos can sequence on it.
     */
    public Command CoastFlywheelToIdle() {
        return new FunctionalCommand(
                ()->{flywheel.coastToIdle();},   // initialize
                ()->{},                          // execute (PID hold runs in Periodic())
                (interrupted)->{},               // end
                ()->{return flywheel.isHoldingIdle();}, // isFinished
                flywheel);                       // requirement
    }

    /**
     * Hold-to-aim: faces the closest AimPoints.pp target for the current alliance
     * (Red* / Blue* filtered, re-picked every loop) while {@code forward}/{@code strafe}
     * keep the driver on translation. Runs until cancelled - hold the left trigger
     * to aim, compose with the flywheel via {@code .until(...)} / parallel / deadline.
     *
     * <p>Pass the same alliance-aware stick mapping the TeleOp uses, e.g. for Red:
     * {@code () -> -gamepad1.left_stick_y} and {@code () -> -gamepad1.left_stick_x}.
     */
    public Command AimAtClosestGoal(DoubleSupplier forward, DoubleSupplier strafe) {
        return new AimAtGoal(follower, hardwareMap, forward, strafe, () -> RobotMain.CurrentAlliance);
    }

    public Command ServoTogglePos = new CommandBase() {
        // Requirements resolved lazily: this field initializer runs before the
        // constructor body assigns PosServ, so capture it at query time instead.
        @Override
        public Set<Subsystem> getRequirements() {
            return Set.of(PosServ);
        }
        //Initialize Happens when the Command is called, it happens ONCE.
        @Override
        public void initialize() {
            PosServ.toggle(); //toggle currently just "goes to other position"
        }
        //Execute is a loop of actions every system loop, will repeat until command is completed or interrupted.
        @Override
        public void execute() {

        }
        //isFinished is the "is it finished?" question. return true to be done, or false to run forever until interrupted.
        @Override
        public boolean isFinished() {
            return PosServ.isAtPosition();
        }
        //End is the final actions taken when this command is ended (interruption, isFinished returns true etc.)
        @Override
        public void end(boolean interrupted) {

        }
    };

    public Command ServoToPos(double GotoPos) {
        return new FunctionalCommand(
                ()->{PosServ.setPosition(GotoPos);}, // initialize
                ()->{},                              // execute (slew runs in Periodic())
                (interrupted)->{},                   // end
                ()->{return PosServ.isAtPosition();}, // isFinished
                PosServ);                            // requirement
    }
}
