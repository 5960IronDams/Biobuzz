package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import org.firstinspires.ftc.teamcode.killerwatts.Intake;
import org.firstinspires.ftc.teamcode.killerwatts.PositionalServo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collections;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class CommandFactory {

    public final Follower follower;
    public final Intake intake;
    public final PositionalServo PosServ;
    public final HardwareMap hardwareMap;
    public CommandFactory(Follower _follower, Intake _intake, PositionalServo _posServ, HardwareMap _hardwareMap) {
        follower = _follower;
        intake = _intake;
        PosServ = _posServ;
        hardwareMap = _hardwareMap;
    }

    public Command RunIntake() {
        return instant(()->{intake.runIntake();}).requiring(intake);
    }
    public Command StopIntake() {
        return instant(()->{intake.stop();}).requiring(intake);
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

    public Command ServoTogglePos = new Command() {
        //Start Happens when the Command is called, it happens ONCE.
        @Override
        public void start() {
            PosServ.toggle(); //toggle currently just "goes to other position"
        }
        //Execute is a loop of actions every system loop, will repeat until command is completed or interrupted.
        @Override
        public void execute() {

        }
        //done is the "is it finished?" question. return true to be done, or false to run forever until interrupted.
        @Override
        public boolean done() {
            return PosServ.isAtPosition();
        }
        //End is the final actions taken when this command is ended (interruption,done returns true etc.)
        @Override
        public void end(EndCondition endCondition) {

        }
        //requirements are the subsystem requirements for the command, not always needed, but it helps to have.
        @Override
        public Set<Object> requirements() {
            return Set.of(PosServ);
        }

        @Override
        public int priority() {
            return 0;
        }

        @Override
        public InterruptedBehavior interruptedBehavior() {
            return null;
        }

        @Override
        public ConflictBehavior conflictBehavior() {
            return null;
        }

        @Override
        public BlockedBehavior blockedBehavior() {
            return null;
        }


    };

    public Command ServoToPos(double GotoPos) {
        return Command.build()
                .requiring(PosServ)
                .setStart(()->{PosServ.setPosition(GotoPos);})
                .setDone(()->{return PosServ.isAtPosition();});

    }
}
