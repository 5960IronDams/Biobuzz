package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ShootGateServo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

/**
 * Multi-subsystem compositions only. Single-subsystem commands live on their
 * subsystem ({@code intake.runCmd()/holdCmd()/stopCmd()},
 * {@code flywheel.toRpm()/holdTunable()/stopCmd()/coastToIdleCmd()},
 * {@code PosServ.toggleCmd()/toPos()/nudgeCmd()}).
 */
public class CommandFactory {

    public final Follower follower;
    public final Intake intake;
    public final Flywheel flywheel;
    public final ShootGateServo PosServ;
    public final HardwareMap hardwareMap;
    public CommandFactory(Follower _follower, Intake _intake, Flywheel _flywheel, ShootGateServo _posServ, HardwareMap _hardwareMap) {
        follower = _follower;
        intake = _intake;
        flywheel = _flywheel;
        PosServ = _posServ;
        hardwareMap = _hardwareMap;
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
}
