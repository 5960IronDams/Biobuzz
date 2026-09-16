package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.RunPeriodic;
import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.clearAll;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.killerwatts.ContinuousServo;
import org.firstinspires.ftc.teamcode.killerwatts.Drivetrain;
import org.firstinspires.ftc.teamcode.killerwatts.FieldTracker;
import org.firstinspires.ftc.teamcode.killerwatts.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.killerwatts.Intake;
import org.firstinspires.ftc.teamcode.killerwatts.PositionalServo;
import org.firstinspires.ftc.teamcode.killerwatts.ServoMove;

@TeleOp(name = "JowByTeleOp", group = "Robot")

public class JowByTeleOp extends LinearOpMode {

    //private GyroMecanumDriveTrain DamDrive;
    //private ServoMove servermovetest;
    private Drivetrain DT;
    private GoBildaPinpoint PinOdo;
    private FieldTracker tracker;
    private Intake intake;
    private PositionalServo PosServ;
    private ContinuousServo ContServ;

    @Override
    public void runOpMode() {
        clearAll(); // avoid double-registration on re-run
        //DamDrive = new GyroMecanumDriveTrain(this,)
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        //build subsystems — order matters: pinpoint before tracker (tracker reads pinpoint.pos).
        //servermovetest = new ServoMove();
        DT = new Drivetrain(this);
        PinOdo = new GoBildaPinpoint(this);
        tracker = new FieldTracker();
        intake = new Intake(this);
        tracker.setPinpoint(PinOdo);
        // Heading assist if pinpoint drops: Control-Hub IMU yaw (radians CCW+).
        tracker.setImuYawSupplier(() ->
                DT.GMDT.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        DT.setFieldTracker(tracker);

        PosServ = new PositionalServo(this);
        //ContServ = new ContinuousServo(this);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            RunPeriodic();
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
