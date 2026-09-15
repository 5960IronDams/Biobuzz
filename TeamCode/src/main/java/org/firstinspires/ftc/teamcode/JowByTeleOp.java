package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.RunPeriodic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.killerwatts.Drivetrain;
import org.firstinspires.ftc.teamcode.killerwatts.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.killerwatts.ServoMove;

@TeleOp(name = "JowByTeleOp", group = "Robot")

public class JowByTeleOp extends LinearOpMode {

    //private GyroMecanumDriveTrain DamDrive;
    private ServoMove servermovetest;
    private Drivetrain DT;
    private GoBildaPinpoint PinOdo;
    @Override
    public void runOpMode() {;
        //DamDrive = new GyroMecanumDriveTrain(this,)
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        //build subsystems
        servermovetest = new ServoMove();
        DT = new Drivetrain(this);
        PinOdo = new GoBildaPinpoint(this);
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
