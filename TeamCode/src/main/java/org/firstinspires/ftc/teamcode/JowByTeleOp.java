package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.irondams.killerwatts.lib.SubsystemBase.RunPeriodic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.irondams.drivetrain.GyroMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.irondams.killerwatts.Drivetrain;
import org.firstinspires.ftc.teamcode.irondams.killerwatts.ServoMove;

@TeleOp

public class JowByTeleOp extends LinearOpMode {

    //private GyroMecanumDriveTrain DamDrive;
    private ServoMove servermovetest;
    private Drivetrain DT;
    @Override
    public void runOpMode() {;
        //DamDrive = new GyroMecanumDriveTrain(this,)
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        //build subsystems
        servermovetest = new ServoMove();
        DT = new Drivetrain(this);
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
