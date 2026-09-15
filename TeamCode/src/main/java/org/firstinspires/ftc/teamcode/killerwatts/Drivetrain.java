package org.firstinspires.ftc.teamcode.killerwatts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.irondams.drivetrain.FourWheelDriveTrain;
import org.firstinspires.ftc.teamcode.irondams.drivetrain.GyroMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    public int loop = 0;
    public FourWheelDriveTrain FWDT;
    public GyroMecanumDriveTrain GMDT;
    public Gamepad gamepad1;
    public Drivetrain(LinearOpMode opMode)
    {
        FWDT = new FourWheelDriveTrain(opMode.hardwareMap);
        GMDT = new GyroMecanumDriveTrain(opMode,FWDT);
        gamepad1 = opMode.gamepad1;
    }

    @Override
    public void Periodic() {
        if(gamepad1.a){GMDT.reset();}
        // GMDT.drive expects (strafeRight+, forward+, turnCCW+).
        // left_stick_y up is negative, so forward = -left_stick_y.
        // right_stick_x right is positive, clockwise should be negative turn.
        GMDT.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.addData("drivetrain loop", loop);
        telemetry.addData("IMUYawAngle", GMDT.imu.getRobotYawPitchRollAngles().getYaw());
        loop++;
        telemetry.update();
    }
}
