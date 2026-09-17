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
    public FourWheelDriveTrain FWDT;
    public GyroMecanumDriveTrain GMDT;
    public Gamepad gamepad1;

    /** Optional velocity feed so FieldTracker can dead-reckon without pinpoint. */
    public static double MAX_CMD_VEL_IN_PER_SEC = 40.0;
    public static double MAX_CMD_OMEGA_DEG_PER_SEC = 120.0;//default 180
    private FieldTracker fieldTracker;

    /** Wire after constructing both: dt.setFieldTracker(tracker). */
    public void setFieldTracker(FieldTracker tracker) {
        this.fieldTracker = tracker;
    }
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
        // FieldTracker listens via these hooks; set it with setFieldTracker().
        double strafe = gamepad1.left_stick_x;
        double fwd = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        GMDT.drive(strafe, fwd, turn);
        if (fieldTracker != null) {
            // Commanded-velocity feed for dead-reckoning when pinpoint is absent.
            // FieldTracker.noteVelocity takes FIELD frame at start heading 0:
            // fwd+ -> +Y (up-field), strafe-right+ -> +X (east, right from red wall).
            fieldTracker.noteVelocity(strafe * MAX_CMD_VEL_IN_PER_SEC,
                    fwd * MAX_CMD_VEL_IN_PER_SEC,
                    -turn * Math.toRadians(MAX_CMD_OMEGA_DEG_PER_SEC));
        }
        telemetry.addData("IMUYawAngle", GMDT.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.update();
    }
}
