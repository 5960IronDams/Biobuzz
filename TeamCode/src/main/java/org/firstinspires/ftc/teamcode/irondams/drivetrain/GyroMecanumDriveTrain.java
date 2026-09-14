package org.firstinspires.ftc.teamcode.irondams.drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-centric (Field-Relative) Mecanum drivetrain implementation.
 * Uses an internal IMU gyro sensor to steer the robot relative to the field orientation
 * rather than the robot's heading, making it easier for drivers to control.
 */
public class GyroMecanumDriveTrain implements IDriveTrain {
    private double currentHorizontal = 0;
    private double currentVertical = 0;
    private double currentPivot = 0;

    public final IMU imu;

    private final FourWheelDriveTrain driveTrain;

    /**
     * Constructs a field-centric drivetrain wrapper.
     *
     * @param opMode     The active LinearOpMode context for telemetry and hardware mapping access.
     * @param driveTrain The baseline four-wheel motor configuration to pass powers to.
     */
    public GyroMecanumDriveTrain(LinearOpMode opMode, FourWheelDriveTrain driveTrain) {
        this.driveTrain = driveTrain;

        imu = opMode.hardwareMap.get(IMU.class, "imu2");

        initImu();
    }

    /**
     * Configures, initializes, and recalibrates the IMU gyroscope sensor.
     * Establishes the current orientation heading as the reference zero (Forward).
     */
    public void initImu() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    /**
     * Resets the gyroscope orientation heading, setting the robot's current heading as the new 'Forward'.
     */
    public void reset() {
        imu.resetYaw();
    }

    /**
     * Translates directional vector commands into field-centric motor behaviors.
     * Defaults to an estimated fixed loop interval calculation.
     *
     * @param x    Desired horizontal strafe speed [-1.0, 1.0].
     * @param y    Desired vertical forward/reverse speed [-1.0, 1.0].
     * @param turn Desired rotational steering speed [-1.0, 1.0].
     */
    @Override
    public void drive(double x, double y, double turn) {
        driveWithTime(x, y, turn, 0.05); // Default to 20Hz timing if time not provided
    }

    /**
     * Translates directional vector commands into field-centric motor behaviors,
     * utilizing a precise deltaTime step to smoothly ramp input logic.
     *
     * @param x            Desired horizontal strafe speed [-1.0, 1.0].
     * @param y            Desired vertical forward/reverse speed [-1.0, 1.0].
     * @param turn         Desired rotational steering speed [-1.0, 1.0].
     * @param deltaTimeSec Exact time duration elapsed in seconds since the last loop iteration.
     */
    public void driveWithTime(double x, double y, double turn, double deltaTimeSec) {
        double maxDeltaPerSec = 3.0; // Adjustable ramp speed

        double newHorizontal = Acceleration.rampPower(currentHorizontal, x, maxDeltaPerSec, deltaTimeSec);
        x = newHorizontal;
        currentHorizontal = newHorizontal;

        double newVertical = Acceleration.rampPower(currentVertical, y, maxDeltaPerSec, deltaTimeSec);
        y = newVertical;
        currentVertical = newVertical;

        double newPivot = Acceleration.rampPower(currentPivot, turn, maxDeltaPerSec, deltaTimeSec);
        turn = newPivot;
        currentPivot = newPivot;

        // Field-centric transform: rotate the driver input by -yaw so that
        // pushing the stick away from you always drives away from you,
        // regardless of which way the robot is facing.
        // yaw = 0 means the robot is facing the field-forward direction set by resetYaw().
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-yaw) - y * Math.sin(-yaw);
        double rotY = x * Math.sin(-yaw) + y * Math.cos(-yaw);

        // Same robot-relative mixing as MecanumDriveTrain so both modes agree.
        // rotX = strafe (right +), rotY = forward (+), turn = rotate (CCW +).
        double leftFront = rotY + rotX + turn;
        double rightFront = rotY - rotX - turn;
        double leftBack = rotY - rotX + turn;
        double rightBack = rotY + rotX - turn;

        // Normalize so no power exceeds 1.0 while keeping direction proportions.
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                Math.max(Math.abs(leftBack), Math.abs(rightBack)));
        if (max > 1.0) {
            leftFront /= max;
            rightFront /= max;
            leftBack /= max;
            rightBack /= max;
        }

        driveTrain.getLeftFrontDrive().setPower(leftFront);
        driveTrain.getRightFrontDrive().setPower(rightFront);
        driveTrain.getLeftBackDrive().setPower(leftBack);
        driveTrain.getRightBackDrive().setPower(rightBack);
    }
}