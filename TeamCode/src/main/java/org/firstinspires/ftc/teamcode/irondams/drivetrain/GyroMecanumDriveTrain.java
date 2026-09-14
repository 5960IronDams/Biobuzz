package org.firstinspires.ftc.teamcode.irondams.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Field-centric (Field-Relative) Mecanum drivetrain implementation.
 * Uses an internal IMU gyro sensor to steer the robot relative to the field orientation
 * rather than the robot's heading, making it easier for drivers to control.
 */
public class GyroMecanumDriveTrain implements IDriveTrain {
    private double currentHorizontal = 0;
    private double currentVertical = 0;
    private double currentPivot = 0;

    private final BNO055IMU imu;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private Orientation angles = new Orientation();
    private double initYaw;

    private final FourWheelDriveTrain driveTrain;

    /**
     * Constructs a field-centric drivetrain wrapper.
     *
     * @param opMode     The active LinearOpMode context for telemetry and hardware mapping access.
     * @param driveTrain The baseline four-wheel motor configuration to pass powers to.
     */
    public GyroMecanumDriveTrain(LinearOpMode opMode, FourWheelDriveTrain driveTrain) {
        this.driveTrain = driveTrain;

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu2");

        initImu();
    }

    /**
     * Configures, initializes, and recalibrates the IMU gyroscope sensor.
     * Establishes the current orientation heading as the reference zero (Forward).
     */
    public void initImu() {
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = angles.firstAngle;
    }

    /**
     * Resets the gyroscope orientation heading, setting the robot's current heading as the new 'Forward'.
     */
    public void reset() {
        initImu();
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

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double zeroedYaw = -initYaw + angles.firstAngle;
        double theta = Math.atan2(y, x) * 180 / Math.PI; // aka angle
        double realTheta = (360 - zeroedYaw) + theta;
        double power = Math.hypot(x, y);
        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));
        double leftFront = (power * cos / maxSinCos + turn);
        double rightFront = (power * sin / maxSinCos - turn);
        double leftBack = (power * sin / maxSinCos + turn);
        double rightBack = (power * cos / maxSinCos - turn);

        if ((power + Math.abs(turn)) > 1) {
            leftFront /= power + turn;
            rightFront /= power - turn;
            leftBack /= power + turn;
            rightBack /= power - turn;
        }

        driveTrain.getLeftFrontDrive().setPower(leftFront);
        driveTrain.getRightFrontDrive().setPower(rightFront);
        driveTrain.getLeftBackDrive().setPower(leftBack);
        driveTrain.getRightBackDrive().setPower(rightBack);

//        reset();
    }
}