package org.firstinspires.ftc.teamcode.irondams.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * IMU sensor wrapper tailored for older or external Expansion Hubs utilizing the Bosch BNO055 chip.
 * Implements the IGyro interface to expose standard heading calculations and basic pose geometry.
 */
public class ExpansionHubImu implements IGyro {
    private final BNO055IMU imu;

    private Orientation angles = new Orientation();
    private double initYaw;

    /**
     * Constructs the Expansion Hub IMU hardware wrapper.
     *
     * @param hardwareMap The active hardware mapping context used to pull the IMU sensor instance.
     */
    public ExpansionHubImu(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        init();
    }

    /**
     * Initializes and calibrates the internal BNO055 gyroscope sensor parameters.
     * Captures the startup configuration layout to establish a baseline reference zero heading.
     */
    private void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = angles.firstAngle;
    }

    /**
     * Fetches current orientation data from the hardware chip and updates internal fields.
     *
     * @return The current relative robot heading/yaw angle in degrees.
     */
    @Override
    public double update() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -initYaw + angles.firstAngle;
    }

    /**
     * Zeroes out the active orientation tracker, setting the robot's current heading as the new baseline zero angle.
     */
    @Override
    public void reset() {
        initYaw = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * Pulls simulated three-dimensional pose orientations mapped into standard 2D formats.
     * Note: BNO055 chips do not track dead-reckoned spatial translations, so X and Y are returned as raw pitch/roll metrics.
     *
     * @return A Pose2D tracking representation containing absolute rotational headings.
     */
    @Override
    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.INCH, angles.thirdAngle, angles.secondAngle, AngleUnit.DEGREES, angles.firstAngle);
    }
}