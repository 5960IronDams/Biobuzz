package org.firstinspires.ftc.teamcode.irondams.odometry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * IMU sensor wrapper tailored for newer REV Control Hubs utilizing the universal IMU interface (BHI260AP or BMI088 chips).
 * Implements the IGyro interface to expose standard heading calculations and matching geometry tracking formats.
 */
    public class ControlHubImu implements IGyro {
        private final IMU imu;
     private double initYaw;

    /**
     * Constructs the Control Hub universal IMU hardware wrapper.
     *
     * @param hardwareMap The active hardware mapping context used to pull the IMU sensor instance.
     */
    public ControlHubImu(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        init();
    }

    /**
     * Initializes and configures the internal universal IMU orientation relative to the robot's physical mounting.
     * Sets up a standard forward-facing orientation alignment schema to establish a baseline reference zero heading.
     */
    private void init() {
        // Configure standard vertical/horizontal orientation alignment profiles
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        initYaw = orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Fetches current orientation data from the universal IMU interface structure.
     *
     * @return The current relative robot heading/yaw angle in degrees.
     */
    @Override
    public double update() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return -initYaw + orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Zeroes out the active orientation tracker, setting the robot's current heading as the new baseline zero angle.
     */
    @Override
    public void reset() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        initYaw = orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Pulls simulated three-dimensional pose orientations mapped into standard 2D formats.
     * Note: Control Hub IMU sensors do not track dead-reckoned spatial translations, so X and Y are returned as raw pitch/roll metrics.
     *
     * @return A Pose2D tracking representation containing absolute rotational headings.
     */
    @Override
    public Pose2D getPose() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return new Pose2D(
                DistanceUnit.INCH, 
                orientation.getRoll(AngleUnit.DEGREES), 
                orientation.getPitch(AngleUnit.DEGREES), 
                AngleUnit.DEGREES, 
                orientation.getYaw(AngleUnit.DEGREES)
        );
    }
}