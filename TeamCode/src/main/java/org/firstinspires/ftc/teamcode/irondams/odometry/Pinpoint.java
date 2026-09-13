package org.firstinspires.ftc.teamcode.irondams.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Odometry tracking system wrapper built around the goBILDA Pinpoint Odometry Computer.
 * Implements the IGyro interface to fetch absolute X/Y field coordinates alongside integrated gyro headings.
 */
public class Pinpoint implements IGyro {
    private final GoBildaPinpointDriver pinpoint;

    private double initYaw;

    /**
     * Constructs a goBILDA Pinpoint tracker module wrapper.
     *
     * @param opMode The active LinearOpMode context used to pull the specialized device registry object.
     */
    public Pinpoint(LinearOpMode opMode) {
        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        init();
    }

    /**
     * Initialized tracking pods configurations, sets encoder calibration tick values, 
     * assigns forward directional orientations, and flushes initial tracking states to zero.
     */
    private void init() {
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        initYaw = getPose().getHeading(AngleUnit.DEGREES);
    }

    /**
     * Instructs the hardware system to query pod tracking data and compiles relative yaw results.
     *
     * @return The offset heading relative to its reference zero index in degrees.
     */
    @Override
    public double update() {
        return -initYaw + getPose().getHeading(AngleUnit.DEGREES);
    }

    /**
     * Resets the entire internal tracking system coordinates back to (X: 0, Y: 0, Heading: 0).
     */
    @Override
    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    /**
     * Overrides the current heading angle field, referencing it as the new baseline zero angle.
     */
    public void resetYaw() {
        initYaw = getPose().getHeading(AngleUnit.DEGREES);
    }


    /**
     * Requests updated positional states and returns dead-reckoned spatial configurations.
     *
     * @return A Pose2D tracking representation containing absolute field tracking parameters.
     */
    @Override
    public Pose2D getPose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }
}