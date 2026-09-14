package org.firstinspires.ftc.teamcode.irondams.odometry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Common abstraction interface representing a gyroscopic orientation sensor or heading provider.
 * Allows pulling orientation updates, resetting coordinate tracking origins, and fetching global 2D poses
 * from a unified tracking system (such as an internal IMU or an external odometry computer like a goBILDA Pinpoint).
 */
public interface IGyro {

    /**
     * Polls the hardware or tracking system for the latest orientation data.
     * Should be called regularly inside the main OpMode loop.
     *
     * @return The current robot heading/yaw angle in degrees or radians depending on implementation.
     */
    double update();

    /**
     * Resets the sensor's heading tracking tracking origin back to zero.
     * Sets the robot's current physical position/heading as the reference starting point.
     */
    void reset();

    /**
     * Fetches the full two-dimensional positioning pose from the sensor subsystem.
     *
     * @return A Pose2D object representing the robot's relative X position, Y position, and heading angle.
     */
    Pose2D getPose();
}