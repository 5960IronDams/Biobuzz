
package org.firstinspires.ftc.teamcode.killerwatts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.killerwatts.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase;
import org.firstinspires.ftc.teamcode.killerwatts.lib.VisionMeasurement;

import java.util.Locale;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */


public class GoBildaPinpoint extends SubsystemBase {

    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    public Gamepad gamepad1;
    public Supplier<Double> getruntime;
    double oldTime = 0;
    public Pose2D pos = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,0); //current Position (x & y in mm, and heading in degrees) of the robot
    public Pose2D vel = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,0); //current Velocity (x & y in mm/sec and heading in degrees/sec)
    private final Queue<Pose2D> pendingVision = new ConcurrentLinkedQueue<>();

    /** Limelight 3A (or any absolute source) calls this; applied via odo.setPosition(). */
    public void correctWithVision(Pose2D fieldAlignedPinpointPose) {
        if (fieldAlignedPinpointPose != null) pendingVision.offer(fieldAlignedPinpointPose);
    }

    /** Forward vision into the FieldTracker Kalman path (inches/deg + std dev). */
    public void correctTrackerWithVision(FieldTracker tracker, VisionMeasurement m) {
        if (tracker != null && m != null) tracker.addVisionMeasurement(m);
    }
    public GoBildaPinpoint(LinearOpMode opMode)
    {
        gamepad1 = opMode.gamepad1;
        getruntime = opMode::getRuntime;
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();
        telemetry.addData("PinPointStatus", "Initialized");
        telemetry.update();
    }
    @Override
    public void Periodic() {
        telemetry.addData("PinPointStatus", "Running");
        // NOTE: getXOffset/getYOffset/getDeviceVersion each do their own I2C read —
        // keep them out of the hot loop. Read once below only when needed for debug.
        /*
        Request an update from the Pinpoint odometry computer. This checks almost all outputs
        from the device in a single I2C read.
         */
        odo.update();

        /*
        Optionally, you can update only the heading of the device. This takes less time to read, but will not
        pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
         */
        //odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);


        if (gamepad1.a){
            odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }

        if (gamepad1.b){
            odo.recalibrateIMU(); //recalibrates the IMU without resetting position
        }

        // Apply any pending absolute corrections (Limelight 3A path) BEFORE reading pos.
        Pose2D correction;
        while ((correction = pendingVision.poll()) != null) {
            odo.setPosition(correction);
        }

        /*
        This code prints the loop frequency of the REV Control Hub. This frequency is effected
        by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
        of time each cycle takes and finds the frequency (number of updates per second) from
        that cycle time.
         */
        double newTime = getruntime.get();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;


        /*
        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
         */
        pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        /*
        gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
         */
        vel = odo.getVelocity();
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        /*
        Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
        READY: the device is working as normal
        CALIBRATING: the device is calibrating and outputs are put on hold
        NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
        FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
        FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
        FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
        */
        telemetry.addData("Status", odo.getDeviceStatus());

        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.update();
    }
}

