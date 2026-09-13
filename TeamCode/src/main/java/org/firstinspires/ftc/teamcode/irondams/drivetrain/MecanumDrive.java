package org.firstinspires.ftc.teamcode.irondams.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Top-level manager class for a Mecanum drivetrain subsystem.
 * Coordinates between Robot-Centric (roboView) and Field-Centric (fieldPer) navigation,
 * handles user toggle changes, tracks loop timing metrics, and forwards input vectors 
 * through acceleration rate limiters.
 */
public class MecanumDrive {
    private final MecanumDriveTrain roboView;
    private final GyroMecanumDriveTrain fieldPer;

    private IDriveTrain activeDrive;
    private boolean _isFieldPer = false;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    /**
     * Instantiates the Mecanum subsystem components, initializes drive motors, 
     * registers the internal gyro, and defaults control states to Robot-Centric perspective.
     *
     * @param opMode The active LinearOpMode context providing hardware map access.
     */
    public MecanumDrive(LinearOpMode opMode) {
        FourWheelDriveTrain dt = new FourWheelDriveTrain(opMode.hardwareMap);
        roboView = new MecanumDriveTrain(dt);
        fieldPer = new GyroMecanumDriveTrain(opMode, dt);
        activeDrive = _isFieldPer ? fieldPer : roboView;
        timer.reset();
        lastTime = timer.seconds();
    }

    /**
     * Resets the underlying IMU gyro reference angle. 
     * The robot's current heading direction will now establish the new "Forward" baseline for Field-Centric drive.
     */
    public void resetFieldView() {
        fieldPer.reset();
    }

    /**
     * Toggles the active drive perspective state back and forth between Robot-Centric and Field-Centric orientations.
     *
     * @return True if the system changed to Field-Centric mode, false if changed to Robot-Centric mode.
     */
    public boolean switchDrive() {
        _isFieldPer = !_isFieldPer;
        activeDrive = _isFieldPer ? fieldPer : roboView;
        return _isFieldPer;
    }

    /**
     * Directs joystick motion vectors down to the active drivetrain interface.
     * Injects a loop duration timestamp to scale acceleration ramps independently of variation in cycle rates.
     *
     * @param x    The horizontal strafe axis velocity command [-1.0, 1.0].
     * @param y    The vertical forward/reverse axis velocity command [-1.0, 1.0].
     * @param turn The rotational steering axis velocity command [-1.0, 1.0].
     */
    public void drive(double x, double y, double turn) {
        double currentTime = timer.seconds();
        double deltaTimeSec = currentTime - lastTime;
        if (deltaTimeSec <= 0) deltaTimeSec = 0.001; // Avoid divide-by-zero or zero step
        lastTime = currentTime;

        // Pass delta time into the drivetrain systems
        if (activeDrive instanceof MecanumDriveTrain) {
            ((MecanumDriveTrain) activeDrive).driveWithTime(x, -y, turn, deltaTimeSec);
        } else if (activeDrive instanceof GyroMecanumDriveTrain) {
            ((GyroMecanumDriveTrain) activeDrive).driveWithTime(x, -y, turn, deltaTimeSec);
        } else {
            activeDrive.drive(x, -y, turn);
        }
    }
}