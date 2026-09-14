package org.firstinspires.ftc.teamcode.irondams.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Robot-Centric Mecanum drivetrain implementation.
 * Directs movement relative to the robot's internal frame (Forward is always where the front of the robot faces).
 * Includes motor-specific scaling for autonomous modes and input slew-rate limiting (ramping).
 */
public class MecanumDriveTrain implements IDriveTrain {
    private double currentHorizontal = 0;
    private double currentVertical = 0;
    private double currentPivot = 0;

    public final FourWheelDriveTrain DRIVE_TRAIN;
    public final boolean IS_AUTO;

    /**
     * Constructs a standard Teleop robot-centric drivetrain.
     *
     * @param driveTrain The baseline four-wheel motor configuration to control.
     */
    public MecanumDriveTrain(FourWheelDriveTrain driveTrain)  {
        DRIVE_TRAIN = driveTrain;
        IS_AUTO = false;
    }

    /**
     * Constructs a robot-centric drivetrain with specialized mode flags.
     *
     * @param driveTrain The baseline four-wheel motor configuration to control.
     * @param isAuto     If true, applies autonomous-specific power trimming to motors for better straight-line tracking.
     */
    public MecanumDriveTrain(FourWheelDriveTrain driveTrain, boolean isAuto)  {
        DRIVE_TRAIN = driveTrain;
        IS_AUTO = isAuto;
    }

    /**
     * Executes robot-centric mecanum wheel-power calculations.
     * Defaults to an estimated fixed loop interval calculation.
     *
     * @param horizontal Desired lateral strafe speed [-1.0, 1.0].
     * @param vertical   Desired forward/reverse speed [-1.0, 1.0].
     * @param pivot      Desired rotational steering speed [-1.0, 1.0].
     */
    @Override
    public void drive(double horizontal, double vertical, double pivot) {
        driveWithTime(horizontal, vertical, pivot, 0.05); // Default to 20Hz timing if time not provided
    }

    /**
     * Executes robot-centric mecanum wheel-power calculations,
     * utilizing a precise deltaTime step to smoothly ramp input logic.
     *
     * @param horizontal   Desired lateral strafe speed [-1.0, 1.0].
     * @param vertical     Desired forward/reverse speed [-1.0, 1.0].
     * @param pivot        Desired rotational steering speed [-1.0, 1.0].
     * @param deltaTimeSec Exact time duration elapsed in seconds since the last loop iteration.
     */
    public void driveWithTime(double horizontal, double vertical, double pivot, double deltaTimeSec) {
        double maxDeltaPerSec = 3.0; // Adjustable ramp speed

        double newHorizontal = Acceleration.rampPower(currentHorizontal, horizontal, maxDeltaPerSec, deltaTimeSec);
        horizontal = newHorizontal;
        currentHorizontal = newHorizontal;

        double newVertical = Acceleration.rampPower(currentVertical, vertical, maxDeltaPerSec, deltaTimeSec);
        vertical = newVertical;
        currentVertical = newVertical;

        double newPivot = Acceleration.rampPower(currentPivot, pivot, maxDeltaPerSec, deltaTimeSec);
        pivot = newPivot;
        currentPivot = newPivot;

        double flp = (pivot + vertical + horizontal);
        double frp = (-pivot + (vertical - horizontal));
        double rlp = (pivot + (vertical - horizontal));
        double rrp = (-pivot + vertical + horizontal);

        if (IS_AUTO) {
            flp *= 0.9333333333;
            frp *= 0.9733333333;
            rrp *= 0.96;
        }

        DRIVE_TRAIN.getLeftBackDrive().setPower(rlp);
        DRIVE_TRAIN.getRightBackDrive().setPower(rrp);
        DRIVE_TRAIN.getLeftFrontDrive().setPower(flp);
        DRIVE_TRAIN.getRightFrontDrive().setPower(frp);
    }

    /**
     * @return A flat array containing the hardware objects for all four drive motors.
     */
    public DcMotorEx[] getMotors() {
        return new DcMotorEx[] {
                DRIVE_TRAIN.getLeftBackDrive(),
                DRIVE_TRAIN.getRightBackDrive(),
                DRIVE_TRAIN.getLeftFrontDrive(),
                DRIVE_TRAIN.getRightFrontDrive()
        };
    }
}