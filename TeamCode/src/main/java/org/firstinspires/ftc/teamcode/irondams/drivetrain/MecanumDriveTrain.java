package org.firstinspires.ftc.teamcode.irondams.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Robot-Centric Mecanum drivetrain implementation.
 * Directs movement relative to the robot's internal frame (Forward is always where the front of the robot faces).
 * Includes motor-specific scaling for autonomous modes and input slew-rate limiting (ramping).
 *
 * <p>Input chain: raw stick -> {@link DriveConstants#conditionStick} (deadband/expo) ->
 * per-axis gain (STRAFE/FWD/TURN) -> slew ramp -> mecanum mix -> normalize ->
 * {@link FourWheelDriveTrain#setWheelPowers} (kS boost + per-wheel trims,
 * or velocity-PID when enabled).
 *
 * <p>No per-axis clamp: the mix normalize below preserves the boosted strafe/forward
 * ratio and caps magnitude without distorting diagonal direction.
 * (Robot-centric stick frame == robot frame, so gains apply directly.)
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
        // Condition FIRST (deadband/expo on raw stick), then per-axis gain, then
        // ramp the shaped value. Stick frame == robot frame here, so gains apply directly.
        horizontal = DriveConstants.conditionStick(horizontal) * DriveConstants.STRAFE_GAIN;
        vertical = DriveConstants.conditionStick(vertical) * DriveConstants.FWD_GAIN;
        pivot = DriveConstants.conditionStick(pivot) * DriveConstants.TURN_GAIN;

        double rate = DriveConstants.RAMP_RATE;

        double newHorizontal = Acceleration.rampPower(currentHorizontal, horizontal, rate, deltaTimeSec);
        horizontal = newHorizontal;
        currentHorizontal = newHorizontal;

        double newVertical = Acceleration.rampPower(currentVertical, vertical, rate, deltaTimeSec);
        vertical = newVertical;
        currentVertical = newVertical;

        double newPivot = Acceleration.rampPower(currentPivot, pivot, rate, deltaTimeSec);
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

        // Normalize before the kS boost so direction proportions are preserved.
        double max = Math.max(Math.max(Math.abs(flp), Math.abs(frp)),
                Math.max(Math.abs(rlp), Math.abs(rrp)));
        if (max > 1.0) {
            flp /= max;
            frp /= max;
            rlp /= max;
            rrp /= max;
        }

        DRIVE_TRAIN.setWheelPowers(flp, frp, rlp, rrp);
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
