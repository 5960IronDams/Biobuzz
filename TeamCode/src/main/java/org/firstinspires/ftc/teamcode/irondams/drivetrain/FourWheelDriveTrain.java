package org.firstinspires.ftc.teamcode.irondams.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Represents a standard four-wheel robot drivetrain configuration.
 * This class handles hardware initialization for all four drive motors,
 * setting their default directions and zero-power behaviors.
 */
public final class FourWheelDriveTrain {
    private final DcMotorEx leftFrontDrive;
    private final DcMotorEx leftBackDrive;
    private final DcMotorEx rightFrontDrive;
    private final DcMotorEx rightBackDrive;

    /**
     * @return The hardware instance for the front-left motor.
     */
    public DcMotorEx getLeftFrontDrive() {
        return leftFrontDrive;
    }

    /**
     * @return The hardware instance for the back-left motor.
     */
    public DcMotorEx getLeftBackDrive() {
        return leftBackDrive;
    }

    /**
     * @return The hardware instance for the front-right motor.
     */
    public DcMotorEx getRightFrontDrive() {
        return rightFrontDrive;
    }

    /**
     * @return The hardware instance for the back-right motor.
     */
    public DcMotorEx getRightBackDrive() {
        return rightBackDrive;
    }


    /**
     * Initializes the drivetrain by fetching motors from the hardware map.
     * Configures the right-side motors to REVERSE direction and all motors to BRAKE mode.
     *
     * @param hardwareMap The OpMode's hardware map used to access the motor ports.
     */
    public FourWheelDriveTrain (HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}