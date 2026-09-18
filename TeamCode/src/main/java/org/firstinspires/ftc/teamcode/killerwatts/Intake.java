package org.firstinspires.ftc.teamcode.killerwatts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase;

/**
 * Velocity-PID intake subsystem, same pattern as {@link ServoMove}.
 *
 * <p>Dashboard-tunable (FTC Dashboard -> Intake): set INTAKE_RPM plus the
 * RUN_USING_ENCODER PIDF gains live. Hold gamepad1 right trigger past
 * TRIGGER_THRESHOLD to spin at INTAKE_RPM, release to stop.
 *
 * <p>Construct once in the OpMode (auto-registers with SubsystemBase):
 * <pre>
 * intake = new Intake(this);
 * </pre>
 */
@Config
public class Intake extends SubsystemBase {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    // ---- Dashboard tuning ----
    public static double INTAKE_RPM = 100;
    public static double TICKS_PER_REV = 537.7; // goBILDA 5203 312rpm = 537.7; adjust to your motor
    public static double kP = 20;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 12;
    public static boolean MOTOR_REVERSED = true;
    public static double TRIGGER_THRESHOLD = 0.2;
    public static String MOTOR_NAME = "intake";

    private final DcMotorEx motor;
    private final Gamepad gamepad1;

    private double lastP = Double.NaN;
    private double lastI = Double.NaN;
    private double lastD = Double.NaN;
    private double lastF = Double.NaN;
    private boolean lastReversed = false;

    public Intake(OpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        gamepad1 = opMode.gamepad1;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        applyPids(true);
    }

    /** RPM -> encoder ticks/sec for DcMotorEx.setVelocity(). */
    public static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    /** Encoder ticks/sec -> RPM for telemetry. */
    public static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    public double getTargetRpm() {
        return INTAKE_RPM;
    }

    public double getActualRpm() {
        return ticksPerSecToRpm(motor.getVelocity());
    }

    public void runIntake() {
        motor.setVelocity(rpmToTicksPerSec(INTAKE_RPM));
    }

    public void stop() {
        motor.setVelocity(0);
    }

    private void applyPids(boolean force) {
        if (force || kP != lastP || kI != lastI || kD != lastD || kF != lastF) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kP, kI, kD, kF));
            lastP = kP;
            lastI = kI;
            lastD = kD;
            lastF = kF;
        }
        if (force || MOTOR_REVERSED != lastReversed) {
            motor.setDirection(MOTOR_REVERSED
                    ? DcMotor.Direction.REVERSE
                    : DcMotor.Direction.FORWARD);
            lastReversed = MOTOR_REVERSED;
        }
    }

    @Override
    public void Periodic() {
        applyPids(false);

        if (gamepad1.rightTriggerWasPressed()) {  //gamepad1.right_trigger > TRIGGER_THRESHOLD;
            // Re-assert every loop so dashboard RPM edits apply live
            // and nothing else can starve the velocity.
            motor.setVelocity(rpmToTicksPerSec(INTAKE_RPM));
        } else if (gamepad1.rightTriggerWasReleased()){
            stop();
        }

        telemetry.addData("Intake target RPM", INTAKE_RPM);
        telemetry.addData("Intake actual RPM", "%.1f", getActualRpm());
        telemetry.addData("Intake target tps", "%.1f", rpmToTicksPerSec(INTAKE_RPM));
        telemetry.addData("Intake actual tps", "%.1f", motor.getVelocity());
        telemetry.update();
    }
}
