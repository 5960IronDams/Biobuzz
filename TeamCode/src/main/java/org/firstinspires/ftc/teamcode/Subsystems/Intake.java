package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IronConstants;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Velocity-PID intake subsystem.
 *
 * <p>Hardware + its own single-subsystem commands - gamepad bindings live in
 * {@code KeyBindings} (right trigger press/release schedules
 * {@code holdCmd()} / {@code stopCmd()}).
 * Panels-tunable (Panels -> Intake): set INTAKE_RPM plus the
 * RUN_USING_ENCODER PIDF gains live.
 *
 * <p>Construct once in the OpMode (auto-registers with SubsystemBase):
 * <pre>
 * intake = new Intake(this);
 * </pre>
 */
@Configurable
public class Intake extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // ---- Panels tuning ----
    public static double INTAKE_RPM = 100;
    public static double TICKS_PER_REV = 537.7; // goBILDA 5203 312rpm = 537.7; adjust to your motor
    public static double kP = 20;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 12;
    public static boolean MOTOR_REVERSED = true;
    public static double TRIGGER_THRESHOLD = 0.2;


    private final DcMotorEx motor;

    private double lastP = Double.NaN;
    private double lastI = Double.NaN;
    private double lastD = Double.NaN;
    private double lastF = Double.NaN;
    private boolean lastReversed = false;

    public Intake(OpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, IronConstants.IntakeMotorName);

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

    // ---- Commands (single-subsystem; multi-subsystem sequences live in CommandFactory) ----

    /**
     * Fire-and-forget: spin the intake at INTAKE_RPM once (the motor keeps
     * spinning until stopCmd). For autos / sequencing - pairs with
     * {@link #stopCmd} in a sequence. NOT Panels-live-tunable.
     */
    public Command runCmd() {
        return runOnce(this::runIntake);
    }

    /**
     * Continuous hold: re-asserts INTAKE_RPM every scheduler loop, so live
     * Panels retunes apply immediately while held. For TeleOp trigger
     * bindings - releases should bind {@link #stopCmd}.
     */
    public Command holdCmd() {
        return run(this::runIntake);
    }

    /** PID-brake the intake to zero. */
    public Command stopCmd() {
        return runOnce(this::stop);
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
    public void periodic() {
        applyPids(false);

        // NOTE: addData only, no update() here. Panels TelemetryManager sends AND
        // clears its line buffer on every update(), so >1 update per loop sends
        // partial frames (flicker) and drops lines. RobotMain.flushTelemetry()
        // does the single end-of-loop flush.
        telemetry.addData("Intake target RPM", INTAKE_RPM);
        telemetry.addData("Intake actual RPM", "%.1f", getActualRpm());
        telemetry.addData("Intake target tps", "%.1f", rpmToTicksPerSec(INTAKE_RPM));
        telemetry.addData("Intake actual tps", "%.1f", motor.getVelocity());
    }
}
