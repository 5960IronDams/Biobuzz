package org.firstinspires.ftc.teamcode.killerwatts;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase;

/**
 * Velocity-PID flywheel subsystem.
 *
 * <p>Hardware only - gamepad bindings live in {@code KeyBindings}, commands live in
 * {@code CommandFactory} ({@code SetFlywheelRpm} / {@code StopFlywheel} /
 * {@code CoastFlywheelToIdle}).
 * Panels-tunable (Panels -&gt; Flywheel): TARGET_RPM, IDLE_RPM plus the
 * RUN_USING_ENCODER PIDF gains live.
 *
 * <p>Modes:
 * <ul>
 *   <li>HOLDING - RUN_USING_ENCODER velocity PID at {@link #TARGET_RPM}.</li>
 *   <li>STOPPED - RUN_USING_ENCODER velocity PID at 0.</li>
 *   <li>COASTING_TO_IDLE - RUN_WITHOUT_ENCODER + FLOAT + zero power, free-spinning
 *       down. When actual RPM falls to {@link #IDLE_RPM}, re-engages the normal
 *       PID to hold idle (reduces re-spool time vs a full stop).</li>
 * </ul>
 *
 * <p>Construct once in the OpMode (auto-registers with SubsystemBase):
 * <pre>
 * flywheel = new Flywheel(this);
 * </pre>
 */
@Configurable
public class Flywheel extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    /** Active PID setpoint. Written by {@link #setTargetRpm} / coast-arrive, tunable live. */
    public static double TARGET_RPM = 2500;
    /** Coast destination + PID hold point after the coast arrives. */
    public static double IDLE_RPM = 1200;
    public static double TICKS_PER_REV = 537.7; // goBILDA 5203 312rpm = 537.7; adjust to your motor
    public static double kP = 20;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 12;
    public static boolean MOTOR_REVERSED = false;
    /** Within this of TARGET counts as "at speed" for sequencing / telemetry. */
    public static double AT_SPEED_TOLERANCE_RPM = 75;
    /** Coast is "arrived" once actual RPM has fallen to IDLE + this. */
    public static double COAST_ARRIVE_TOLERANCE_RPM = 50;

    private enum Mode {
        STOPPED,
        HOLDING,
        COASTING_TO_IDLE
    }

    private final DcMotorEx motor;
    private Mode mode = Mode.STOPPED;

    private double lastP = Double.NaN;
    private double lastI = Double.NaN;
    private double lastD = Double.NaN;
    private double lastF = Double.NaN;
    private boolean lastReversed = false;

    public Flywheel(OpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, IronConstants.FlywheelMotorName);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        engagePidMode();
        motor.setVelocity(0);
        applyPids(true);
        mode = Mode.STOPPED;
    }

    /** RPM -> encoder ticks/sec for DcMotorEx.setVelocity(). */
    public static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    /** Encoder ticks/sec -> RPM for telemetry / arrival checks. */
    public static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    public double getTargetRpm() {
        return TARGET_RPM;
    }

    public double getActualRpm() {
        return ticksPerSecToRpm(motor.getVelocity());
    }

    public boolean isCoasting() {
        return mode == Mode.COASTING_TO_IDLE;
    }

    /** Within {@link #AT_SPEED_TOLERANCE_RPM} of the active target (PID modes only). */
    public boolean isAtTargetRpm() {
        if (mode == Mode.COASTING_TO_IDLE) return false;
        double goal = mode == Mode.STOPPED ? 0 : TARGET_RPM;
        return Math.abs(getActualRpm() - goal) <= AT_SPEED_TOLERANCE_RPM;
    }

    /**
     * True once the coast has arrived AND the normal PID is holding idle.
     * Use as the {@code done} condition for the coast command.
     */
    public boolean isHoldingIdle() {
        return mode == Mode.HOLDING
                && TARGET_RPM == IDLE_RPM
                && Math.abs(getActualRpm() - IDLE_RPM)
                    <= Math.max(AT_SPEED_TOLERANCE_RPM, COAST_ARRIVE_TOLERANCE_RPM);
    }

    /**
     * Spin (or re-spin) to an explicit RPM via velocity PID. Cancels any coast.
     * Fire-and-forget: the PID hold runs in {@link #Periodic}, so the command
     * wrapping this can be instant; sequence on {@link #isAtTargetRpm} if needed.
     */
    public void setTargetRpm(double rpm) {
        TARGET_RPM = rpm;
        engagePidMode();
        mode = Mode.HOLDING;
        motor.setVelocity(rpmToTicksPerSec(TARGET_RPM));
    }

    /** PID-brake to zero. Cancels any coast. */
    public void stop() {
        engagePidMode();
        mode = Mode.STOPPED;
        motor.setVelocity(0);
    }

    /**
     * Cut drive (FLOAT, no PID) and free-spin toward {@link #IDLE_RPM}. When actual
     * RPM falls to idle, re-engages the normal velocity PID to hold it there.
     * If already at/below idle, holds idle via PID immediately (a coast from
     * rest could never arrive).
     */
    public void coastToIdle() {
        if (getActualRpm() <= IDLE_RPM + COAST_ARRIVE_TOLERANCE_RPM) {
            setTargetRpm(IDLE_RPM);
            return;
        }
        mode = Mode.COASTING_TO_IDLE;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
    }

    /** Back to RUN_USING_ENCODER + BRAKE for any PID-driven state. */
    private void engagePidMode() {
        if (motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        switch (mode) {
            case COASTING_TO_IDLE:
                if (getActualRpm() <= IDLE_RPM + COAST_ARRIVE_TOLERANCE_RPM) {
                    // Arrived: latch idle as the target and hold it with the normal PID.
                    TARGET_RPM = IDLE_RPM;
                    engagePidMode();
                    mode = Mode.HOLDING;
                    motor.setVelocity(rpmToTicksPerSec(TARGET_RPM));
                }
                // else: stay FLOAT + zero power, keep falling.
                break;
            case HOLDING:
                motor.setVelocity(rpmToTicksPerSec(TARGET_RPM));
                break;
            case STOPPED:
                // Re-assert zero in case the mode was flipped under us; cheap and safe.
                if (motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    engagePidMode();
                }
                motor.setVelocity(0);
                break;
        }

        // NOTE: addData only, no update() here. Panels TelemetryManager sends AND
        // clears its line buffer on every update(), so >1 update per loop sends
        // partial frames (flicker) and drops lines. RobotMain.flushTelemetry()
        // does the single end-of-loop flush.
        telemetry.addData("Flywheel mode", mode);
        telemetry.addData("Flywheel target RPM", mode == Mode.STOPPED ? 0 : TARGET_RPM);
        telemetry.addData("Flywheel actual RPM", "%.1f", getActualRpm());
        telemetry.addData("Flywheel at speed", isCoasting() ? "COAST" : isAtTargetRpm());
    }
}
