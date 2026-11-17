package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;

import java.util.Locale;

/**
 * Velocity-PID flywheel subsystem.
 *
 * <p>Modes:
 * <ul>
 *   <li>HOLDING - RUN_USING_ENCODER velocity PID at {@link #TARGET_RPM}.</li>
 *   <li>STOPPED - RUN_USING_ENCODER velocity PID at 0 (zero-power FLOAT).</li>
 *   <li>COASTING_TO_IDLE - RUN_WITHOUT_ENCODER + FLOAT + zero power until
 *       actual RPM drops to {@link #IDLE_RPM}, then holds idle (or stops if
 *       {@link #IDLE_RPM} &lt;= 0).</li>
 * </ul>
 *
 * <p>Panels-tunable (Panels -&gt; Flywheel): {@code TARGET_RPM}, {@code IDLE_RPM},
 * PIDF gains, tolerance, and {@code RPM_SMOOTH_TAU_S} apply live.
 */
@Configurable
public class Flywheel extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    /** Active PID setpoint (RPM). Updated by {@link #setTargetRpm(double)} or Panels. */
    public static double TARGET_RPM = 2000;
    /** Coast destination RPM after shooting ends (<= 0 coasts all the way to STOPPED). */
    public static double IDLE_RPM = 0;
    /** Encoder ticks per flywheel shaft revolution (goBILDA 5203 motor shaft = 28). */
    public static double TICKS_PER_REV = IronConstants.gobilda5203TicksPerRev(1.0);

    /** Velocity PIDF gains for REV Hub RUN_USING_ENCODER control. */
    public static double kP = 200.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 23.0;
    /** Set true if positive velocity spins the flywheel backward. */
    public static boolean MOTOR_REVERSED = false;

    /** Tolerance (RPM) within which {@link #isAtTargetRpm()} reports true. */
    public static double AT_SPEED_TOLERANCE_RPM = 85.0;
    /** Coast is considered arrived once actual RPM drops to IDLE_RPM + this margin. */
    public static double COAST_ARRIVE_TOLERANCE_RPM = 50.0;
    /**
     * Low-pass filter time constant (seconds) for {@link #getSmoothedRpm()}.
     * Smooths out 42.9-RPM single-tick encoder steps; larger = smoother, 0 = raw.
     */
    public static double RPM_SMOOTH_TAU_S = 0.06;

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

    private double lastAppliedTicks = Double.NaN;
    private Mode lastAppliedMode = null;
    private DcMotor.RunMode cachedRunMode;
    private DcMotor.ZeroPowerBehavior cachedZeroBehavior = null;
    private double idleLocked = Double.NaN;

    private double smoothedRpm = Double.NaN;
    private long lastPeriodicNs = 0L;

    public Flywheel(OpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, IronConstants.FlywheelMotorName);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cachedRunMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        engagePidMode();
        applyPids(true);
        motor.setVelocity(0);
        lastAppliedTicks = 0.0;
        lastAppliedMode = Mode.STOPPED;
        mode = Mode.STOPPED;
    }

    /** Converts flywheel RPM to encoder ticks/sec for {@code DcMotorEx.setVelocity()}. */
    public static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    /** Converts encoder ticks/sec from {@code DcMotorEx.getVelocity()} to RPM. */
    public static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    public double getTargetRpm() {
        return TARGET_RPM;
    }

    /** Raw instantaneous RPM from the motor encoder. */
    public double getActualRpm() {
        return ticksPerSecToRpm(motor.getVelocity());
    }

    /** Low-pass filtered RPM for steady readiness checks and telemetry. */
    public double getSmoothedRpm() {
        return Double.isNaN(smoothedRpm) ? getActualRpm() : smoothedRpm;
    }

    public boolean isCoasting() {
        return mode == Mode.COASTING_TO_IDLE;
    }

    /** True while the velocity PID is actively holding a commanded shoot RPM. */
    public boolean isHoldingTarget() {
        return mode == Mode.HOLDING;
    }

    /**
     * True when the smoothed flywheel speed is within {@link #AT_SPEED_TOLERANCE_RPM}
     * of the active target (never true while coasting down to idle).
     */
    public boolean isAtTargetRpm() {
        if (mode == Mode.COASTING_TO_IDLE) {
            return false;
        }
        double goal = mode == Mode.STOPPED ? 0.0 : TARGET_RPM;
        return Math.abs(getSmoothedRpm() - goal) <= AT_SPEED_TOLERANCE_RPM;
    }

    /** True once a coast-to-idle has arrived and the flywheel is at idle. */
    public boolean isHoldingIdle() {
        if (IDLE_RPM <= 0.0) {
            return mode == Mode.STOPPED;
        }
        return mode == Mode.HOLDING
                && TARGET_RPM == IDLE_RPM
                && Math.abs(getSmoothedRpm() - IDLE_RPM)
                    <= Math.max(AT_SPEED_TOLERANCE_RPM, COAST_ARRIVE_TOLERANCE_RPM);
    }

    /** Spin (or adjust) to an explicit RPM using velocity PID. Cancels any coast. */
    public void setTargetRpm(double rpm) {
        TARGET_RPM = rpm;
        engagePidMode();
        mode = Mode.HOLDING;
        applyVelocity();
    }

    /** Stop driving the flywheel (sets velocity target to 0 with FLOAT zero-power behavior). */
    public void stop() {
        engagePidMode();
        mode = Mode.STOPPED;
        applyVelocity();
    }

    /**
     * Cut drive power (FLOAT) and coast down toward {@link #IDLE_RPM}.
     * Once at idle, holds {@link #IDLE_RPM} via PID (or enters STOPPED if IDLE_RPM <= 0).
     */
    public void coastToIdle() {
        if (getActualRpm() <= IDLE_RPM + COAST_ARRIVE_TOLERANCE_RPM) {
            if (IDLE_RPM <= 0.0) {
                stop();
            } else {
                setTargetRpm(IDLE_RPM);
            }
            return;
        }
        idleLocked = IDLE_RPM;
        mode = Mode.COASTING_TO_IDLE;
        if (cachedRunMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cachedRunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        }
        if (cachedZeroBehavior != DcMotor.ZeroPowerBehavior.FLOAT) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            cachedZeroBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        }
        motor.setPower(0);
        lastAppliedTicks = Double.NaN;
        lastAppliedMode = Mode.COASTING_TO_IDLE;
    }

    // ---- Commands (single-subsystem; multi-subsystem sequences live in CommandFactory) ----

    /** Fire-and-forget command to spin the flywheel to {@code rpm}. */
    public Command toRpm(double rpm) {
        return runOnce(() -> setTargetRpm(rpm));
    }

    /** Stop the flywheel. */
    public Command stopCmd() {
        return runOnce(this::stop);
    }

    /** Coast down to {@link #IDLE_RPM} and finish once idle is reached. */
    public Command coastToIdleCmd() {
        return new FunctionalCommand(
                this::coastToIdle,
                () -> {},
                (interrupted) -> {},
                this::isHoldingIdle,
                this);
    }

    /** Adjust feedforward gain {@link #kF} by {@code delta}, clamped to [5, 60]. */
    public void adjustKf(double delta) {
        kF = Math.round(Math.max(5.0, Math.min(60.0, kF + delta)) * 100.0) / 100.0;
        applyPids(true);
    }

    /** Adjust proportional gain {@link #kP} by {@code delta}, clamped to [5, 120]. */
    public void adjustKp(double delta) {
        kP = Math.round(Math.max(5.0, Math.min(120.0, kP + delta)) * 10.0) / 10.0;
        applyPids(true);
    }

    /** Toggle between spinning at {@link #TARGET_RPM} (or 2000 if <= 500) and coasting to idle. */
    public void toggleTestSpin() {
        if (isHoldingTarget()) {
            coastToIdle();
        } else {
            setTargetRpm(TARGET_RPM > 500.0 ? TARGET_RPM : 2000.0);
        }
    }

    /** Compact status summary for Driver Station telemetry. */
    public String getStatusLine() {
        double act = getSmoothedRpm();
        double tgt = mode == Mode.HOLDING ? TARGET_RPM : (mode == Mode.COASTING_TO_IDLE ? IDLE_RPM : 0.0);
        double err = act - tgt;
        boolean atSpeed = isAtTargetRpm();
        return String.format(Locale.US, "%.0f/%.0f (err %+.0f) | kF=%.2f kP=%.1f [%s]",
                act, tgt, err, kF, kP, atSpeed ? "OK" : (mode == Mode.HOLDING ? "WAIT" : mode.name()));
    }

    @Override
    public void periodic() {
        long nowNs = System.nanoTime();
        double dt = lastPeriodicNs == 0L ? 0.02
                : Math.min(Math.max((nowNs - lastPeriodicNs) / 1.0e9, 0.0), 0.25);
        lastPeriodicNs = nowNs;

        double rawRpm = getActualRpm();
        if (Double.isNaN(smoothedRpm) || RPM_SMOOTH_TAU_S <= 0.0) {
            smoothedRpm = rawRpm;
        } else {
            double alpha = dt / (RPM_SMOOTH_TAU_S + dt);
            smoothedRpm += alpha * (rawRpm - smoothedRpm);
        }

        applyPids(false);

        if (mode == Mode.COASTING_TO_IDLE) {
            double idle = Double.isNaN(idleLocked) ? IDLE_RPM : idleLocked;
            if (rawRpm <= idle + COAST_ARRIVE_TOLERANCE_RPM) {
                idleLocked = Double.NaN;
                engagePidMode();
                if (idle <= 0.0) {
                    mode = Mode.STOPPED;
                } else {
                    TARGET_RPM = idle;
                    mode = Mode.HOLDING;
                }
            }
        } else if (mode == Mode.STOPPED) {
            engagePidMode();
        }

        applyVelocity();

        if (RobotMain.DASHBOARD) {
            telemetry.addData("Flywheel mode", mode);
            telemetry.addData("Flywheel target RPM", targetRpmText());
            telemetry.addData("Flywheel actual RPM", "%.1f", rawRpm);
            telemetry.addData("Flywheel smoothed RPM", "%.1f", smoothedRpm);
            telemetry.addData("Flywheel at speed", isCoasting() ? "COAST" : isAtTargetRpm());
            telemetry.addData("Flywheel PIDF", "kF=%.2f kP=%.1f", kF, kP);
        }
    }

    private void applyVelocity() {
        if (mode == Mode.COASTING_TO_IDLE) {
            return;
        }
        if (mode == Mode.HOLDING) {
            double ticks = rpmToTicksPerSec(TARGET_RPM);
            if (lastAppliedMode != mode || Math.abs(ticks - lastAppliedTicks) > 1e-6) {
                motor.setVelocity(ticks);
                lastAppliedTicks = ticks;
                lastAppliedMode = mode;
            }
            return;
        }
        if (lastAppliedMode != mode || lastAppliedTicks != 0.0) {
            motor.setVelocity(0.0);
            lastAppliedTicks = 0.0;
            lastAppliedMode = mode;
        }
    }

    private void engagePidMode() {
        if (cachedRunMode != DcMotor.RunMode.RUN_USING_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            cachedRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
        }
        if (cachedZeroBehavior != DcMotor.ZeroPowerBehavior.FLOAT) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            cachedZeroBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        }
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
            lastAppliedTicks = Double.NaN;
        }
    }

    private String targetRpmText() {
        switch (mode) {
            case HOLDING:
                return String.format(Locale.US, "%.0f", TARGET_RPM);
            case COASTING_TO_IDLE:
                return String.format(Locale.US, "COAST -> %.0f",
                        Double.isNaN(idleLocked) ? IDLE_RPM : idleLocked);
            case STOPPED:
            default:
                return "0";
        }
    }
}
