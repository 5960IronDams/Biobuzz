package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IronConstants;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Velocity-PID flywheel subsystem.
 *
 * <p>Hardware + its own single-subsystem commands ({@code toRpm} /
 * {@code holdTunable} / {@code stopCmd} / {@code coastToIdleCmd}) -
 * gamepad bindings live in {@code KeyBindings}, multi-subsystem
 * compositions live in {@code CommandFactory}.
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
    public static double TARGET_RPM = 2000; //max observed is 4000;
    /** Coast destination + PID hold point after the coast arrives. */
    public static double IDLE_RPM = 0;
    public static double TICKS_PER_REV = IronConstants.gobilda5203TicksPerRev(1.0);
    public static double kP = 20;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 19;
    public static boolean MOTOR_REVERSED = false;
    /** Within this of TARGET counts as "at speed" for sequencing / telemetry. */
    public static double AT_SPEED_TOLERANCE_RPM = 75;
    /** Coast is "arrived" once actual RPM has fallen to IDLE + this. */
    public static double COAST_ARRIVE_TOLERANCE_RPM = 50;

    private enum Mode {
        STOPPED,
        HOLDING,
        FLOATING,
        COASTING_TO_IDLE
    }

    private final DcMotorEx motor;
    private Mode mode = Mode.STOPPED;

    private double lastP = Double.NaN;
    private double lastI = Double.NaN;
    private double lastD = Double.NaN;
    private double lastF = Double.NaN;
    private boolean lastReversed = false;

    /** Last velocity setpoint actually written to hardware (ticks/sec). */
    private double lastAppliedTicks = Double.NaN;
    /** Last mode applied to hardware, so a mode change forces one re-write. */
    private Mode lastAppliedMode = null;
    /** TARGET_RPM at the last setpoint write — detects a live Panels retune. */
    private double lastAppliedRpm = Double.NaN;
    /**
     * The idle RPM latched at the moment coast-to-idle started. The coast
     * arrives at THIS, not at a live {@link #IDLE_RPM} that may have been
     * retuned (possibly to 0) mid-coast — which used to make the coast
     * stall, never arrive, and skip the PID idle hold.
     */
    private double idleLocked = Double.NaN;

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

    /**
     * True while the velocity PID is actively holding a commanded setpoint
     * (HOLDING mode). False in STOPPED/FLOATING/COASTING - so "at speed" checks
     * like {@code isHoldingTarget() && isAtTargetRpm()} can't pass before any
     * shoot RPM has been commanded (STOPPED reads as at-0).
     */
    public boolean isHoldingTarget() {
        return mode == Mode.HOLDING;
    }

    /**
     * Within {@link #AT_SPEED_TOLERANCE_RPM} of the active target (PID modes
     * only). COASTING_TO_IDLE is never "at speed" - it is deliberately falling
     * to {@link #IDLE_RPM} and must not let the shoot gate open.
     */
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
     * Fire-and-forget: the PID hold runs in {@link #periodic}, so the command
     * wrapping this can be instant; sequence on {@link #isAtTargetRpm} if needed.
     */
    public void setTargetRpm(double rpm) {
        TARGET_RPM = rpm;
        engagePidMode();
        mode = Mode.HOLDING;
        applyVelocity();
    }

    /** PID-brake to zero. Cancels any coast. */
    public void stop() {
        engagePidMode();
        mode = Mode.FLOATING;
        applyVelocity();
    }

    // ---- Commands (single-subsystem; multi-subsystem sequences live in CommandFactory) ----

    /**
     * Spin the flywheel to an explicit RPM via velocity PID (fire-and-forget).
     * The PID hold runs in {@code periodic()}, so this finishes instantly -
     * sequence on {@code isAtTargetRpm()} (e.g. {@code .until(...)}) if the
     * next step needs it up to speed first.
     */
    public Command toRpm(double rpm) {
        return runOnce(() -> setTargetRpm(rpm));
    }

    /**
     * (Re)engage the velocity hold at the current TARGET_RPM. Re-asserting is
     * instant: once HOLDING, periodic() notices any live Panels retune of
     * TARGET_RPM and re-writes the setpoint, so no per-loop command is needed.
     */
    public Command holdTunable() {
        return runOnce(() -> setTargetRpm(TARGET_RPM));
    }

    /** PID-brake the flywheel to zero. */
    public Command stopCmd() {
        return runOnce(this::stop);
    }

    /**
     * Cut drive (FLOAT) and free-spin toward idle, then re-engage the normal PID
     * to hold idle on arrival. Finishes when {@code isHoldingIdle()} -
     * i.e. coast arrived AND PID is holding idle - so autos can sequence on it.
     */
    public Command coastToIdleCmd() {
        return new FunctionalCommand(
                this::coastToIdle,   // initialize
                () -> {},            // execute (PID hold runs in periodic())
                (interrupted) -> {}, // end
                this::isHoldingIdle, // isFinished
                this);               // requirement
    }

    /**
     * Cut drive (FLOAT, no PID) and free-spin toward {@link #IDLE_RPM}. When actual
     * RPM falls to idle, re-engages the normal velocity PID to hold it there.
     * If already at/below idle, holds idle via PID immediately (a coast from
     * rest could never arrive). This is also where a live Panels retune of
     * IDLE_RPM lands: it latches whatever {@code IDLE_RPM} reads at trigger
     * release as the hold point.
     */
    public void coastToIdle() {
        if (getActualRpm() <= IDLE_RPM + COAST_ARRIVE_TOLERANCE_RPM) {
            setTargetRpm(IDLE_RPM);
            return;
        }
        // Own the whole coast-arrive latch: the drive now falls to IDLE_RPM.
        idleLocked = IDLE_RPM;
        mode = Mode.COASTING_TO_IDLE;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
    }

    /**
     * Write the velocity setpoint for the current mode. The hub-side velocity
     * PI keeps its own target, so this is written EVERY loop rather than only
     * on change: a single write after a loop-mode change is not enough (a
     * velocity write while the motor sits in RUN_WITHOUT_ENCODER is ignored,
     * which is how the hold at the shoot RPM survived the coast).
     */
    private void applyVelocity() {
        final double ticks;
        if (mode == Mode.HOLDING) {
            ticks = rpmToTicksPerSec(TARGET_RPM);
        } else {
            // COASTING_TO_IDLE, STOPPED and FLOATING all command zero: the coast
            // is free-spinning (RUN_WITHOUT_ENCODER + FLOAT + 0 power, set by
            // coastToIdle), so the wheel falls toward IDLE_RPM instead of being
            // held at TARGET_RPM. FLOATING/STOPPED just spin down from there.
            // (COASTING_TO_IDLE used to be lumped in with HOLDING — the PID
            // kept the shoot RPM up, actual RPM never fell to IDLE_RPM, and
            // the coast-to-idle never arrived. Seen in capture 20261024-232657.)
            ticks = 0.0;
        }
        motor.setVelocity(ticks);
        lastAppliedTicks = ticks;
        lastAppliedRpm = TARGET_RPM;
        lastAppliedMode = mode;
    }

    /**
     * PID-hold mode (RUN_USING_ENCODER, FLOAT on 0 power). A no-op when the
     * motor is already there — setMode is a device write and this runs in the
     * hot path.
     */
    private void engagePidMode() {
        if (motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (motor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        }
        // Live Panels retune of TARGET_RPM: re-write the held setpoint.
        if (mode == Mode.HOLDING && TARGET_RPM != lastAppliedRpm) {
            applyVelocity();
        }
    }

    @Override
    public void periodic() {
        applyPids(false);

        switch (mode) {
            case COASTING_TO_IDLE: {
                double idle = Double.isNaN(idleLocked) ? IDLE_RPM : idleLocked;
                if (getActualRpm() <= idle + COAST_ARRIVE_TOLERANCE_RPM) {
                    // Arrived: latch idle as the target and hold it with the
                    // normal velocity PID until someone commands otherwise.
                    TARGET_RPM = idle;
                    idleLocked = Double.NaN;
                    engagePidMode();
                    mode = Mode.HOLDING;
                }
                // else: stay FLOAT + zero power, keep falling.
                break;
            }
            case STOPPED:
                // Re-engage in case the mode was flipped under us.
                if (motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    engagePidMode();
                    lastAppliedMode = null; // force a fresh setpoint write
                }
                break;
            default:
                break;
        }

        // Write the setpoint (the hub velocity PI is stateful — see applyVelocity).
        applyVelocity();

        // NOTE: addData only, no update() here. Panels TelemetryManager sends AND
        // clears its line buffer on every update(), so >1 update per loop sends
        // partial frames (flicker) and drops lines. RobotMain.flushTelemetry()
        // does the single end-of-loop flush.
        telemetry.addData("Flywheel mode", mode);
        telemetry.addData("Flywheel target RPM", targetRpmText());
        telemetry.addData("Flywheel actual RPM", "%.1f", getActualRpm());
        telemetry.addData("Flywheel at speed", isCoasting() ? "COAST" : isAtTargetRpm());
    }

    /** What the motor is actually being driven to, for the dashboard. */
    private String targetRpmText() {
        switch (mode) {
            case HOLDING:
                return String.format(java.util.Locale.US, "%.0f", TARGET_RPM);
            case COASTING_TO_IDLE:
                return String.format(java.util.Locale.US, "COAST -> %.0f",
                        Double.isNaN(idleLocked) ? IDLE_RPM : idleLocked);
            case STOPPED:
            case FLOATING:
            default:
                return "0";
        }
    }
}
