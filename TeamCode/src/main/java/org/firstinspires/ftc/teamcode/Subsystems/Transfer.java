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
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Velocity-PID transfer subsystem (rolls balls from the intake up into the
 * flywheel).
 *
 * <p>Hardware + its own single-subsystem commands - gamepad bindings live in
 * {@code KeyBindings} (left bumper hold = forward, right bumper hold = reverse;
 * either release schedules {@code stopCmd()}). Multi-subsystem compositions
 * (intake + transfer together) live in {@code CommandFactory}.
 * Panels-tunable (Panels -&gt; Transfer): set TRANSFER_RPM plus the
 * RUN_USING_ENCODER PIDF gains live.
 *
 * <p>Construct once in the OpMode (auto-registers with SubsystemBase):
 * <pre>
 * transfer = new Transfer(this);
 * </pre>
 */
@Configurable
public class Transfer extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // ---- Panels tuning ----
    public static double TRANSFER_RPM = 500.0; //max measured empty RL = 1050rpm;
    /**
     * goBILDA 5203 Yellow Jacket geared 5.2:1 = 28 PPR x 5.2 = 145.6
     * (computed by {@link IronConstants#gobilda5203TicksPerRev(double)} - the
     * same motor + ratio as the intake).
     */
    public static double TICKS_PER_REV = IronConstants.gobilda5203TicksPerRev(5.2);
    public static double kP = 20.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 12.5;
    /** True = motor wiring is flipped from the code's forward; swap if forward/reverse run the wrong way. */
    public static boolean MOTOR_REVERSED = false;//forward = sends items to turret.

    private final DcMotorEx motor;

    private double lastP = Double.NaN;
    private double lastI = Double.NaN;
    private double lastD = Double.NaN;
    private double lastF = Double.NaN;
    private boolean lastReversed = false;

    public Transfer(OpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, IronConstants.TransferMotorName);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        applyPids(true);
    }

    /** RPM -> encoder ticks/sec for DcMotorEx.setVelocity(). */
    public static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    /** Encoder ticks/sec -> RPM for telemetry (reads negative while reversing). */
    public static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    public double getTargetRpm() {
        return TRANSFER_RPM;
    }

    /** Signed actual RPM: positive running forward, negative in reverse. */
    public double getActualRpm() {
        return ticksPerSecToRpm(motor.getVelocity());
    }

    /** Spin FORWARD (toward the flywheel) at TRANSFER_RPM via velocity PID. */
    public void runForward() {
        motor.setVelocity(rpmToTicksPerSec(TRANSFER_RPM));
    }

    /** Spin REVERSE (back toward the intake / jam-clear) at TRANSFER_RPM via velocity PID. */
    public void runReverse() {
        motor.setVelocity(-rpmToTicksPerSec(TRANSFER_RPM));
    }

    public void stop() {
        motor.setVelocity(0);
    }

    // ---- Commands (single-subsystem; multi-subsystem sequences live in CommandFactory) ----

    /**
     * Fire-and-forget: run the transfer FORWARD at TRANSFER_RPM once (the motor
     * keeps spinning until stopCmd). For autos / sequencing - pairs with
     * {@link #stopCmd} in a sequence. NOT Panels-live-tunable.
     */
    public Command runCmd() {
        return runOnce(this::runForward);
    }

    /**
     * Continuous hold: re-asserts TRANSFER_RPM forward every scheduler loop, so
     * live Panels retunes apply immediately while held. For TeleOp bindings -
     * releases should bind {@link #stopCmd}.
     */
    public Command holdCmd() {
        return run(this::runForward);
    }

    /**
     * Fire-and-forget: run the transfer REVERSE at TRANSFER_RPM once (keeps
     * spinning until stopCmd). For autos / sequencing - pairs with
     * {@link #stopCmd} in a sequence. NOT Panels-live-tunable.
     */
    public Command reverseCmd() {
        return runOnce(this::runReverse);
    }

    /**
     * Continuous REVERSE hold: re-asserts -TRANSFER_RPM every scheduler loop,
     * so live Panels retunes apply immediately while held. For TeleOp
     * bindings - releases should bind {@link #stopCmd}.
     */
    public Command reverseHoldCmd() {
        return run(this::runReverse);
    }

    /** PID-brake the transfer to zero. */
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
        telemetry.addData("Transfer target RPM", TRANSFER_RPM);
        telemetry.addData("Transfer actual RPM", "%.1f", getActualRpm());
        telemetry.addData("Transfer target tps", "%.1f", rpmToTicksPerSec(TRANSFER_RPM));
        telemetry.addData("Transfer actual tps", "%.1f", motor.getVelocity());
    }
}
