package org.firstinspires.ftc.teamcode.killerwatts;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase;

import java.util.Locale;

/**
 * Single continuous-rotation servo (spinner, intake wheel, feeder...).
 *
 * <p>Hardware only - gamepad bindings live in {@code KeyBindings} (KeyBindings drives
 * it via {@link #setPower}; add a section there when this is wired in).
 *
 * <p>To make a new mechanism: copy this file, rename the class, change
 * {@link #SERVO_NAME} to the hardware-map name. That's it.
 *
 * <p>Panels-tunable (Panels -&gt; ContinuousServo): direction, caps,
 * hold powers, deadband. Hardware NAME change needs an OpMode restart (looked
 * up once in the constructor); everything else applies live.
 *
 * <p>Construct once in the OpMode (auto-registers with SubsystemBase):
 * <pre>
 * spinner = new ContinuousServo(this);
 * </pre>
 * then drive from code with {@link #setPower} / {@link #stop}.
 */
@Configurable
public class ContinuousServo extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // ---- TODO: edit these per mechanism (hardcoded, one servo per subsystem) ----
    /** Must match the RC config name. Changing it needs a restart. */
    public static String SERVO_NAME = "eeerrr";
    public static boolean REVERSED = false;
    /** Output cap so full command isn't full speed, 0..1. */
    public static double MAX_POWER = 1.0;
    /** Programmatic forward hold power (used by forward()/gamepad). */
    public static double FWD_POWER = 1.0;
    /** Programmatic reverse hold power (used by reverse()/gamepad). */
    public static double REV_POWER = -1.0;
    /** Commands under this park the servo instead of creeping. */
    public static double DEADBAND = 0.08;

    private final CRServo servo;

    /** Programmatic hold value when no gamepad input is held. */
    private double requested = 0;
    /** Last power actually sent to hardware (post-cap). */
    private double applied = 0;

    public ContinuousServo(LinearOpMode opMode) {
        servo = opMode.hardwareMap.get(CRServo.class, SERVO_NAME);

        servo.setDirection(REVERSED ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        servo.setPower(0);
    }

    // ---- Code API (KeyBindings drives these; autos set a hold power and forget) ----

    /**
     * Hold a power (-1..1, deadbanded + capped by {@link #MAX_POWER}).
     */
    public void setPower(double power) {
        requested = cap(power);
    }

    public void forward() {
        setPower(FWD_POWER);
    }

    public void reverse() {
        setPower(REV_POWER);
    }

    public void stop() {
        requested = 0;
        // Write immediately so auto sequences don't wait a loop.
        applied = 0;
        servo.setPower(0);
    }

    public double getRequested() {
        return requested;
    }

    public double getApplied() {
        return applied;
    }

    public boolean isRunning() {
        return applied != 0;
    }

    @Override
    public void Periodic() {
        // Live-tune support: Panels edits apply without restart.
        servo.setDirection(REVERSED ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        requested = cap(requested);

        applied = requested;
        servo.setPower(applied);

        telemetry.addData("CR " + SERVO_NAME,
                String.format(Locale.US, "pwr %.2f req %.2f", applied, requested));
        // NOTE: addData only, no update() here. See Intake.Periodic for why.
    }

    private static double cap(double power) {
        if (Math.abs(power) < DEADBAND) return 0;
        double cap = Math.min(1.0, Math.abs(MAX_POWER));
        return Math.min(cap, Math.max(-cap, power));
    }
}
