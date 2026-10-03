package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.WattageLib.lib.killaUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IronConstants;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.Locale;

/**
 * Single positional servo (claw, wrist, arm, diverter...).
 *
 * <p>Hardware only in this class - gamepad bindings live in {@code KeyBindings}
 * to use a gamepad inputs and control this mechanism add them there. ).
 *
 * <p>To make a new mechanism: copy this file, rename the class, change
 * add the MotorName to constants then Get it from the HardwareMap. That's it.
 *
 * <p>Panels-tunable (Panels -&gt; PositionalServo): presets, limits,
 * direction, slew. Hardware NAME change needs an OpMode restart (looked up once
 * in the constructor); everything else applies live.
 *
 * <p>Construct once in the OpMode (auto-registers with SubsystemBase):
 * <pre>
 * claw = new PositionalServo(this);
 * </pre>
 * then drive from code with {@link #setPosition}, {@link #toggle}, etc.
 */
@Configurable
public class ShootGateServo extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // ---- TODO: edit these per mechanism (hardcoded, one servo per subsystem) ----
    /** Must match the RC config name. Changing it needs a restart. */
    public String Servoname = IronConstants.FrontServoName;
    /** Preset A, e.g. closed / stowed. */
    public static double POS_A = 0.15;
    /** Preset B, e.g. open / scored. */
    public static double POS_B = 0.75;
    /** Software clamp. Keeps linkages off the hard stops. */
    public static double POS_MIN = 0.0;
    public static double POS_MAX = 1.0;
    public static boolean REVERSED = false;
    /**
     * Slew limit, position units/sec. 1.5 = full sweep in ~0.67s (gentle on arms).
     * 0 or negative = snap instantly.
     */
    public static double SLEW = 1.5;
    /** Hold-to-nudge speed while a bumper is held, position units/sec. */
    public static double NUDGE_RATE = 0.6;
    /** Within this of target counts as "there" for {@link #isAtPosition}. */
    public static double TOLERANCE = 0.01;

    private final Servo servo;
    private final ElapsedTime timer = new ElapsedTime();

    private double target;
    private double current;
    private double lastTime = Double.NaN;


    public ShootGateServo(OpMode opMode) {
        servo = opMode.hardwareMap.get(Servo.class, Servoname);

        target = clamp(POS_A, POS_MIN, POS_MAX);
        current = target;
        servo.setDirection(REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo.setPosition(current);
        timer.reset();
        register(); // required for CommandScheduler.run() to call periodic()
    }

    // ---- Code API (KeyBindings + CommandFactory drive these; autos use them too) ----

    /** Command a position (0..1, clamped to MIN/MAX). Slew-limited on the way out. */
    public void setPosition(double position) {
        target = clamp(position, POS_MIN, POS_MAX);
    }

    /** Snap target AND output immediately (bypasses slew once). Useful for init. */
    public void setPositionImmediate(double position) {
        target = clamp(position, POS_MIN, POS_MAX);
        current = target;
        servo.setPosition(current);
    }

    public void goA() {
        setPosition(POS_A);
    }

    public void goB() {
        setPosition(POS_B);
    }

    /** Toggle between POS_A and POS_B (whichever is farther from target). */
    public void toggle() {
        double dA = Math.abs(target - POS_A);
        double dB = Math.abs(target - POS_B);
        setPosition(dA < dB ? POS_B : POS_A);
    }

    /** Relative jog, clamped. Positive = toward MAX. */
    public void nudge(double delta) {
        setPosition(target + delta);
    }

    public double getTarget() {
        return target;
    }

    /** Last slew-limited position actually sent to hardware. */
    public double getCommanded() {
        return current;
    }

    public boolean isAtPosition() {
        return Math.abs(target - current) <= TOLERANCE;
    }

    @Override
    public void periodic() {
        double now = timer.seconds();
        double rawDt = Double.isNaN(lastTime) ? 0.02 : Math.max(0, now - lastTime);
        double dtSec = Math.min(rawDt, 0.25); // Panels pause shouldn't slingshot servos
        lastTime = now;

        // Live-tune support: Panels edits apply without restart.
        // NOTE: no scaleRange() on purpose -- positions are absolute 0..1 and
        // software-clamped, so scaleRange would double-map them.
        servo.setDirection(REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        target = clamp(target, Math.min(POS_MIN, POS_MAX), Math.max(POS_MIN, POS_MAX));

        if (SLEW > 0) {
            double maxMove = SLEW * dtSec;
            double err = target - current;
            if (Math.abs(err) <= maxMove) {
                current = target;
            } else {
                current += Math.signum(err) * maxMove;
            }
        } else {
            current = target;
        }
        servo.setPosition(current);

        telemetry.addData("Pos " + Servoname,
                String.format(Locale.US, "tgt %.3f cmd %.3f %s", target, current,
                        isAtPosition() ? "AT" : "moving"));
        // NOTE: addData only, no update() here. See Intake.Periodic for why.
    }


}
