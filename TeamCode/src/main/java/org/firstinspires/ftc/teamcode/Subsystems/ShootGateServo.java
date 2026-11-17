package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.WattageLib.lib.killaUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.Locale;

/**
 * Single positional servo controlling the shoot gate before the flywheel.
 *
 * <p>Panels-tunable (Panels -> ShootGateServo): presets ({@link #POS_A}, {@link #POS_B}),
 * limits, direction, and slew rate apply live.
 */
@Configurable
public class ShootGateServo extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    /** RC config servo name (changing requires an OpMode restart). */
    public String Servoname = IronConstants.FrontServoName;
    /** Closed / stowed preset position (0..1). */
    public static double POS_A = 0.15;
    /** Open / firing preset position (0..1). */
    public static double POS_B = 0.75;
    /** Minimum allowed servo position. */
    public static double POS_MIN = 0.0;
    /** Maximum allowed servo position. */
    public static double POS_MAX = 1.0;
    /** True reverses the servo output direction. */
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
    /** Shared dt tracker for the nudge commands (rate * dt, loop-time independent). */
    private long lastNudgeNs = -1L;
    private Servo.Direction lastDirection = null;
    private double lastWrittenPos = Double.NaN;


    public ShootGateServo(OpMode opMode) {
        servo = opMode.hardwareMap.get(Servo.class, Servoname);

        target = clamp(POS_A, POS_MIN, POS_MAX);
        current = target;
        Servo.Direction dir = REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD;
        servo.setDirection(dir);
        lastDirection = dir;
        servo.setPosition(current);
        lastWrittenPos = current;
        timer.reset();
    }

    // ---- Code API (KeyBindings drives these; autos use them too) ----

    /** Command a position (0..1, clamped to MIN/MAX). Slew-limited on the way out. */
    public void setPosition(double position) {
        target = clamp(position, POS_MIN, POS_MAX);
    }

    /** Snap target AND output immediately (bypasses slew once). Useful for init. */
    public void setPositionImmediate(double position) {
        target = clamp(position, POS_MIN, POS_MAX);
        current = target;
        servo.setPosition(current);
        lastWrittenPos = current;
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

    /**
     * True once the software slew ramp has reached {@link #target}.
     * Note: servos have no position sensor, so this reflects commanded motion only.
     */
    public boolean isAtPosition() {
        return Math.abs(target - current) <= TOLERANCE;
    }

    /** True when the gate target is set to closed ({@link #POS_A}). */
    public boolean isClosed() {
        return Math.abs(target - POS_A) <= TOLERANCE;
    }

    /** True when the gate target is open ({@link #POS_B}) and the servo has finished slewing there. */
    public boolean isOpen() {
        return Math.abs(target - POS_B) <= TOLERANCE && isAtPosition();
    }

    // ---- Commands (single-subsystem; multi-subsystem sequences live in CommandFactory) ----

    /**
     * Toggle between POS_A and POS_B, holding the requirement until the slew
     * arrives so a preset can't be interrupted mid-travel by another servo cmd.
     */
    public Command toggleCmd() {
        return new FunctionalCommand(
                this::toggle,        // initialize
                () -> {},            // execute (slew runs in periodic())
                (interrupted) -> {}, // end
                this::isAtPosition,  // isFinished
                this);               // requirement
    }

    /** Go to an absolute position, holding the requirement until arrival. */
    public Command toPos(double gotoPos) {
        return new FunctionalCommand(
                () -> setPosition(gotoPos), // initialize
                () -> {},                   // execute (slew runs in periodic())
                (interrupted) -> {},        // end
                this::isAtPosition,         // isFinished
                this);                      // requirement
    }

    /**
     * Nudge the servo target each loop the command runs (loop-time independent:
     * dt is measured internally). Rate and direction per ShootGateServo.
     *
     * @param up true nudges toward one end, false toward the other (matches the
     *           dpad_up/dpad_down binding directions)
     */
    public Command nudgeCmd(boolean up) {
        return run(() -> {
            long nowNs = System.nanoTime();
            double dt = lastNudgeNs < 0 ? 0.02 : (nowNs - lastNudgeNs) / 1.0e9;
            dt = Math.min(Math.max(dt, 0.0), 0.25);
            lastNudgeNs = nowNs;
            nudge((up ? -1 : 1) * NUDGE_RATE * dt);
        });
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
        Servo.Direction dir = REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD;
        if (dir != lastDirection) {
            servo.setDirection(dir);
            lastDirection = dir;
            lastWrittenPos = Double.NaN;
        }
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
        if (Double.isNaN(lastWrittenPos) || Math.abs(current - lastWrittenPos) > 1e-4) {
            servo.setPosition(current);
            lastWrittenPos = current;
        }

        if (RobotMain.DASHBOARD) {
            telemetry.addData("Pos " + Servoname,
                    String.format(Locale.US, "tgt %.3f cmd %.3f %s", target, current,
                            isAtPosition() ? "AT" : "moving"));
        }
        // NOTE: addData only, no update() here. RobotMain.flushTelemetry() does the single flush.
    }


}
