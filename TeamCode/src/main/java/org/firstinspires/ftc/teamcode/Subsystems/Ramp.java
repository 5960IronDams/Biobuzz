package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.WattageLib.lib.killaUtils.clamp;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareHealth;
import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.Locale;

/**
 * Ramp subsystem: two positional servos driving a crossbar to raise and lower the front ramp.
 *
 * <p>Create via {@link #tryCreate(OpMode)} so missing hardware returns {@code null} cleanly.
 *
 * <p>Key constants (Panels -&gt; Ramp):
 * <ul>
 *   <li>{@link #POS_UP} — parked (vertical) servo position.</li>
 *   <li>{@link #POS_DOWN} — lowered servo position for driving up.</li>
 *   <li>{@link #RIGHT_REVERSED} / {@link #LEFT_REVERSED} — mirror servo output in software.</li>
 *   <li>{@link #SLEW} — max travel speed (units/sec); 0 disables slew smoothing.</li>
 * </ul>
 */
@Configurable
public class Ramp extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    /** DS failure-board name for this subsystem (HardwareHealth). */
    public static final String HEALTH_NAME = "RAMP";

    /** RC config servo names (changing these requires an OpMode restart). */
    public String LeftServoName = IronConstants.DriverSideServoName;
    public String RightServoName = IronConstants.PassengerSideServoName;

    /** Parked (vertical) position in un-mirrored servo units. */
    public static double POS_UP = 0.0;
    /** Lowered position in un-mirrored servo units (higher moves toward the floor when DOWN_IS_INCREASING is +1). */
    public static double POS_DOWN = 0.405;

    /** Jog direction for lowering: +1 = larger position lowers ramp, -1 = larger position raises ramp. */
    public static double DOWN_IS_INCREASING = 1.0;

    /** Mirror the right servo output (1.0 - pos) so both crossbar arms move together. */
    public static boolean RIGHT_REVERSED = true;
    /** Mirror the left servo output (1.0 - pos). */
    public static boolean LEFT_REVERSED = false;

    /** Minimum allowed servo position limit. */
    public static double POS_MIN = 0.0;
    /** Maximum allowed servo position limit. */
    public static double POS_MAX = 1.0;

    /** Max travel speed in servo units/sec (1.0 = full sweep in ~1s; <= 0 moves instantly). */
    public static double SLEW = 1.0;
    /** Calibration jog speed while button is held (servo units/sec). */
    public static double JOG_RATE = 0.25;
    /** Minimum jog step per tick so short button taps still register. */
    public static double JOG_STEP_MIN = 0.002;
    /** Channel moved by {@link #jogCmd(boolean)}: 0 = left, 1 = right, 2 = both. */
    public static int JOG_SELECT = 2;
    /** Position tolerance for {@link #isAtPosition()}. */
    public static double TOLERANCE = 0.01;

    private final Servo left;
    private final Servo right;
    private final ElapsedTime timer = new ElapsedTime();

    private double leftTarget;
    private double rightTarget;
    private double leftCommanded;
    private double rightCommanded;
    private double lastLeftOut = Double.NaN;
    private double lastRightOut = Double.NaN;

    private double lastTime = Double.NaN;
    private long lastJogNs = -1L;

    private Ramp(OpMode opMode, Servo left, Servo right) {
        this.left = left;
        this.right = right;

        leftTarget = clamp(POS_UP, Math.min(POS_MIN, POS_MAX), Math.max(POS_MIN, POS_MAX));
        rightTarget = leftTarget;
        leftCommanded = leftTarget;
        rightCommanded = rightTarget;
        writeNow();
        timer.reset();
        Log.i("IronLog", "Ramp created (" + LeftServoName + " + " + RightServoName
                + ", parked at POS_UP=" + POS_UP + ", right mirrored=" + RIGHT_REVERSED + ")");
    }

    /**
     * Null-safe factory: looks up both servos before constructing/registering the subsystem.
     * Returns null if either servo is missing from the RC config.
     */
    public static Ramp tryCreate(OpMode opMode) {
        final Servo l;
        final Servo r;
        try {
            l = opMode.hardwareMap.get(Servo.class, IronConstants.DriverSideServoName);
            r = opMode.hardwareMap.get(Servo.class, IronConstants.PassengerSideServoName);
        } catch (Exception e) {
            Log.w("IronLog", "Ramp skipped — needs '" + IronConstants.DriverSideServoName
                    + "' and '" + IronConstants.PassengerSideServoName + "' in the RC config", e);
            // A ramp absent from the comp bot is match-critical (no driving up
            // the bar): DS banner before PLAY. Bench rigs stay quiet.
            if (IronConstants.CompBot) {
                String detail = e.getMessage();
                HardwareHealth.get().report(HEALTH_NAME, "absent at init ("
                        + e.getClass().getSimpleName()
                        + (detail == null ? ")" : ": " + detail + ")"));
            }
            return null;
        }
        return new Ramp(opMode, l, r);
    }

    // ---- Mapping helpers ---------------------------------------------------------

    /**
     * Converts un-mirrored target position to hardware servo output (0..1).
     * Reversal is handled strictly here rather than calling {@code Servo.setDirection()}.
     */
    private static double output(double target, boolean reversed) {
        return reversed ? 1.0 - target : target;
    }

    // ---- Code API (KeyBindings drives these; autos use them too) ----

    /** Command BOTH servos to the same position (clamped to MIN/MAX). Slew-limited on the way out. */
    public void setPosition(double position) {
        setSingle(0, position);
        setSingle(1, position);
    }

    /**
     * Command ONE channel. Calibration / rescue only — the normal path is
     * {@link #setPosition} so the crossbar stays square.
     *
     * @param channel 0 = left (driver side), 1 = right (passenger side)
     */
    public void setSingle(int channel, double position) {
        double t = clamp(position, Math.min(POS_MIN, POS_MAX), Math.max(POS_MIN, POS_MAX));
        if (channel == 0) {
            leftTarget = t;
        } else if (channel == 1) {
            rightTarget = t;
        }
    }

    /** Snap BOTH targets AND outputs to a position immediately (bypasses slew; bring-up / init). */
    public void setPositionImmediate(double position) {
        double t = clamp(position, Math.min(POS_MIN, POS_MAX), Math.max(POS_MIN, POS_MAX));
        leftTarget = t;
        rightTarget = t;
        leftCommanded = t;
        rightCommanded = t;
        writeNow();
    }

    /** Snap both servos to the parked POS_UP (the install-the-horns position). */
    public void parkHere() {
        setPositionImmediate(POS_UP);
    }

    /** Both servos to the parked (vertical, 0) position, through the slew ramp. */
    public void goUp() {
        setPosition(POS_UP);
    }

    /** Both servos to the lowered position, through the slew ramp. */
    public void goDown() {
        setPosition(POS_DOWN);
    }

    /** Which preset (if either) BOTH servos are currently commanded at, for the readout. */
    public String getState() {
        boolean atUp = Math.abs(leftTarget - POS_UP) <= TOLERANCE
                && Math.abs(rightTarget - POS_UP) <= TOLERANCE;
        boolean atDown = Math.abs(leftTarget - POS_DOWN) <= TOLERANCE
                && Math.abs(rightTarget - POS_DOWN) <= TOLERANCE;
        if (atUp) return "UP (parked)";
        if (atDown) return "DOWN";
        return isAtPosition() ? "HOLD" : "MOVING";
    }

    /** True while the commanded preset is DOWN (or on its way there). */
    public boolean isDown() {
        return Math.abs(leftTarget - POS_DOWN) <= TOLERANCE
                && Math.abs(rightTarget - POS_DOWN) <= TOLERANCE;
    }

    public double getLeftTarget() {
        return leftTarget;
    }

    public double getRightTarget() {
        return rightTarget;
    }

    /**
     * True once the SLEW RAMP has reached its target on BOTH servos.
     *
     * <p><b>CAREFUL — a statement about the software ramp, not the hardware.</b>
     * Servos have no position feedback: {@code commanded} is a value we
     * generate ourselves by slewing toward {@code target}. Do not gate a
     * collision-risky auto move on this alone; it is timing, not sensing.
     */
    public boolean isAtPosition() {
        return Math.abs(leftTarget - leftCommanded) <= TOLERANCE
                && Math.abs(rightTarget - rightCommanded) <= TOLERANCE;
    }

    /** Relative jog of one channel (or both), clamped. Calibration only. */
    private void jog(int channel, double delta) {
        if (channel == 0) {
            setSingle(0, leftTarget + delta);
        } else if (channel == 1) {
            setSingle(1, rightTarget + delta);
        } else {
            setSingle(0, leftTarget + delta);
            setSingle(1, rightTarget + delta);
        }
    }

    /** Clamp + mirror + write (cached so unchanged positions skip redundant REV Hub writes). */
    private void writeNow() {
        double lo = Math.min(POS_MIN, POS_MAX);
        double hi = Math.max(POS_MIN, POS_MAX);
        leftCommanded = clamp(leftCommanded, lo, hi);
        rightCommanded = clamp(rightCommanded, lo, hi);
        double outL = output(leftCommanded, LEFT_REVERSED);
        double outR = output(rightCommanded, RIGHT_REVERSED);
        if (Double.isNaN(lastLeftOut) || Math.abs(outL - lastLeftOut) > 1e-4) {
            left.setPosition(outL);
            lastLeftOut = outL;
        }
        if (Double.isNaN(lastRightOut) || Math.abs(outR - lastRightOut) > 1e-4) {
            right.setPosition(outR);
            lastRightOut = outR;
        }
    }

    /** Span between the two presets (negative-safe), for the travel-time estimate. */
    private double presetSpan() {
        return Math.abs(POS_DOWN - POS_UP);
    }

    // ---- Commands (single-subsystem; gamepad bindings live in KeyBindings) ----

    /**
     * Both servos UP, holding the requirement until the slew arrives so the
     * parked state can't be interrupted mid-travel by another ramp command.
     */
    public Command upCmd() {
        return new FunctionalCommand(
                this::goUp,          // initialize
                () -> {},            // execute (slew runs in periodic())
                (interrupted) -> {}, // end
                this::isAtPosition,  // isFinished
                this);               // requirement
    }

    /** Both servos DOWN, holding the requirement until the slew arrives. */
    public Command downCmd() {
        return new FunctionalCommand(
                this::goDown,        // initialize
                () -> {},            // execute (slew runs in periodic())
                (interrupted) -> {}, // end
                this::isAtPosition,  // isFinished
                this);               // requirement
    }

    /**
     * Goes to an explicit position on BOTH servos, holding the requirement
     * until arrival. Autos / intermediate heights.
     */
    public Command toPos(double gotoPos) {
        return new FunctionalCommand(
                () -> setPosition(gotoPos), // initialize
                () -> {},                   // execute (slew runs in periodic())
                (interrupted) -> {},        // end
                this::isAtPosition,         // isFinished
                this);                      // requirement
    }

    /**
     * Bring-up zero: snap BOTH servos to POS_UP. Bind this so the horns can be
     * installed against a known-0 output, and rescue a rig whose ramp is
     * somewhere unknown.
     */
    public Command zeroCmd() {
        return runOnce(this::parkHere);
    }

    /**
     * Hold-to-jog on the command's OWN channel selection ({@link #JOG_SELECT}),
     * re-read every loop so Panels edits apply live.
     *
     * @param down true = jog toward DOWN ({@link #DOWN_IS_INCREASING}), false toward UP
     */
    public Command jogCmd(boolean down) {
        return run(() -> {
            long nowNs = System.nanoTime();
            double dt = lastJogNs < 0 ? 0.02 : (nowNs - lastJogNs) / 1.0e9;
            dt = Math.min(Math.max(dt, 0.0), 0.25);
            lastJogNs = nowNs;
            // A tap is often shorter than one loop: make a sub-frame jog still
            // register one visible step instead of a round-to-zero no-op.
            double step = Math.max(JOG_RATE * dt, JOG_STEP_MIN);
            double sign = down ? Math.signum(DOWN_IS_INCREASING) : -Math.signum(DOWN_IS_INCREASING);
            jog(JOG_SELECT, sign * step);
        });
    }

    /**
     * Cycle which channel the jog moves: LEFT (0) -&gt; RIGHT (1) -&gt; BOTH (2).
     * Bind to a pad button so one arm can be trimmed to square the crossbar
     * without editing {@link #JOG_SELECT} on Panels.
     */
    public Command cycleJogSelectCmd() {
        return runOnce(() -> JOG_SELECT = (JOG_SELECT + 1) % 3);
    }

    /**
     * Edge-triggered TAP binding: {@code dpadDown} -> {@link #toggleDownCmd()},
     * {@code dpadUp} -> {@link #toggleUpCmd()}.
     *
     * <p>Semantics, chosen so a tap always leaves the ramp somewhere sane:
     * <ul>
     *   <li>never at a real preset: go toward it (a tap after a jog completes it).</li>
     *   <li>already there: from a settled UP-request, DOWN lowers; from a
     *       settled DOWN-request, UP re-parks. dpadDown therefore always ends
     *       LOWERED, dpadUp always ends PARKED — each direction is correct
     *       whichever state you were in, and the pair can never chatter.</li>
     * </ul>
     * Completes as soon as the slew arrives; nothing drives the servo after
     * that (a tap is a toggle, not a hold).
     *
     * @param toDown true = the DOWN command, false = the UP command
     */
    public Command toggleCmd(boolean toDown) {
        return new FunctionalCommand(
                () -> setPosition(presetTarget(toDown ? POS_DOWN : POS_UP)),
                () -> {},            // execute (slew runs in periodic())
                (interrupted) -> {}, // end
                this::isAtPosition,  // isFinished
                this);               // requirement
    }

    /** dpadDown tap: ends LOWERED (or continues a partial jog toward DOWN). */
    public Command toggleDownCmd() {
        return toggleCmd(true);
    }

    /** dpadUp tap: ends PARKED at POS_UP (or continues a partial jog toward UP). */
    public Command toggleUpCmd() {
        return toggleCmd(false);
    }

    /**
     * Where {@link #toggleCmd} actually goes for a requested preset. Both
     * servos share the LEFT target as the reference (they are driven together
     * outside calibration), so a lopsided jog can't fool it.
     */
    private double presetTarget(double preset) {
        double other = preset == POS_DOWN ? POS_UP : POS_DOWN;
        if (Math.abs(leftTarget - preset) > TOLERANCE) {
            return preset; // somewhere else: drive to the requested preset
        }
        return other; // already there: a tap means "the other state"
    }

    @Override
    public void periodic() {
        double now = timer.seconds();
        double rawDt = Double.isNaN(lastTime) ? 0.02 : Math.max(0, now - lastTime);
        double dtSec = Math.min(rawDt, 0.25);
        lastTime = now;

        // Direction is mirrored solely via output(); never call Servo.setDirection() here.
        double lo = Math.min(POS_MIN, POS_MAX);
        double hi = Math.max(POS_MIN, POS_MAX);
        leftTarget = clamp(leftTarget, lo, hi);
        rightTarget = clamp(rightTarget, lo, hi);

        if (SLEW > 0) {
            double maxMove = SLEW * dtSec;
            leftCommanded = slewToward(leftCommanded, leftTarget, maxMove);
            rightCommanded = slewToward(rightCommanded, rightTarget, maxMove);
        } else {
            leftCommanded = leftTarget;
            rightCommanded = rightTarget;
        }
        writeNow();

        report();
    }

    private static double slewToward(double current, double target, double maxMove) {
        double err = target - current;
        if (Math.abs(err) <= maxMove) return target;
        return current + Math.signum(err) * maxMove;
    }

    /**
     * Panels debug block. addData only, no update() — RobotMain.flushTelemetry()
     * does the single end-of-loop flush (a second update() per loop sends
     * partial frames and flickers).
     */
    private void report() {
        HardwareHealth.get().ok(HEALTH_NAME);
        if (!RobotMain.DASHBOARD) {
            return;
        }
        // The numbers a calibrating driver copies into POS_DOWN, in
        // un-mirrored units (identical to the raw servo value on an
        // un-mirrored channel).
        telemetry.addData("Ramp state", getState());
        telemetry.addData("Ramp position",
                String.format(Locale.US, "%s  cmd L%.3f R%.3f", isAtPosition() ? "AT" : "moving",
                        leftCommanded, rightCommanded));
        telemetry.addData("Ramp Left " + LeftServoName,
                String.format(Locale.US, "tgt %.3f -> out %.3f%s",
                        leftTarget, output(leftCommanded, LEFT_REVERSED),
                        LEFT_REVERSED ? " (REV)" : ""));
        telemetry.addData("Ramp Right " + RightServoName,
                String.format(Locale.US, "tgt %.3f -> out %.3f%s",
                        rightTarget, output(rightCommanded, RIGHT_REVERSED),
                        RIGHT_REVERSED ? " (REV)" : ""));
        telemetry.addData("Ramp presets",
                String.format(Locale.US, "UP %.3f  DOWN %.3f  span %.3f  travel %.2fs  %s",
                        POS_UP, POS_DOWN, presetSpan(),
                        (SLEW > 0 ? presetSpan() / SLEW : 0.0),
                        DOWN_IS_INCREASING >= 0 ? "down=+" : "down=-"));
        telemetry.addData("Ramp jog <-- POS_DOWN",
                String.format(Locale.US, "L %.3f  R %.3f  (jog sel %d)",
                        leftTarget, rightTarget, JOG_SELECT));
    }
}
