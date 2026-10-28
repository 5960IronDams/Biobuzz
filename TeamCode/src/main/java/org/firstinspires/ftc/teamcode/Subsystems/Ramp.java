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
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.Locale;

/**
 * Ramp subsystem: TWO positional servos direct-driving a 2-armed crossbar.
 *
 * <p>The crossbar pivots the front ramp between two states:
 * <ul>
 *   <li>{@link #POS_UP} — parked, ramp VERTICAL. This is servo position 0 in
 *       the raw output convention ({@code 0.0 = 0 us-cal end}); it is also the
 *       state the horns should be installed in (see "BRING-UP" below).</li>
 *   <li>{@link #POS_DOWN} — ramp lowered for driving up, found by jogging the
 *       servos toward DOWN and copying the commanded value.</li>
 * </ul>
 *
 * <p><b>WHY dpadDown LIVES ON gamepad2 (co-driver):</b> the same dpad_up /
 * dpad_down pair on gamepad1 already holds the shoot-gate servo nudge
 * ({@code KeyBindings}). Binding them twice on one pad would move the gate and
 * the ramp at the same time, so the ramp owns the OPERATOR pad's dpad:
 * <pre>
 *   gamepad2 dpad DOWN   toggle:  up (parked) -> down (lowered)   [tap]
 *   gamepad2 dpad UP     toggle:  any -> up (parked, 0)           [tap]
 *   gamepad2 LB / RB     held:    jog toward DOWN / toward UP (calibration)
 *   gamepad2 BACK        tap:     cycle jog channel LEFT -> RIGHT -> BOTH
 *   gamepad2 Y           tap:     snap BOTH servos to POS_UP (install the horns)
 * </pre>
 * The dpad bindings are TOGGLES, not holds: after a tap nothing is driving
 * the servo, so it holds position on its own (no "release drops the ramp"
 * failure mode, and holding HPU on the servos is avoided).
 *
 * <p><b>BRING-UP (the calibration the driver assumed — it is the intended one):</b>
 * <ol>
 *   <li>Power the robot, let the OpModes init. {@link #POS_UP} is written to
 *       both servos on construction and re-asserted every loop.</li>
 *   <li>WITH THE RAMP HELD VERTICAL (parked), press the horns onto the servo
 *       splines at position 0. Use the servo LEFT / servo RIGHT readouts to
 *       confirm both channels really are at 0.000 — if a channel was reversed
 *       or the horns went on 180 deg out, they will read 0.5/1.0 here.</li>
 *   <li>Jog down (gamepad2 LB): the jog channel steps toward DOWN ({@link
 *       #DOWN_IS_INCREASING} decides which servo-position direction that is;
 *       {@link #RIGHT_REVERSED} makes the mirrored right arm follow). Press
 *       BACK to jog only LEFT or RIGHT if the crossbar binds.</li>
 *   <li>When the ramp sits at its drive-up angle, copy the shown
 *       "Ramp jog" value here (identical for both arms unless one arm needed a
 *       single-servo trim to square the crossbar). Panels -&gt; Ramp applies
 *       live. Set {@link #IMPLEMENTED} = true.</li>
 * </ol>
 *
 * <p><b>Why a single-servo jog exists:</b> two servos on one crossbar can bind
 * if their horns are a spline apart. Cycle {@link #JOG_SELECT} (gamepad2
 * BACK) to LEFT or RIGHT, bring that arm flush with the other, then copy the
 * same position into both via {@link #POS_DOWN}.
 *
 * <p>No position feedback exists (standard servos), so "arrived" is a claim
 * about the software slew ramp only, exactly like {@code ShootGateServo}.
 *
 * <p>Panels-tunable (Panels -&gt; Ramp): POS_UP / POS_DOWN / limits /
 * direction / slew / jog rate / tolerance. Hardware NAME change needs an
 * OpMode restart (looked up once in the constructor); everything else applies
 * live.
 *
 * <p>Construct once in the OpMode via the null-safe factory (never the
 * constructor directly — see {@link #tryCreate} for why):
 * <pre>
 * ramp = Ramp.tryCreate(this);   // null when the servos aren't in the RC config
 * </pre>
 */
@Configurable
public class Ramp extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    /** DS failure-board name for this subsystem (HardwareHealth). */
    public static final String HEALTH_NAME = "RAMP";

    // ---- TODO: edit these per mechanism (hardcoded, two servos per subsystem) ----
    /**
     * Must match the RC config names. Changing them needs a restart. Both are
     * resolved in the constructor, so a Ramp whose servos are not in the RC
     * config throws here (RobotMain turns that into a DS fault + null).
     */
    public String LeftServoName = IronConstants.DriverSideServoName;
    public String RightServoName = IronConstants.PassengerSideServoName;

    /**
     * IS THE RAMP PHYSICALLY BUILT AND CALIBRATED? Kept for parity with
     * {@code ShootGateServo.IMPLEMENTED}: anything that sequences the ramp into
     * a match path should check this before gating on the ramp, and autos that
     * lower the ramp for driving should be written against {@link #downCmd()}
     * only once the true POS_DOWN is entered. It does NOT disable the servos —
     * the driver needs them live to calibrate.
     */
    public static boolean IMPLEMENTED = true;

    /** PARKED, ramp vertical. Servo 0 in the raw output convention — install the horns here. */
    public static double POS_UP = 0.0;
    /**
     * Lowered, ramp on the floor. CALIBRATE: jog down, then copy the shown
     * "Ramp jog" value here. Placeholder = half travel, NEVER drive on it.
     */
    public static double POS_DOWN = 0.5;

    /**
     * Which way is DOWN in servo units: +1 = a LARGER position lowers the ramp
     * (servo horn toward the floor on the driver / left side), -1 = a larger
     * position RAISES it. The right channel is mirrored by
     * {@link #RIGHT_REVERSED} on top of this, so both arms lower together.
     */
    public static double DOWN_IS_INCREASING = 1.0;

    /**
     * Mirror the passenger / right servo's direction. True when the two
     * crossbar arms are mounted on opposite sides of the robot: then one
     * channel must be flipped so a single commanded value lowers BOTH arms.
     * Verify during bring-up — if the arms fight, flip this (or the servo's
     * own RC-config direction).
     */
    public static boolean RIGHT_REVERSED = true;
    /** Mirror the driver / left servo. Rarely needed; here for a swapped-cable rescue. */
    public static boolean LEFT_REVERSED = false;

    /** Software clamp. Keeps the linkage off the hard stops / the floor. */
    public static double POS_MIN = 0.0;
    public static double POS_MAX = 1.0;

    /**
     * Slew limit, position units/sec, applied to the travel between UP and
     * DOWN. 1.0 = full sweep in ~1s. 0 or negative = snap instantly (only do
     * that once the mechanism is happy, servos slam otherwise).
     */
    public static double SLEW = 1.0;
    /** Hold-to-jog speed during CALIBRATION (units/sec). Faster than the travel slew. */
    public static double JOG_RATE = 0.25;
    /**
     * Smallest jog a single scheduler tick may command, so a dpad tap shorter
     * than one loop still visibly steps the ramp. Deliberately much finer than
     * {@link #TOLERANCE}: this is a CALIBRATION resolution, not an arrival check.
     */
    public static double JOG_STEP_MIN = 0.002;
    /** Which channel the jog moves: 0 = left, 1 = right, 2 = both (BACK cycles it). */
    public static int JOG_SELECT = 2;
    /** Within this of target counts as "there" for {@link #isAtPosition}. */
    public static double TOLERANCE = 0.01;

    private final Servo left;
    private final Servo right;
    private final ElapsedTime timer = new ElapsedTime();

    /** Requested positions, in the SAME (un-mirrored) units as POS_UP/POS_DOWN. */
    private double leftTarget;
    private double rightTarget;
    /** Last slew-limited positions actually written to hardware. */
    private double leftCommanded;
    private double rightCommanded;

    private double lastTime = Double.NaN;
    /** Shared dt tracker for the jog commands (rate * dt, loop-time independent). */
    private long lastJogNs = -1L;

    /**
     * Construction contract: {@link #tryCreate} is THE way to build a Ramp
     * from an OpMode, because it resolves the hardware BEFORE this constructor
     * runs. That is not stylistic — {@link SubsystemBase}'s constructor (run
     * by our implicit {@code super()}) registers this instance with the
     * CommandScheduler. If this body threw on a missing servo AFTER that point
     * the scheduler would keep a half-built object and call {@link #periodic()}
     * on it every loop, NPE-ing on the null servos — exactly the crash in
     * Match-0-JowBy_TeleOp ("setDirection ... on a null object reference").
     * {@code SubsystemBase} has no injectable constructor and Java forbids
     * touching {@code this} before {@code super()}, so the only correct spot
     * for the lookup is before construction.
     */
    private Ramp(OpMode opMode, Servo left, Servo right) {
        this.left = left;
        this.right = right;

        leftTarget = clamp(POS_UP, Math.min(POS_MIN, POS_MAX), Math.max(POS_MIN, POS_MAX));
        rightTarget = leftTarget;
        leftCommanded = leftTarget;
        rightCommanded = rightTarget;
        // NOTE: no setDirection() call here or anywhere else — see output().
        left.setPosition(output(leftCommanded, LEFT_REVERSED));
        right.setPosition(output(rightCommanded, RIGHT_REVERSED));
        timer.reset();
        Log.i("IronLog", "Ramp created (" + LeftServoName + " + " + RightServoName
                + ", parked at POS_UP=" + POS_UP + ", right mirrored=" + RIGHT_REVERSED + ")");
    }

    /**
     * Null-safe constructor: resolves both servos FIRST, then constructs (and
     * therefore registers) the subsystem. Returns null when either servo name
     * is missing from the RC config. See the private constructor for why the
     * lookup must precede construction.
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
     * Target (un-mirrored) -> what this channel is actually written with.
     * Mirrors the target, NOT the output, so a command can still be read back
     * in POS_UP/POS_DOWN units.
     *
     * <p><b>This is the ONLY place servo direction is applied.</b> Do not also
     * call {@code Servo.setDirection()} — doing both double-negates and the
     * reversal silently stops working (§13.1), and a per-loop setDirection()
     * also overrides whatever direction the DS / RC config asked for.
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

    /** Clamp + mirror + write. Shared by the constructor, parkHere() and setPositionImmediate(). */
    private void writeNow() {
        leftCommanded = clamp(leftCommanded, Math.min(POS_MIN, POS_MAX), Math.max(POS_MIN, POS_MAX));
        rightCommanded = clamp(rightCommanded, Math.min(POS_MIN, POS_MAX), Math.max(POS_MIN, POS_MAX));
        left.setPosition(output(leftCommanded, LEFT_REVERSED));
        right.setPosition(output(rightCommanded, RIGHT_REVERSED));
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
        double dtSec = Math.min(rawDt, 0.25); // Panels pause shouldn't slingshot servos
        lastTime = now;

        // Live-tune support: limits apply without restart.
        // NOTE: no scaleRange() on purpose -- positions are absolute 0..1,
        // mirrored for the right channel, and clamped here.
        //
        // §13.1: applyDirections() used to be called here EVERY loop. That was
        // a bug on two counts:
        //   1. Together with output()'s value mirroring it double-negated the
        //      reversal, so LEFT_REVERSED / RIGHT_REVERSED had NO physical
        //      effect no matter what you set (capture 20261029-012050).
        //   2. Calling setDirection() every loop made this the last word on
        //      servo direction, so changing direction on the DS / in the RC
        //      config was immediately overwritten — which is why the dashboard
        //      showed correct positions while the horn went the wrong way.
        // Direction is now mirrored in exactly ONE place: output(). Do not
        // reintroduce setDirection() calls here.
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
        if (IMPLEMENTED) {
            HardwareHealth.get().ok(HEALTH_NAME);
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
                String.format(Locale.US, "UP %.3f  DOWN %.3f  span %.3f  travel %.2fs  %s  [%s]",
                        POS_UP, POS_DOWN, presetSpan(),
                        (SLEW > 0 ? presetSpan() / SLEW : 0.0),
                        DOWN_IS_INCREASING >= 0 ? "down=+" : "down=-",
                        IMPLEMENTED ? "CALIBRATED" : "NOT CALIBRATED"));
        telemetry.addData("Ramp jog <-- POS_DOWN",
                String.format(Locale.US, "L %.3f  R %.3f  (jog sel %d)",
                        leftTarget, rightTarget, JOG_SELECT));
    }
}
