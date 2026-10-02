package org.firstinspires.ftc.teamcode.killerwatts.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.button.Trigger;

import java.util.function.BooleanSupplier;

/**
 * WPILib-style fluent bindings for an FTC gamepad — feature parity with
 * WPILib's {@code edu.wpi.first.wpilibj2.command.button.CommandXboxController},
 * adapted to the FTC SDK {@link Gamepad} and SolversLib {@link Trigger}s.
 *
 * <p>Wrap the SDK gamepad once, then bind commands in one readable line each —
 * all in a constructor or init block, no per-loop bookkeeping:
 *
 * <pre>
 * CommandGamepad controller = new CommandGamepad(opMode.gamepad1);
 *
 * controller.a().whenActive(robot.CommandF.ServoTogglePos);
 * controller.leftBumper().whenActive(robot.CommandF.HoldFlywheelTunable())
 *                        .whenInactive(robot.CommandF.StopFlywheel());
 * controller.leftTrigger().whileActiveOnce(aimCmd, true);  // default 0.5 threshold
 * controller.leftTrigger(0.3).whileActiveOnce(aimCmd, true);
 * controller.povUp().whileActiveOnce(robot.CommandF.NudgeServo(true), true);
 * controller.axisGreaterThan(AXIS_LEFT_X, 0.5).whenActive(cmd);  // generic axes
 * controller.a().and(controller.b()).whenActive(cmd);            // logical chain
 * </pre>
 *
 * <p>Differences from WPILib (FTC adaptations, documented so nothing surprises):
 * <ul>
 *   <li>No {@code EventLoop} overloads — SolversLib {@link Trigger} is polled by
 *       {@code CommandScheduler.run()} every loop; there is no alternate-loop API.</li>
 *   <li>No raw {@code button(int)} — the FTC SDK gamepad exposes named buttons
 *       only; {@link #button(int)} maps the Xbox standard button indices
 *       (A=1 ... RightStick=10, see the BUTTON_* constants) to named accessors.</li>
 *   <li>D-pad is exposed as {@code pov*()} (WPILib parity) plus explicit
 *       {@code dpadUp()}-style aliases used by this team's existing bindings.
 *       The FTC SDK has no diagonal dpad fields, so diagonal {@code pov*()}
 *       triggers fire when the two adjacent cardinal dpads are held.</li>
 *   <li>Stick Y getters: the FTC SDK reports stick-up as negative. Getters here
 *       return raw SDK values ({@code getLeftY()} is "up negative"), matching
 *       WPILib's "back is positive" convention only in spirit — check the
 *       per-method javadoc before porting FRC bindings verbatim.</li>
 * </ul>
 *
 * <p>Trigger methods available on every returned {@link Trigger}:
 * {@code whenActive} (press), {@code whenInactive} (release),
 * {@code whileActiveOnce} (hold; interrupts on release),
 * {@code whileActiveContinuous} (hold; keeps re-scheduling),
 * {@code toggleOnActive} (edge->toggle), plus {@code and}/{@code or}/{@code negate}
 * logical chaining.
 */
public class CommandGamepad {

    /** Xbox standard button indices used by {@link #button(int)} (1-based, WPILib-style). */
    public static final int BUTTON_A = 1, BUTTON_B = 2, BUTTON_X = 3, BUTTON_Y = 4;
    public static final int BUTTON_LEFT_BUMPER = 5, BUTTON_RIGHT_BUMPER = 6;
    public static final int BUTTON_BACK = 7, BUTTON_START = 8;
    public static final int BUTTON_LEFT_STICK = 9, BUTTON_RIGHT_STICK = 10;

    /** Generic axis indices for {@link #axisGreaterThan(int, double)}-style calls (WPILib order). */
    public static final int AXIS_LEFT_X = 0, AXIS_LEFT_Y = 1, AXIS_RIGHT_X = 2, AXIS_RIGHT_Y = 3;
    public static final int AXIS_LEFT_TRIGGER = 4, AXIS_RIGHT_TRIGGER = 5;

    /** Default threshold for the no-arg trigger overloads (WPILib parity: 0.5). */
    public static final double DEFAULT_TRIGGER_THRESHOLD = 0.5;
    /** Default stick activation threshold for {@code leftStick(...)}-style overloads. */
    public static final double DEFAULT_STICK_THRESHOLD = 0.5;

    private final Gamepad gp;

    /** Wrap an SDK gamepad (typically {@code opMode.gamepad1} or {@code gamepad2}). */
    public CommandGamepad(Gamepad gamepad) {
        if (gamepad == null) throw new IllegalArgumentException("gamepad is null");
        this.gp = gamepad;
    }

    // ================================================================== buttons
    // (No EventLoop overloads — SolversLib Trigger has no alternate-loop API.)

    public Trigger a() { return trig(() -> gp.a); }
    public Trigger b() { return trig(() -> gp.b); }
    public Trigger x() { return trig(() -> gp.x); }
    public Trigger y() { return trig(() -> gp.y); }

    public Trigger leftBumper()  { return trig(() -> gp.left_bumper); }
    public Trigger rightBumper() { return trig(() -> gp.right_bumper); }

    public Trigger back()  { return trig(() -> gp.back); }
    public Trigger start() { return trig(() -> gp.start); }
    /** Xbox guide / PS home button (FTC SDK: {@code gamepad.guide}). */
    public Trigger guide() { return trig(() -> gp.guide); }

    /** Pressing the left stick (Xbox L3 / PS L3). */
    public Trigger leftStick()  { return stickTrig(() -> gp.left_stick_button); }
    /** Pressing the right stick (Xbox R3 / PS R3). */
    public Trigger rightStick() { return stickTrig(() -> gp.right_stick_button); }

    // ------------------------------------------------- generic button (WPILib parity)

    /**
     * WPILib {@code CommandGenericHID.button(int)} parity: standard Xbox button
     * index (1-10, see the BUTTON_* constants) to its named trigger. The FTC SDK
     * gamepad has no raw button indexing, so this maps to the named fields.
     *
     * @param button button index, 1-based ({@link #BUTTON_A} .. {@link #BUTTON_RIGHT_STICK})
     */
    public Trigger button(int button) {
        switch (button) {
            case BUTTON_A:           return a();
            case BUTTON_B:           return b();
            case BUTTON_X:           return x();
            case BUTTON_Y:           return y();
            case BUTTON_LEFT_BUMPER: return leftBumper();
            case BUTTON_RIGHT_BUMPER:return rightBumper();
            case BUTTON_BACK:        return back();
            case BUTTON_START:       return start();
            case BUTTON_LEFT_STICK:  return leftStick();
            case BUTTON_RIGHT_STICK: return rightStick();
            default: throw new IllegalArgumentException("Unknown button index: " + button);
        }
    }

    // ================================================================== analog triggers

    /** True while the left trigger axis exceeds {@link #DEFAULT_TRIGGER_THRESHOLD}. */
    public Trigger leftTrigger() { return leftTrigger(DEFAULT_TRIGGER_THRESHOLD); }
    /** True while the left trigger axis exceeds {@code threshold} (0..1). */
    public Trigger leftTrigger(double threshold) {
        return trig(() -> gp.left_trigger > threshold);
    }

    /** True while the right trigger axis exceeds {@link #DEFAULT_TRIGGER_THRESHOLD}. */
    public Trigger rightTrigger() { return rightTrigger(DEFAULT_TRIGGER_THRESHOLD); }
    /** True while the right trigger axis exceeds {@code threshold} (0..1). */
    public Trigger rightTrigger(double threshold) {
        return trig(() -> gp.right_trigger > threshold);
    }

    /** Raw left trigger axis value (0..1). */
    public double getLeftTriggerAxis()  { return gp.left_trigger; }
    /** Raw right trigger axis value (0..1). */
    public double getRightTriggerAxis() { return gp.right_trigger; }

    // ================================================================== stick axes

    /**
     * True while the left stick's X axis exceeds {@code threshold} in magnitude
     * (either direction). {@code leftX()} with no args uses
     * {@link #DEFAULT_STICK_THRESHOLD}.
     */
    public Trigger leftX() { return axisMagnitudeGreaterThan(AXIS_LEFT_X, DEFAULT_STICK_THRESHOLD); }
    /** Magnitude-thresholded left-stick X trigger with explicit threshold. */
    public Trigger leftX(double threshold) {
        return axisMagnitudeGreaterThan(AXIS_LEFT_X, threshold);
    }
    /** Magnitude-thresholded left-stick Y trigger (default threshold). */
    public Trigger leftY() { return axisMagnitudeGreaterThan(AXIS_LEFT_Y, DEFAULT_STICK_THRESHOLD); }
    /** Magnitude-thresholded left-stick Y trigger with explicit threshold. */
    public Trigger leftY(double threshold) {
        return axisMagnitudeGreaterThan(AXIS_LEFT_Y, threshold);
    }
    /** Magnitude-thresholded right-stick X trigger (default threshold). */
    public Trigger rightX() { return axisMagnitudeGreaterThan(AXIS_RIGHT_X, DEFAULT_STICK_THRESHOLD); }
    /** Magnitude-thresholded right-stick X trigger with explicit threshold. */
    public Trigger rightX(double threshold) {
        return axisMagnitudeGreaterThan(AXIS_RIGHT_X, threshold);
    }
    /** Magnitude-thresholded right-stick Y trigger (default threshold). */
    public Trigger rightY() { return axisMagnitudeGreaterThan(AXIS_RIGHT_Y, DEFAULT_STICK_THRESHOLD); }
    /** Magnitude-thresholded right-stick Y trigger with explicit threshold. */
    public Trigger rightY(double threshold) {
        return axisMagnitudeGreaterThan(AXIS_RIGHT_Y, threshold);
    }

    /**
     * Left stick X axis. RAW SDK value: right positive, up/forward is NEGATIVE
     * (FTC convention). Compare WPILib: "right is positive" — same sign for X.
     */
    public double getLeftX() { return gp.left_stick_x; }
    /**
     * Left stick Y axis. RAW SDK value: stick-UP is NEGATIVE (FTC convention).
     * WPILib documents "back is positive" — the SDK matches this directly.
     */
    public double getLeftY() { return gp.left_stick_y; }
    /** Right stick X axis (raw SDK: right positive). */
    public double getRightX() { return gp.right_stick_x; }
    /** Right stick Y axis (raw SDK: stick-up negative / back positive). */
    public double getRightY() { return gp.right_stick_y; }

    // ================================================================== generic axes (CommandGenericHID parity)

    /** True while axis {@code axis} is greater than {@code threshold} (signed). */
    public Trigger axisGreaterThan(int axis, double threshold) {
        return trig(() -> axisValue(axis) > threshold);
    }
    /** True while axis {@code axis} is less than {@code threshold} (signed). */
    public Trigger axisLessThan(int axis, double threshold) {
        return trig(() -> axisValue(axis) < threshold);
    }
    /** True while axis {@code axis}'s magnitude exceeds {@code threshold} (either direction). */
    public Trigger axisMagnitudeGreaterThan(int axis, double threshold) {
        return trig(() -> Math.abs(axisValue(axis)) > threshold);
    }

    private double axisValue(int axis) {
        switch (axis) {
            case AXIS_LEFT_X:      return gp.left_stick_x;
            case AXIS_LEFT_Y:      return gp.left_stick_y;
            case AXIS_RIGHT_X:     return gp.right_stick_x;
            case AXIS_RIGHT_Y:     return gp.right_stick_y;
            case AXIS_LEFT_TRIGGER:  return gp.left_trigger;
            case AXIS_RIGHT_TRIGGER: return gp.right_trigger;
            default: throw new IllegalArgumentException("Unknown axis index: " + axis);
        }
    }

    // ================================================================== dpad / pov

    public Trigger dpadUp()    { return trig(() -> gp.dpad_up); }
    public Trigger dpadDown()  { return trig(() -> gp.dpad_down); }
    public Trigger dpadLeft()  { return trig(() -> gp.dpad_left); }
    public Trigger dpadRight() { return trig(() -> gp.dpad_right); }

    // ------------------------------------------------- pov* (WPILib CommandGenericHID parity)

    /**
     * True while the POV is within 45 degrees of {@code angle}
     * (degrees, 0 = up, clockwise). Diagonals map to the two adjacent
     * cardinal dpad booleans (the FTC SDK has no separate diagonal fields).
     */
    public Trigger pov(int angle) {
        // Snap to the nearest 45-degree sector (0 = up, clockwise).
        switch (Math.floorMod(angle + 22, 360) / 45) {
            case 0: return povUp();
            case 1: return povUpRight();
            case 2: return povRight();
            case 3: return povDownRight();
            case 4: return povDown();
            case 5: return povDownLeft();
            case 6: return povLeft();
            default: return povUpLeft();
        }
    }
    public Trigger povUp()        { return trig(() -> gp.dpad_up); }
    /** Up-LEFT diagonal: both dpad_up and dpad_left held (SDK has no diagonal field). */
    public Trigger povUpLeft()    { return trig(() -> gp.dpad_up && gp.dpad_left); }
    /** Up-RIGHT diagonal: both dpad_up and dpad_right held. */
    public Trigger povUpRight()   { return trig(() -> gp.dpad_up && gp.dpad_right); }
    public Trigger povLeft()      { return trig(() -> gp.dpad_left); }
    public Trigger povRight()     { return trig(() -> gp.dpad_right); }
    public Trigger povDown()      { return trig(() -> gp.dpad_down); }
    /** Down-LEFT diagonal: both dpad_down and dpad_left held. */
    public Trigger povDownLeft()  { return trig(() -> gp.dpad_down && gp.dpad_left); }
    /** Down-RIGHT diagonal: both dpad_down and dpad_right held. */
    public Trigger povDownRight() { return trig(() -> gp.dpad_down && gp.dpad_right); }
    /** True while the POV/dpad is NOT touched at all (WPILib {@code povCenter}). */
    public Trigger povCenter() {
        return trig(() -> !gp.dpad_up && !gp.dpad_down
                && !gp.dpad_left && !gp.dpad_right);
    }

    // ================================================================== PS aliases

    public Trigger circle()   { return a(); }
    public Trigger cross()    { return x(); }
    public Trigger square()   { return b(); }
    public Trigger triangle() { return y(); }

    // ================================================================== misc (CommandGenericHID parity)

    /** Raw button getter parity: true while the named button index is pressed. */
    public boolean getRawButton(int button) {
        switch (button) {
            case BUTTON_A:           return gp.a;
            case BUTTON_B:           return gp.b;
            case BUTTON_X:           return gp.x;
            case BUTTON_Y:           return gp.y;
            case BUTTON_LEFT_BUMPER: return gp.left_bumper;
            case BUTTON_RIGHT_BUMPER:return gp.right_bumper;
            case BUTTON_BACK:        return gp.back;
            case BUTTON_START:       return gp.start;
            case BUTTON_LEFT_STICK:  return gp.left_stick_button;
            case BUTTON_RIGHT_STICK: return gp.right_stick_button;
            default: throw new IllegalArgumentException("Unknown button index: " + button);
        }
    }

    /** Connection check parity (WPILib): true while the DS reports this gamepad attached. */
    public boolean isConnected() { return gp.getGamepadId() != -1; }

    /** The wrapped SDK gamepad (WPILib {@code getHID()} parity). */
    public Gamepad getHID() { return gp; }

    // ================================================================== internals

    /** Threshold-tolerant trigger factory (kept in one place for future debounce). */
    private static Trigger trig(BooleanSupplier condition) {
        return new Trigger(condition);
    }

    private static Trigger stickTrig(BooleanSupplier condition) {
        return new Trigger(condition);
    }
}
