package org.firstinspires.ftc.teamcode.WattageLib.lib;

import com.bylazar.gamepad.GamepadManager;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.button.Trigger;

import java.util.function.BooleanSupplier;

/**
 * Drop-in replacement for {@link CommandGamepad} for XBOX controllers (Xbox One /
 * Series, USB or Bluetooth). Extends {@link CommandGamepad}, so every binding
 * written against {@code CommandGamepad} keeps working unchanged:
 *
 * <pre>
 * // Drop-in: same constructor, same methods, same constants.
 * private final CommandGamepad DriveController;          // existing field, type unchanged
 * this.DriveController = new CommandXboxController(gamepad1);
 * </pre>
 *
 * <h2>Axis remap (the reason this class exists)</h2>
 *
 * <p>Xbox controllers in BLE/odd HID modes report their axes in a different order
 * than the SDK's {@code left_stick_*} / {@code *_trigger} fields assume: the
 * trigger pair lands on the right-stick fields and the RIGHT STICK lands on the
 * TRIGGER fields (when the DS sees it at all). This class swaps those two axis
 * pairs back, so all the standard accessors mean the physical control you expect:
 *
 * <table summary="physical control -> SDK field -> accessor">
 *   <tr><th>Physical control</th><th>SDK field it arrives on</th><th>Read it with</th></tr>
 *   <tr><td>Left stick X/Y</td><td>{@code left_stick_x/y} (unchanged)</td>
 *       <td>{@link #getLeftX()} / {@link #getLeftY()}</td></tr>
 *   <tr><td>Right stick X (rotation)</td><td>{@code left_trigger} (signed -1..1)</td>
 *       <td>{@link #getRightX()} / {@code rightX()} / {@code AXIS_RIGHT_X}</td></tr>
 *   <tr><td>Right stick Y</td><td>{@code right_trigger} (signed -1..1)</td>
 *       <td>{@link #getRightY()} / {@code rightY()} / {@code AXIS_RIGHT_Y}</td></tr>
 *   <tr><td>Left trigger</td><td>{@code right_stick_x} (rest &lt;= 0, pull = +1)</td>
 *       <td>{@link #getLeftTriggerAxis()} / {@link #leftTrigger()} — normalized 0..1</td></tr>
 *   <tr><td>Right trigger</td><td>{@code right_stick_y} (rest &lt;= 0, pull = +1)</td>
 *       <td>{@link #getRightTriggerAxis()} / {@link #rightTrigger()} — normalized 0..1</td></tr>
 *   <tr><td>All buttons, bumpers, dpad, sticks-as-buttons</td><td>unchanged</td>
 *       <td>inherited from {@link CommandGamepad}</td></tr>
 * </table>
 *
 * <p>The swap is scoped to exactly this quadrant — the left stick and all buttons
 * are identical between the F310 and the Xbox pad and pass through untouched. By
 * default ({@link AxisRemapMode#AUTO}) the conversion applies ONLY to pads the
 * DS reports as {@code XBOX_360}: a real Logitech F310 (the team's backup
 * controller, already F310/SDK-native) passes through unmodified, so one build
 * drives either pad.
 *
 * <h2>Where the right stick goes when the DS does not see it (browser path)</h2>
 *
 * <p>On this controller the DS/Android HID stack sometimes delivers NO lane for
 * the right stick (while Windows/XINPUT shows it fine in the gamepad test — the
 * browser Gamepad API sees it correctly too). Code cannot recover a lane the DS
 * never delivers, so {@link #refreshPad(GamepadManager)} wires the Panels
 * Gamepad plugin in as a SECOND source: the driver opens the Panels page on a
 * browser with the Xbox pad connected to that machine, and the plugin's
 * {@code GamepadManager} supplies the correct right stick (and everything else)
 * through the standard browser mapping. Per-lane rule, re-evaluated every read:
 * a live browser value overrides the DS lane; otherwise the DS lane applies
 * (remapped for Xbox pads). So:
 * <ul>
 *   <li>Turn input ({@link #getRightX()}) = the Xbox controller's physical
 *       RIGHT STICK X — via the browser when Panels is open, via the swapped DS
 *       lane when it is not. The LEFT TRIGGER can never drive turn again
 *       (that was the boot-rotation bug: the old raw {@code right_stick_x}
 *       turn read sat on the left-trigger lane).</li>
 *   <li>Buttons/dpad/stick-clicks come from the merged pad ({@code gp}),
 *       which the plugin layers over the DS pad; triggers read the browser's
 *       {@code l2}/{@code r2} when live, else the normalized DS lanes.</li>
 *   <li>Rumble/LED always target the physical DS pad, never the merged copy.</li>
 * </ul>
 * Call {@link #refreshPad(GamepadManager)} once per loop BEFORE anything polls
 * this controller (see {@code KeyBindings.refreshPad()}).
 *
 * <p>Source of truth: the Driver Station's raw hardware-test readout (pre-code).
 * Observed there: the RIGHT-STICK-X readout responds to the LEFT trigger (and
 * the right-stick-Y readout to the right trigger) — the same pairing viewed from
 * the other side, since a swap is symmetric. On this controller the trigger
 * lanes rest at -1 (classic Android Xbox trigger quirk), which is exactly why
 * the robot rotated at boot whenever code read {@code right_stick_x} raw for
 * the drive turn. {@link #stickToTrigger(double)} normalizes both rest
 * conventions (-1 or 0) to a clean 0..1 trigger. Verify live with
 * {@link PanelsGamepadDebug} (wired into {@code KeyBindings}).
 *
 * <h2>Class-level configuration (global; set once in robot init)</h2>
 * <ul>
 *   <li>{@link #setStickDeadband(double)} — deadband (default
 *       {@link #DEFAULT_STICK_DEADBAND}) applied to all stick axes, rescaled so
 *       the full throw still maps to -1..1. Counters Xbox stick drift.</li>
 *   <li>{@link #axisRemapMode} / {@link #setAxisRemapMode(AxisRemapMode)} —
 *       {@link AxisRemapMode#AUTO} (default) remaps only XBOX_360 pads;
 *       {@link AxisRemapMode#ALWAYS}/{@link AxisRemapMode#NEVER} force it
 *       on/off for every pad (NEVER == plain {@link CommandGamepad} behavior,
 *       deadband still applied). Per-pad effective state: {@link #isRemapActive()}.</li>
 * </ul>
 *
 * <h2>Xbox-specific extras (FTC SDK 12.0)</h2>
 * <ul>
 *   <li>{@link #share()} / {@link #options()} buttons.</li>
 *   <li>Rumble + LED: {@link #rumble(int)}, {@link #rumble(double, double, int)},
 *       {@link #rumbleBlips(int)}, {@link #stopRumble()}, {@link #isRumbling()},
 *       {@link #setLedColor(double, double, double, int)},
 *       {@link #runRumbleEffect(Gamepad.RumbleEffect)},
 *       {@link #runLedEffect(Gamepad.LedEffect)}.</li>
 *   <li>{@link #getGamepadType()} / {@link #isXbox()} to verify what the DS
 *       detected (Xbox controllers report {@code Gamepad.Type.XBOX_360}).</li>
 * </ul>
 *
 * <p>The PS-style aliases ({@code circle()}/{@code cross()}/{@code square()}/
 * {@code triangle()}) are inherited unchanged so this stays a byte-for-byte
 * drop-in for {@link CommandGamepad} callers.
 */
public class CommandXboxController extends CommandGamepad {

    /** Default stick deadband ({@link #setStickDeadband(double)}). */
    public static final double DEFAULT_STICK_DEADBAND = 0.05;

    /** Class-level stick deadband, 0 = disabled. Global to all instances. */
    private static double stickDeadband = DEFAULT_STICK_DEADBAND;

    /**
     * How the right-stick/trigger axis remap is applied, per pad:
     * <ul>
     *   <li>{@link AxisRemapMode#AUTO} (default) — remap ONLY pads the DS reports
     *       as {@code XBOX_360}, converting Xbox hardware into the F310/SDK lane
     *       mapping the robot code (written for the Logitech F310) expects. A
     *       real F310 or PS4 pad passes through unmodified — swap controllers
     *       freely, no code or config change.</li>
     *   <li>{@link AxisRemapMode#ALWAYS} — remap every pad (old behavior).</li>
     *   <li>{@link AxisRemapMode#NEVER} — remap no pad (plain SDK lanes).</li>
     * </ul>
     * Panels-tunable class-level switch; re-evaluated every read, so a pad
     * change (or a Panels edit) applies on the next loop.
     */
    public static AxisRemapMode axisRemapMode = AxisRemapMode.AUTO;

    /** Axis remap policy. See {@link #axisRemapMode}. */
    public enum AxisRemapMode { AUTO, ALWAYS, NEVER }

    /** Set the stick deadband (0..0.9). Applied to all stick axes of every instance. */
    public static void setStickDeadband(double deadband) {
        stickDeadband = Math.max(0.0, Math.min(0.9, deadband));
    }
    /** Current class-level stick deadband. */
    public static double getStickDeadband() { return stickDeadband; }

    /** Set the remap policy (Panels-tunable via {@link #axisRemapMode}). */
    public static void setAxisRemapMode(AxisRemapMode mode) {
        if (mode == null) throw new IllegalArgumentException("mode is null");
        axisRemapMode = mode;
    }
    /**
     * Boolean override kept for quick field fixes: true = {@link AxisRemapMode#ALWAYS},
     * false = {@link AxisRemapMode#NEVER}. Reset to {@link AxisRemapMode#AUTO} via
     * {@link #setAxisRemapMode(AxisRemapMode)} or a Panels edit of {@link #axisRemapMode}.
     */
    public static void setAxisRemapEnabled(boolean enabled) {
        axisRemapMode = enabled ? AxisRemapMode.ALWAYS : AxisRemapMode.NEVER;
    }
    /** Current remap policy. */
    public static AxisRemapMode getAxisRemapMode() { return axisRemapMode; }

    // -------------------------------------------------------------- instance state

    /** The physical Driver Station pad — remapped lanes, type, rumble/LED target. */
    private final Gamepad dsPad;
    /** Panels Gamepad plugin manager (browser pad source) or null = DS-only. */
    private GamepadManager browser = null;

    /** Wrap an SDK gamepad (typically {@code opMode.gamepad1} or {@code gamepad2}). */
    public CommandXboxController(Gamepad gamepad) {
        super(gamepad);
        if (gamepad == null) throw new IllegalArgumentException("gamepad is null");
        this.dsPad = gamepad;
    }

    /**
     * Refresh the merged pad for this loop: buttons/dpad/touchpad come from the
     * Panels Gamepad plugin's browser pad layered over the DS pad; with no
     * browser pad live every lane falls through to the DS values (the plugin's
     * own per-lane rule), so calling this unconditionally is safe. Call once
     * per loop, BEFORE anything polls this controller. Passing null reverts to
     * DS-only.
     */
    public void refreshPad(GamepadManager browserManager) {
        browser = browserManager;
        gp = (browser == null) ? dsPad : browser.asCombinedFTCGamepad(dsPad);
    }

    /** True while a Panels browser pad source is attached (see {@link #refreshPad}). */
    public boolean hasBrowser() { return browser != null; }

    /** The physical DS pad (raw lanes + type + rumble target) — distinct from the merged {@link #getHID()}. */
    public Gamepad getDsHID() { return dsPad; }

    /**
     * Effective remap state for THIS pad right now (AUTO checks the DS-reported
     * type; {@code gamepad.type} is volatile and updates live). Read per access,
     * never baked in at binding-creation time.
     */
    public boolean isRemapActive() {
        switch (axisRemapMode) {
            case ALWAYS: return true;
            case NEVER:  return false;
            default:     return dsPad.type() == Gamepad.Type.XBOX_360;
        }
    }

    // ================================================================== stick axes

    /** Deadbanded left stick X (browser pad when live; raw SDK sign: right positive). */
    @Override public double getLeftX() {
        if (browser != null) {
            double bx = browser.getLeftStickX();
            if (bx != 0.0) return applyDeadband(bx);
        }
        return applyDeadband(dsPad.left_stick_x);
    }
    /** Deadbanded left stick Y (browser pad when live; raw SDK sign: stick-UP negative). */
    @Override public double getLeftY() {
        if (browser != null) {
            double by = browser.getLeftStickY();
            if (by != 0.0) return applyDeadband(by);
        }
        return applyDeadband(dsPad.left_stick_y);
    }

    /**
     * Right stick X (drive rotation) — the Xbox controller's physical right
     * stick: browser pad when live, else the swapped DS lane ({@code left_trigger}).
     */
    @Override public double getRightX() {
        if (browser != null) {
            double bx = browser.getRightStickX();
            if (bx != 0.0) return applyDeadband(bx);
        }
        return applyDeadband(isRemapActive() ? dsPad.left_trigger : dsPad.right_stick_x);
    }
    /**
     * Right stick Y — browser pad when live, else the swapped DS lane
     * ({@code right_trigger}); stick-UP negative, matching the SDK convention.
     */
    @Override public double getRightY() {
        if (browser != null) {
            double by = browser.getRightStickY();
            if (by != 0.0) return applyDeadband(by);
        }
        return applyDeadband(isRemapActive() ? dsPad.right_trigger : dsPad.right_stick_y);
    }

    // ================================================================== analog triggers

    /** True while the left trigger axis exceeds {@link #DEFAULT_TRIGGER_THRESHOLD}. */
    @Override public Trigger leftTrigger() { return leftTrigger(DEFAULT_TRIGGER_THRESHOLD); }
    /** True while the left trigger axis exceeds {@code threshold} (0..1). Lane choice is re-evaluated per read (never baked in at creation). */
    @Override public Trigger leftTrigger(double threshold) {
        return trig(() -> leftTriggerAxisRaw() > threshold);
    }

    /** True while the right trigger axis exceeds {@link #DEFAULT_TRIGGER_THRESHOLD}. */
    @Override public Trigger rightTrigger() { return rightTrigger(DEFAULT_TRIGGER_THRESHOLD); }
    /** True while the right trigger axis exceeds {@code threshold} (0..1). Lane choice is re-evaluated per read (never baked in at creation). */
    @Override public Trigger rightTrigger(double threshold) {
        return trig(() -> rightTriggerAxisRaw() > threshold);
    }

    /** Left trigger axis, normalized to 0..1 (browser {@code l2} when live, else the remapped DS lane). */
    @Override public double getLeftTriggerAxis()  { return leftTriggerAxisRaw(); }
    /** Right trigger axis, normalized to 0..1 (browser {@code r2} when live, else the remapped DS lane). */
    @Override public double getRightTriggerAxis() { return rightTriggerAxisRaw(); }

    private double leftTriggerAxisRaw() {
        if (browser != null) {
            double b = browser.getL2();
            if (b != 0.0) return b;
        }
        return isRemapActive() ? stickToTrigger(dsPad.right_stick_x) : dsPad.left_trigger;
    }
    private double rightTriggerAxisRaw() {
        if (browser != null) {
            double b = browser.getR2();
            if (b != 0.0) return b;
        }
        return isRemapActive() ? stickToTrigger(dsPad.right_stick_y) : dsPad.right_trigger;
    }

    // ================================================================== generic axes (CommandGenericHID parity)

    /** Remapped + deadbanded axis lookup; identical indices to {@link CommandGamepad}. */
    @Override protected double axisValue(int axis) {
        switch (axis) {
            case AXIS_LEFT_X:      return getLeftX();
            case AXIS_LEFT_Y:      return getLeftY();
            case AXIS_RIGHT_X:     return getRightX();
            case AXIS_RIGHT_Y:     return getRightY();
            case AXIS_LEFT_TRIGGER:  return leftTriggerAxisRaw();
            case AXIS_RIGHT_TRIGGER: return rightTriggerAxisRaw();
            default: return super.axisValue(axis); // throws for unknown indices
        }
    }

    // ================================================================== raw (unremapped, undeadbanded) diagnostics

    /** RAW DS {@code left_stick_x} — no remap, no deadband (telemetry checks). */
    public double getRawLeftStickX()  { return dsPad.left_stick_x; }
    /** RAW DS {@code left_stick_y} — no remap, no deadband (telemetry checks). */
    public double getRawLeftStickY()  { return dsPad.left_stick_y; }
    /** RAW DS {@code right_stick_x} — under remap this is the LEFT trigger (rest &lt;= 0, pull +1). */
    public double getRawRightStickX() { return dsPad.right_stick_x; }
    /** RAW DS {@code right_stick_y} — under remap this is the RIGHT trigger (rest &lt;= 0, pull +1). */
    public double getRawRightStickY() { return dsPad.right_stick_y; }
    /** RAW DS {@code left_trigger} — under remap this is the RIGHT stick X (signed -1..1). */
    public double getRawLeftTriggerAxis()  { return dsPad.left_trigger; }
    /** RAW DS {@code right_trigger} — under remap this is the RIGHT stick Y (signed -1..1). */
    public double getRawRightTriggerAxis() { return dsPad.right_trigger; }

    // ================================================================== xbox-specific buttons

    /** Xbox Series X|S Share button (FTC SDK 12: {@code gamepad.share}). */
    public Trigger share()   { return trig(() -> gp.share); }
    /** Options button (PS-name alias of start on PS pads; present for SDK parity). */
    public Trigger options() { return trig(() -> gp.options); }

    /** DS-detected controller type (Xbox controllers report {@code XBOX_360}). */
    public Gamepad.Type getGamepadType() { return dsPad.type(); }
    /** True while the DS has identified this controller as an Xbox pad. */
    public boolean isXbox() { return dsPad.type() == Gamepad.Type.XBOX_360; }

    // ================================================================== rumble + LED (SDK 12)

    /** Single-strength rumble for {@code durationMs} (physical DS pad). */
    public void rumble(int durationMs) { dsPad.rumble(durationMs); }
    /** Rumble left/right motors (0..1 strength) for {@code durationMs} (physical DS pad). */
    public void rumble(double strengthLeft, double strengthRight, int durationMs) {
        dsPad.rumble(strengthLeft, strengthRight, durationMs);
    }
    /** Stop any rumble immediately. */
    public void stopRumble() { dsPad.stopRumble(); }
    /** {@code count} short rumble blips (e.g. countdown / confirmations). */
    public void rumbleBlips(int count) { dsPad.rumbleBlips(count); }
    /** True while a rumble effect is running. */
    public boolean isRumbling() { return dsPad.isRumbling(); }

    /** Set the controller LED color (0..1 rgb) for {@code durationMs}; use {@code Gamepad.LED_DURATION_CONTINUOUS} to hold. */
    public void setLedColor(double r, double g, double b, int durationMs) {
        dsPad.setLedColor(r, g, b, durationMs);
    }
    /** Queue a multi-step rumble effect built with {@code Gamepad.RumbleEffect.Builder}. */
    public void runRumbleEffect(Gamepad.RumbleEffect effect) { dsPad.runRumbleEffect(effect); }
    /** Queue a multi-step LED effect built with {@code Gamepad.LedEffect.Builder}. */
    public void runLedEffect(Gamepad.LedEffect effect) { dsPad.runLedEffect(effect); }

    // ================================================================== internals

    private static Trigger trig(BooleanSupplier condition) { return new Trigger(condition); }

    /**
     * Stick-shaped axis (-1..1, rest anywhere &lt;= 0) -> trigger value 0..1.
     * Robust to both rest conventions seen on Android axis normalization:
     * rest -1 (raw 0..max rescaled) and rest 0. Full pull (+1) maps to 1.
     */
    private static double stickToTrigger(double stickValue) {
        return Math.max(0.0, stickValue);
    }

    /** Deadband with rescale: |v| &lt;= band -> 0, else keeps the full -1..1 range. */
    private static double applyDeadband(double value) {
        double band = stickDeadband;
        if (band <= 0.0) return value;
        double magnitude = Math.abs(value);
        if (magnitude <= band) return 0.0;
        double sign = Math.signum(value);
        return sign * (magnitude - band) / (1.0 - band);
    }
}
