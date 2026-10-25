package org.firstinspires.ftc.teamcode.WattageLib.lib;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotMain;

/**
 * Panels gamepad debug view — a close mimic of the official Panels Gamepad
 * example. {@code PanelsGamepad.firstManager.asCombinedFTCGamepad(gamepad)}
 * merges the Panels-page browser pad over the physical DS pad, and the dump
 * shows the standard FTC {@link Gamepad} fields only (buttons + sticks), in the
 * example's structure and format:
 *
 * <pre>
 * GP1 ==== Buttons ====
 * GP1 A: true
 * ...
 * GP1 ==== Sticks ====
 * GP1 Left Stick X: 0.0
 * ...
 * </pre>
 *
 * <p>Only the {@code tag} prefix (so two pads' dumps can be told apart) and the
 * absence of an {@code update()} call differ from the example: in this codebase
 * {@link RobotMain#flushTelemetry()} is the single Panels flush —
 * {@code debug()} only appends to the same buffer the wrapper sends (a second
 * update in one tick would send a partial frame — see flushTelemetry()'s
 * javadoc).
 *
 * <p>Usage:
 * <pre>
 * gamepadDebug = new PanelsGamepadDebug(gamepad1, "GP1");  // init()
 * gamepadDebug.report();                                   // per loop
 * </pre>
 *
 * <p>Toggle the view from Panels &gt; Configurables &gt; PanelsGamepadDebug.
 */
@Configurable
public class PanelsGamepadDebug {

    /** Master switch (Panels &gt; Configurables &gt; PanelsGamepadDebug). */
    public static boolean DEBUG = false;

    private final Gamepad physical;
    private final String tag;

    /**
     * @param gamepad the physical SDK gamepad to inspect ({@code gamepad1}/
     *                {@code gamepad2}); the Panels-page browser pad is layered
     *                over it per read, exactly like the example
     * @param tag     line prefix, e.g. {@code "GP1"} / {@code "GP2"}
     */
    public PanelsGamepadDebug(Gamepad gamepad, String tag) {
        if (gamepad == null) throw new IllegalArgumentException("gamepad is null");
        this.physical = gamepad;
        this.tag = tag;
    }

    /**
     * Add the debug lines for this tick. Emission ONLY (debug) — the single
     * {@link RobotMain#flushTelemetry()} call at the end of the loop sends them.
     */
    public void report() {
        if (!DEBUG || !RobotMain.DASHBOARD) return;

        // Official example pattern: combine the Panels-page (browser) gamepad
        // with the physical DS pad. When no browser pad is touched, the
        // combined values fall through to the physical pad — identical view.
        Gamepad g = PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(physical);
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        panelsTelemetry.debug(
                tag + " ==== Buttons ====",
                tag + " A: " + g.a,
                tag + " B: " + g.b,
                tag + " X: " + g.x,
                tag + " Y: " + g.y,
                tag + " DPad Up: " + g.dpad_up,
                tag + " DPad Down: " + g.dpad_down,
                tag + " DPad Left: " + g.dpad_left,
                tag + " DPad Right: " + g.dpad_right,
                tag + " Left Bumper: " + g.left_bumper,
                tag + " Right Bumper: " + g.right_bumper,
                tag + " Left Trigger: " + g.left_trigger,
                tag + " Right Trigger: " + g.right_trigger,
                tag + " Start / Options: " + g.options,
                tag + " Back / Share: " + g.back,
                tag + " Guide / PS: " + g.guide,
                tag + " Touchpad: " + g.touchpad,
                tag + " Left Stick Button: " + g.left_stick_button,
                tag + " Right Stick Button: " + g.right_stick_button);

        panelsTelemetry.debug(
                tag + " ==== Sticks ====",
                tag + " Left Stick X: " + g.left_stick_x,
                tag + " Left Stick Y: " + g.left_stick_y,
                tag + " Right Stick X: " + g.right_stick_x,
                tag + " Right Stick Y: " + g.right_stick_y);
    }
}
