package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * HardwareHealth — the game-day failure board for the DRIVER STATION.
 *
 * <p><b>Why this exists:</b> on game day the driver has only the DS phone —
 * the Panels laptop lives in the pits. When hardware dies mid-match (turret
 * encoders, the OctoQuad, the Limelight), the driver must SEE it immediately,
 * while the full per-loop diagnostics stay on Panels for post-match
 * debugging. So the bar for getting on this board is deliberately high:
 * only failures that make aiming or feedback quietly wrong.
 *
 * <p><b>NEVER fatal — this is the rule that matters most.</b> Nothing here
 * may ever stop the robot from playing: being unable to run an autonomous or
 * a TeleOp in a match is worse than any hardware fault. So this board
 * deliberately does NOT use {@code RobotLog.setGlobalErrorMsg(...)} — a
 * global ERROR makes the Driver Station demand "To attempt recovery, please
 * restart the robot" and refuses to init/start OpModes until the robot is
 * restarted (that is exactly what an absent Limelight did once). It reports
 * through channels that are purely informational:
 * <ol>
 *   <li><b>The DS warning line</b> — a registered
 *       {@link RobotLog.GlobalWarningSource}. {@code RobotLog} merges every
 *       registered source's text with the SDK's own warnings each cycle
 *       ({@code getGlobalWarningMessage()} re-queries all sources), so this is
 *       additive — it never clobbers the SDK's I2C/battery/robocol warnings,
 *       and warnings never lock the robot. The text carries the
 *       {@link #WARNING_PREFIX} tag so it is recognizable in the merged blob.
 *       (The SDK stores sources in a <em>WeakHashMap</em>, hence the static
 *       strong reference to our source below.)</li>
 *   <li><b>The DS telemetry line</b> ("HW") — each OpMode's
 *       {@code UpdateTelemetry()} pulls {@link #dsLine()} into its OWN DS
 *       update flow. The DS screen shows only what the last
 *       {@code telemetry.update()} contained, so the board never calls
 *       update() itself — two updaters on one DS telemetry would clobber
 *       each other's lines (AutoOpModeBase already updates per tick).</li>
 *   <li><b>The match log</b> — an edge-triggered {@code Log.w} per change, so
 *       the pit laptop can reconstruct the timeline after the match.</li>
 * </ol>
 *
 * <p><b>What counts as critical (the short list):</b>
 * <ul>
 *   <li><b>TURRET</b> — any latched encoder fault from
 *       {@code TurretPivot.faultString()}: CRT crosscheck, motor-encoder
 *       dead/direction, boot-dead encoder, absolute (OctoQuad/encoder-B)
 *       stream lost, stuck watchdog... (reported every loop from
 *       {@code TurretPivot.periodic()}; a failed {@code tryCreate} reports
 *       "absent at init" on the comp bot).</li>
 *   <li><b>LIMELIGHT</b> — frames gone stale or the camera silent
 *       ({@code Vision.healthPing()}, called every tick from RobotMain,
 *       independent of the VisionFusion gates so a disabled filter or a
 *       diverged pose can't starve the liveness check). "No tag solve" is
 *       gameplay, NOT a hardware failure — it does not report.</li>
 * </ul>
 *
 * <p><b>Heartbeats:</b> a system registers via {@link #expect} (making it
 * REQUIRED) and must call {@link #ok} at least every
 * {@link #HEARTBEAT_STALE_MS} while the OpMode is RUNNING, or the board flags
 * "no data". Silence windows only count after PLAY (init-time reporting is
 * sporadic by design — the turret's scheduler only runs once the match
 * starts) and restart at the PLAY edge. Direct faults ({@link #report}) show
 * immediately, even during init.
 *
 * <p>All calls happen on the RC loop thread — no locking. State is wiped by
 * {@link RobotMain}'s constructor at every OpMode init; the last warning text
 * survives across a match on purpose (the driver walks off the field still
 * seeing what failed) and is cleared by the NEXT OpMode's reset.
 */
public final class HardwareHealth {
    public static final String TAG = "HardwareHealth";

    /**
     * Tag on every warning we publish, so our text is recognizable inside the
     * DS's merged warning blob (which also carries SDK warnings). Nothing
     * parses it, so changing the wording is safe.
     */
    public static final String WARNING_PREFIX = "Iron HW: ";

    /**
     * Silence window: a REQUIRED system that has not reported OK for this
     * long WHILE THE OPMODE IS RUNNING is flagged "no data". 2 s rides out a
     * single I2C hiccup / slow PWM refresh, but catches a dead device within
     * a couple of seconds of a match.
     */
    public static double HEARTBEAT_STALE_MS = 2000.0;

    /** Longest DS telemetry line before truncation (the DS phone fits ~50 characters). */
    private static final int DS_LINE_MAX = 55;
    /** Longest warning text before truncation (it shares the DS warning area). */
    private static final int WARNING_MAX_CHARS = 160;

    private static final HardwareHealth INSTANCE = new HardwareHealth();

    /**
     * Our registered {@link GlobalWarningSource}. Held in a static final field
     * because {@code RobotLog} stores sources in a WeakHashMap — without this
     * strong reference the source would be collected and the DS warning would
     * silently disappear.
     */
    private static final HealthWarningSource WARNING_SOURCE = new HealthWarningSource();

    static {
        try {
            RobotLog.registerGlobalWarningSource(WARNING_SOURCE);
        } catch (Exception e) {
            Log.w(TAG, "could not register the DS warning source", e);
        }
    }

    /**
     * Singleton: every subsystem reports to the same board without threading
     * references through constructors. All entry points run on the RC loop
     * thread.
     */
    public static HardwareHealth get() {
        return INSTANCE;
    }

    /**
     * The DS-facing warning text provider. {@code RobotLog} calls
     * {@link #getGlobalWarning()} while aggregating the warning line, so the
     * text stays live without us pushing anything, and coexists with the
     * SDK's own warnings (additive, never clobbering). Advisory only — the
     * warning channel never blocks OpMode init/start.
     */
    private static final class HealthWarningSource implements GlobalWarningSource {
        /** Text we publish; empty string = nothing to say (the SDK skips empties). */
        private volatile String warning = "";
        /** Set by the SDK when the user suppresses warnings on the DS. */
        private volatile boolean suppressed = false;

        @Override
        public String getGlobalWarning() {
            return suppressed ? "" : warning;
        }

        @Override
        public boolean shouldTriggerWarningSound() {
            // Deliberately quiet: the SDK asks on every aggregation and a
            // repeating beep would be worse than useless mid-match. The text
            // on the DS (plus the telemetry line) is the signal.
            return false;
        }

        @Override
        public void suppressGlobalWarning(boolean suppress) {
            suppressed = suppress;
        }

        @Override
        public void setGlobalWarning(String text) {
            warning = text == null ? "" : text;
        }

        @Override
        public void clearGlobalWarning() {
            warning = "";
        }
    }

    private static final class Entry {
        final String name;
        boolean required;
        long lastOkMs;
        /** null = healthy; anything else is the current fault detail. */
        String fault;

        Entry(String name, boolean required, long now) {
            this.name = name;
            this.required = required;
            this.lastOkMs = now;
        }
    }

    private final Map<String, Entry> systems = new LinkedHashMap<>();
    /** Reusable fault-line buffer, rebuilt by each evaluate() call. */
    private final List<String> faults = new ArrayList<>();
    /** Last warning text we published (edge-triggered logging). */
    private String lastWarning = "";
    /** Tracks the OpMode PLAY edge so silence windows restart when the match begins. */
    private boolean started = false;

    private HardwareHealth() {
    }

    /**
     * Wipe all state at OpMode construction — a fresh OpMode starts with a
     * clean board and no leftover warning text. Never touches the SDK's own
     * global error/warning state.
     */
    public void reset() {
        systems.clear();
        faults.clear();
        started = false;
        lastWarning = "";
        WARNING_SOURCE.clearGlobalWarning();
    }

    /**
     * Register a REQUIRED system: it must heartbeat via {@link #ok} while the
     * OpMode runs or the board flags "no data". Call once at construction
     * (RobotMain), not per tick.
     */
    public void expect(String name) {
        Entry e = systems.get(name);
        if (e == null) {
            systems.put(name, new Entry(name, true, nowMs()));
        } else {
            e.required = true;
        }
    }

    /** Healthy heartbeat for a system (advances its silence window). */
    public void ok(String name) {
        report(name, null);
    }

    /**
     * One report per system per tick: {@code faultDetail == null} means
     * healthy (heartbeat); anything else is a fault shown on the DS until the
     * system reports healthy again. Keep details short and driver-readable
     * ("CRT_CROSSCHECK MOTOR_ENC"), not per-loop dumps — full detail stays on
     * the Panels dashboard.
     */
    public void report(String name, String faultDetail) {
        long now = nowMs();
        Entry e = systems.get(name);
        if (e == null) {
            e = new Entry(name, false, now);
            systems.put(name, e);
        }
        if (faultDetail == null) {
            e.fault = null;
            e.lastOkMs = now;
        } else {
            e.fault = faultDetail;
        }
    }

    /**
     * One call per OpMode tick, from {@code RobotMain.flushTelemetry()} —
     * BEFORE the dashboard-off early-return, so the board keeps running in
     * the lean match loop. Rebuilds the fault list and publishes (or clears)
     * the DS warning text. Never touches the fatal global-error channel; see
     * the class javadoc.
     *
     * @param startedNow true once the OpMode has passed PLAY (silence windows
     *                   only run while started; direct faults show either way)
     */
    public void evaluate(boolean startedNow) {
        long now = nowMs();
        boolean startEdge = startedNow && !started;
        started = startedNow;

        faults.clear();
        for (Entry e : systems.values()) {
            if (e.fault != null) {
                faults.add(e.name + " " + e.fault);
                continue;
            }
            if (!e.required) {
                continue;
            }
            if (startEdge) {
                e.lastOkMs = now; // restart silence windows at PLAY
            }
            double silentMs = now - e.lastOkMs;
            if (silentMs > HEARTBEAT_STALE_MS) {
                faults.add(String.format(Locale.US, "%s no data %.1fs", e.name, silentMs / 1000.0));
            }
        }

        String warning = "";
        if (!faults.isEmpty()) {
            StringBuilder sb = new StringBuilder(WARNING_PREFIX);
            for (String f : faults) {
                if (sb.length() > WARNING_PREFIX.length()) {
                    sb.append(" | ");
                }
                sb.append(f);
            }
            warning = sb.toString();
            if (warning.length() > WARNING_MAX_CHARS) {
                warning = warning.substring(0, WARNING_MAX_CHARS - 3) + "...";
            }
        }

        // Edge-triggered: log and publish only when the text changes, so a
        // persisting fault costs nothing per loop.
        if (!warning.equals(lastWarning)) {
            if (warning.isEmpty()) {
                Log.i(TAG, "hardware healthy — DS warning cleared");
            } else {
                Log.w(TAG, warning);
            }
            lastWarning = warning;
            if (warning.isEmpty()) {
                WARNING_SOURCE.clearGlobalWarning();
            } else {
                WARNING_SOURCE.setGlobalWarning(warning);
            }
        }
    }

    /**
     * The one driver-facing line for the DS telemetry screen ("OK", or the
     * first fault plus a count of the rest) — pulled by each OpMode's
     * UpdateTelemetry() into its OWN DS update flow (see the evaluate()
     * javadoc for why this class never calls telemetry.update() itself).
     * Truncated to fit the DS phone.
     */
    public String dsLine() {
        if (faults.isEmpty()) {
            return "OK";
        }
        String line = faults.get(0);
        if (faults.size() > 1) {
            line += " (+" + (faults.size() - 1) + " more)";
        }
        if (line.length() > DS_LINE_MAX) {
            line = line.substring(0, DS_LINE_MAX - 3) + "...";
        }
        return line;
    }

    /** Compact one-line summary for the Panels dashboard: "OK" or the full fault list. */
    public String summary() {
        if (faults.isEmpty()) {
            return "OK";
        }
        StringBuilder sb = new StringBuilder();
        for (String f : faults) {
            if (sb.length() > 0) {
                sb.append(" | ");
            }
            sb.append(f);
        }
        return sb.toString();
    }

    /** True when at least one system is failed right now (valid after evaluate()). */
    public boolean anyFault() {
        return !faults.isEmpty();
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
