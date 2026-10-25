package org.firstinspires.ftc.teamcode;

import android.util.Log;

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
 * <p><b>Two channels, once per tick:</b>
 * <ol>
 *   <li><b>The DS telemetry line</b> ("HW") — each OpMode's
 *       {@code UpdateTelemetry()} pulls {@link #dsLine()} into its OWN DS
 *       update flow. The DS screen shows only what the last
 *       {@code telemetry.update()} contained, so the board never calls
 *       update() itself — two updaters on one DS telemetry would clobber
 *       each other's lines (AutoOpModeBase already updates per tick).</li>
 *   <li><b>The red DS error banner</b> — {@code RobotLog.setGlobalErrorMsg}
 *       while any system is failed, {@code clearGlobalErrorMsg} once healthy.
 *       Managed by {@link #evaluate(boolean)} from
 *       {@code RobotMain.flushTelemetry()}, edge-triggered (set/clear only on
 *       CHANGE — no per-tick log spam). Every banner we write starts with
 *       {@link #BANNER_PREFIX}; an SDK-authored global error (init hardware
 *       failures etc.) is NEVER clobbered or wiped — we manage only our own
 *       prefixed messages.</li>
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
 *       gameplay, NOT a hardware failure — it does not alarm.</li>
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
 * {@link RobotMain}'s constructor at every OpMode init; the banner survives
 * across a match on purpose (the driver walks off the field still seeing
 * what failed) and is cleared by the NEXT OpMode's reset.
 */
public final class HardwareHealth {
    public static final String TAG = "HardwareHealth";

    /**
     * Every banner we write starts with this. It is how we tell our own
     * global-error messages apart from the SDK's (init failures, I2C
     * warnings) so we never clobber or clear a message we didn't write.
     */
    public static final String BANNER_PREFIX = "IRON HW:";

    /**
     * Silence window: a REQUIRED system that has not reported OK for this
     * long WHILE THE OPMODE IS RUNNING is flagged "no data". 2 s rides out a
     * single I2C hiccup / slow PWM refresh, but catches a dead device within
     * a couple of seconds of a match.
     */
    public static double HEARTBEAT_STALE_MS = 2000.0;

    /** Longest DS line before truncation (the DS phone fits ~50 characters). */
    private static final int DS_LINE_MAX = 55;
    /** Longest banner before truncation (it overlays the DS telemetry). */
    private static final int BANNER_MAX_CHARS = 200;

    private static final HardwareHealth INSTANCE = new HardwareHealth();

    /**
     * Singleton: every subsystem reports to the same board without threading
     * references through constructors. All entry points run on the RC loop
     * thread.
     */
    public static HardwareHealth get() {
        return INSTANCE;
    }

    private static final class Entry {
        final String name;
        boolean required;
        long lastOkMs;
        /** null = healthy; anything else is the current CRITICAL fault detail. */
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
    /** The banner text we currently own in the global-error slot (null = none of ours showing). */
    private String banner = null;
    /** Tracks the OpMode PLAY edge so silence windows restart when the match begins. */
    private boolean started = false;

    private HardwareHealth() {
    }

    /**
     * Wipe all state at OpMode construction — a fresh OpMode starts with a
     * clean board and clears any banner WE own in the global-error slot
     * (an SDK-authored error, if present, is left alone).
     */
    public void reset() {
        systems.clear();
        faults.clear();
        started = false;
        clearOurBannerIfSafe("reset");
        banner = null;
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
     * healthy (heartbeat); anything else is a CRITICAL fault shown on the DS
     * until the system reports healthy again. Keep details short and
     * driver-readable ("CRT_CROSSCHECK MOTOR_ENC"), not per-loop dumps —
     * full detail stays on the Panels dashboard.
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
     * the lean match loop. Rebuilds the fault list and manages the red
     * banner. This does NOT write the DS telemetry screen: each OpMode's
     * {@code UpdateTelemetry()} owns its own DS screen and pulls the line
     * from {@link #dsLine()} (two update() calls on one DS telemetry would
     * clobber the earlier one's lines — last update wins).
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

        if (faults.isEmpty()) {
            clearOurBannerIfSafe("healthy");
        } else {
            StringBuilder sb = new StringBuilder(BANNER_PREFIX);
            for (String f : faults) {
                if (sb.length() > BANNER_PREFIX.length()) {
                    sb.append(" | ");
                }
                sb.append(f);
            }
            String msg = sb.toString();
            if (msg.length() > BANNER_MAX_CHARS) {
                msg = msg.substring(0, BANNER_MAX_CHARS - 3) + "...";
            }
            showBanner(msg);
        }
    }

    /**
     * Set the red DS banner (edge-triggered — only on CHANGE). If the SDK has
     * posted its own global error over ours, leave the slot alone and mark
     * ours as not-showing so we re-assert the moment the slot frees up.
     */
    private void showBanner(String msg) {
        String current = currentGlobalError();
        if (current != null && !current.isEmpty() && !current.startsWith(BANNER_PREFIX)) {
            banner = null;
            return;
        }
        if (msg.equals(banner)) {
            return; // unchanged — no per-tick log spam / re-set
        }
        Log.e(TAG, msg);
        RobotLog.setGlobalErrorMsg(msg);
        banner = msg;
    }

    /**
     * Clear the global-error slot ONLY when the current message is empty or
     * one of ours ({@link #BANNER_PREFIX}) — never wipe an SDK-authored
     * error (that would hide a real init failure from the driver).
     */
    private void clearOurBannerIfSafe(String why) {
        String current = currentGlobalError();
        boolean foreign = current != null && !current.isEmpty() && !current.startsWith(BANNER_PREFIX);
        if (!foreign) {
            if (banner != null || (current != null && !current.isEmpty())) {
                RobotLog.clearGlobalErrorMsg();
                Log.i(TAG, "DS banner cleared (" + why + ")");
            }
        }
        banner = null;
    }

    private static String currentGlobalError() {
        try {
            return RobotLog.getGlobalErrorMsg();
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * The one driver-facing line for the DS screen ("OK", or the first fault
     * plus a count of the rest) — pulled by each OpMode's UpdateTelemetry()
     * into its OWN DS update flow (see the evaluate() javadoc for why this
     * class never calls telemetry.update() itself). Truncated to fit the DS
     * phone.
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
