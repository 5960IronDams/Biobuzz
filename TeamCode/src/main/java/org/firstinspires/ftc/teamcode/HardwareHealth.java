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
 * Driver Station hardware health and warning board.
 *
 * <p>Reports critical faults (e.g., TURRET, LIMELIGHT, RAMP) via DS warnings
 * and {@link #dsLine()} without locking or stopping OpModes. Required systems
 * register with {@link #expect(String)} and heartbeat with {@link #ok(String)}
 * or {@link #report(String, String)}.
 */
public final class HardwareHealth {
    public static final String TAG = "HardwareHealth";

    /** Prefix added to Driver Station warning messages. */
    public static final String WARNING_PREFIX = "Iron HW: ";

    /** Milliseconds without a healthy heartbeat while running before flagging "no data". */
    public static double HEARTBEAT_STALE_MS = 2000.0;

    /** Maximum length of the DS telemetry line before truncating. */
    private static final int DS_LINE_MAX = 55;
    /** Maximum length of the DS warning text before truncating. */
    private static final int WARNING_MAX_CHARS = 160;

    private static final HardwareHealth INSTANCE = new HardwareHealth();

    // Strong static reference required because RobotLog stores sources in a WeakHashMap.
    private static final HealthWarningSource WARNING_SOURCE = new HealthWarningSource();

    static {
        try {
            RobotLog.registerGlobalWarningSource(WARNING_SOURCE);
        } catch (Exception e) {
            Log.w(TAG, "could not register the DS warning source", e);
        }
    }

    public static HardwareHealth get() {
        return INSTANCE;
    }

    private static final class HealthWarningSource implements GlobalWarningSource {
        private volatile String warning = "";
        private volatile boolean suppressed = false;

        @Override
        public String getGlobalWarning() {
            return suppressed ? "" : warning;
        }

        @Override
        public boolean shouldTriggerWarningSound() {
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
        String fault;

        Entry(String name, boolean required, long now) {
            this.name = name;
            this.required = required;
            this.lastOkMs = now;
        }
    }

    private final Map<String, Entry> systems = new LinkedHashMap<>();
    private final List<String> faults = new ArrayList<>();
    private String lastWarning = "";
    private boolean started = false;
    private double loopTimeMs = 0.0;
    private String lastDsLine = null;
    private HardwareHealth() {
    }

    /** Clears all health entries and DS warnings for a new OpMode. */
    public void reset() {
        systems.clear();
        faults.clear();
        started = false;
        lastWarning = "";
        loopTimeMs = 0.0;
        lastDsLine = null;
        WARNING_SOURCE.clearGlobalWarning();
    }

    /** Marks a system as required so missing heartbeats trigger a "no data" warning. */
    public void expect(String name) {
        Entry e = systems.get(name);
        if (e == null) {
            systems.put(name, new Entry(name, true, nowMs()));
        } else {
            e.required = true;
        }
    }

    /** Reports a healthy heartbeat for the given system. */
    public void ok(String name) {
        report(name, null);
    }

    /** Reports a system status: null clears any fault, non-null sets a short fault message. */
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

    /** Rebuilds the fault list and updates the DS warning text (called once per tick). */
    public void evaluate(boolean startedNow) {
        long now = nowMs();
        boolean startEdge = startedNow && !started;
        started = startedNow;
        lastDsLine = null;

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
                e.lastOkMs = now;
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

    /** Updates the loop period (ms) shown on the DS telemetry line. */
    public void setLoopStats(double loopTimeMs) {
        if (Math.abs(loopTimeMs - this.loopTimeMs) < 1e-9) {
            return;
        }
        this.loopTimeMs = loopTimeMs;
        lastDsLine = null;
    }

    /** Returns the formatted DS status line (health + loop rate). */
    public String dsLine() {
        String line = lastDsLine;
        if (line == null) {
            line = buildDsLine();
            lastDsLine = line;
        }
        return line;
    }

    private String buildDsLine() {
        String head;
        if (faults.isEmpty()) {
            head = "OK";
        } else {
            head = faults.get(0);
            if (faults.size() > 1) {
                head += " (+" + (faults.size() - 1) + " more)";
            }
        }
        String rate = loopSuffix(loopTimeMs);
        int budget = Math.max(4, DS_LINE_MAX - 3 - rate.length());
        if (head.length() > budget) {
            head = head.substring(0, budget - 3) + "...";
        }
        return head + " | " + rate;
    }

    private static String loopSuffix(double loopTimeMs) {
        if (loopTimeMs <= 0.0 || !Double.isFinite(loopTimeMs)) {
            return "loop -- ms";
        }
        return String.format(Locale.US, "%.2f ms %.1f Hz", loopTimeMs, 1000.0 / loopTimeMs);
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
