package org.firstinspires.ftc.teamcode.WattageLib.PanelsTelemLogging;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Extracts the actual telemetry lines out of FTControl Panels telemetry.
 *
 * <p>Why this exists: {@code PanelsTelemetry.INSTANCE.getFtcTelemetry()}
 * returns a {@code TelemetryManager$TelemetryWrapper}. That wrapper holds NO
 * data itself — every {@code addData()} forwards to the outer
 * {@code TelemetryManager}, which appends a pre-formatted {@code "key: value"}
 * string to its private {@code List<String> lines}. So:
 * <ul>
 *   <li>{@code wrapper.toString()} is just {@code
 *       TelemetryManager$TelemetryWrapper@88dc098} — useless in a log.</li>
 *   <li>Looking for {@code lines}/{@code data}/{@code map} fields <b>on the
 *       wrapper</b> always fails — the list lives on {@code wrapper.this$0}.</li>
 * </ul>
 *
 * <p>Correct extraction: {@code wrapper.this$0.getLines()} (copied, because
 * {@code TelemetryManager.update()} <b>clears</b> the list on every call —
 * even throttled ones — so callers must snapshot <b>before</b>
 * {@code DashTelemetry.update()}).
 */
public class TelemetrySnapshot {
    private static final String TAG = "TelemetrySnapshot";

    /**
     * Returns the current pending lines as one {@code " | "}-joined string.
     * Returns {@code "(no telemetry lines)"} when the buffer is empty rather
     * than a useless object pointer, so empty frames are distinguishable from
     * extraction failures in the log.
     */
    public static String captureDetail(Telemetry telemetry) {
        List<String> lines = extractLines(telemetry);
        if (lines == null) {
            return telemetry.toString();
        }
        if (lines.isEmpty()) {
            return "(no telemetry lines)";
        }
        StringBuilder sb = new StringBuilder();
        for (String line : lines) {
            if (sb.length() > 0) sb.append(" | ");
            sb.append(line);
        }
        return sb.toString();
    }

    /**
     * Returns a defensive copy of the pending Panels lines, or null if the
     * telemetry object is not a Panels wrapper / reflection failed.
     *
     * <p>Must be called BEFORE {@code telemetry.update()} — update() clears
     * the buffer every tick.
     */
    public static List<String> extractLines(Telemetry telemetry) {
        if (telemetry == null) return null;
        try {
            Object manager = unwrapManager(telemetry);
            if (manager == null) return null;
            Method getLines = manager.getClass().getMethod("getLines");
            Object raw = getLines.invoke(manager);
            if (raw instanceof List) {
                // Defensive copy: update() clears the live list.
                List<?> list = (List<?>) raw;
                List<String> copy = new ArrayList<>(list.size());
                for (Object o : list) copy.add(String.valueOf(o));
                return copy;
            }
        } catch (Exception e) {
            Log.w(TAG, "Failed to extract telemetry lines: " + e.getMessage());
        }
        return null;
    }

    /**
     * Extracts individual lines into an indexed map (line0, line1, ...) for
     * callers that want per-line control.
     */
    public static Map<String, String> extractAllData(Telemetry telemetry) {
        Map<String, String> result = new LinkedHashMap<>();
        List<String> lines = extractLines(telemetry);
        if (lines != null) {
            int i = 0;
            for (String line : lines) {
                result.put("line" + i, line);
                i++;
            }
        }
        return result;
    }

    /**
     * The Panels wrapper is an inner class: its only state besides separators
     * is the synthetic {@code this$0} reference to the outer
     * {@code TelemetryManager}. That manager owns {@code getLines()}.
     */
    private static Object unwrapManager(Telemetry telemetry) throws Exception {
        Class<?> cls = telemetry.getClass();
        // Fast path: known Panels wrapper class name.
        if (cls.getName().contains("TelemetryManager")) {
            try {
                Field outer = cls.getDeclaredField("this$0");
                outer.setAccessible(true);
                Object manager = outer.get(telemetry);
                if (manager != null) return manager;
            } catch (NoSuchFieldException ignored) {
                // Fall through to interface scan below.
            }
        }
        // Generic fallback: any field whose type name contains TelemetryManager.
        for (Field f : cls.getDeclaredFields()) {
            if (f.getType().getName().contains("TelemetryManager")) {
                f.setAccessible(true);
                Object manager = f.get(telemetry);
                if (manager != null) return manager;
            }
        }
        // Last resort: the object itself might be the manager.
        try {
            cls.getMethod("getLines");
            return telemetry;
        } catch (NoSuchMethodException ignored) {
        }
        return null;
    }
}
