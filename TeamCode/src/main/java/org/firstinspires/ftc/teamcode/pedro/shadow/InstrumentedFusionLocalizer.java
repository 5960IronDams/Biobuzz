package org.firstinspires.ftc.teamcode.pedro.shadow;

import android.util.Log;

import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.RobotMain;

import java.lang.reflect.Field;
import java.util.Locale;
import java.util.Map;
import java.util.NavigableMap;

/**
 * Drop-in replacement for Pedro's {@link FusionLocalizer}: IDENTICAL math
 * (subclass — no behavior change), plus per-call instrumentation to catch the
 * one-time heading injection documented in the bug report (runs 23-04 through
 * 00-23: fusedH jumped ~138-206deg inside a single addMeasurement on an
 * unmoved robot while the raw dead-reckoning heading held 90.00 on every loop).
 *
 * <p>What it does: every {@link #addMeasurement} call snapshots the filter's
 * history (per-entry pose + relativeTransform headings), calls super, then
 * diffs. Any heading change > 2deg is dumped (Log.i + Dashboard keys
 * {@code Fuse/injDh}, {@code Fuse/injBefore}, {@code Fuse/injAfter}) — the
 * before/after diff shows exactly which history entries the injection touched.
 *
 * <p>Wiring: RobotMain reflects the stock localizer's parameters (dead
 * reckoning, P/Q/R diagonals, bufferSize), constructs this with them, and
 * swaps it into {@code Follower.localizer} via reflection (the field is
 * {@code public final}, but an instance-final reflective write works on ART).
 * The subsequent Pinpoint-pose seeding then seeds THIS instance unchanged.
 */
public class InstrumentedFusionLocalizer extends FusionLocalizer {

    private static final Field HISTORY_FIELD;
    private static final Field KS_POSE_FIELD;
    private static final Field KS_RT_FIELD;
    static {
        Field h = null, p = null, rt = null;
        try {
            h = FusionLocalizer.class.getDeclaredField("history");
            h.setAccessible(true);
            Class<?> ks = Class.forName("com.pedropathing.localization.FusionLocalizer$KalmanState");
            p = ks.getDeclaredField("pose");
            p.setAccessible(true);
            rt = ks.getDeclaredField("relativeTransform");
            rt.setAccessible(true);
        } catch (Throwable t) {
            Log.w("IronLog", "InstrumentedFusionLocalizer reflection init failed", t);
        }
        HISTORY_FIELD = h;
        KS_POSE_FIELD = p;
        KS_RT_FIELD = rt;
    }

    /** Signed heading change of the last addMeasurement call, deg. */
    public double lastDhDeg = 0.0;
    private String lastInjectionDump = null;

    public InstrumentedFusionLocalizer(Localizer deadReckoning, Pose initialCovariance,
                                       Pose processVariance, Pose measurementVariance,
                                       int bufferSize) {
        super(deadReckoning, initialCovariance, processVariance, measurementVariance, bufferSize);
    }

    @Override
    public void addMeasurement(Pose measuredPose, long timestamp) {
        addMeasurement(measuredPose, timestamp, null);
    }

    @Override
    public void addMeasurement(Pose measuredPose, long timestamp, Pose measurementVariance) {
        String before = snapshot();
        double preH = state().pose().heading();
        super.addMeasurement(measuredPose, timestamp, measurementVariance);
        double postH = state().pose().heading();
        lastDhDeg = Math.toDegrees(Angle.normalizeSigned(postH - preH));
        if (Math.abs(lastDhDeg) > 2.0) {
            String after = snapshot();
            lastInjectionDump = "meas=" + measuredPose + " ts=" + timestamp
                    + "\npreH=" + String.format(Locale.US, "%.2f", Math.toDegrees(preH))
                    + " postH=" + String.format(Locale.US, "%.2f", Math.toDegrees(postH))
                    + " dh=" + String.format(Locale.US, "%.2f", lastDhDeg)
                    + "\nBEFORE: " + before
                    + "\nAFTER:  " + after;
            Log.i("IronLog-Fuse", "HEADING INJECTION\n" + lastInjectionDump);
            RobotMain.DashTelemetry.addData("Fuse/injDh",
                    String.format(Locale.US, "%.1f", lastDhDeg));
            RobotMain.DashTelemetry.addData("Fuse/injBefore", before);
            RobotMain.DashTelemetry.addData("Fuse/injAfter", after);
        }
    }

    /** Full dump of the last detected injection (for FusionTune / offline analysis). */
    public String lastInjectionDump() {
        return lastInjectionDump;
    }

    /**
     * Compact history snapshot, newest 24 entries:
     * {@code t=<ns> h=(x,y,deg) rtH=deg | ...}. The rtH column is each entry's
     * stored relativeTransform heading — for a stationary robot these are ~0;
     * anything large in the BEFORE snapshot is the injection's carrier.
     */
    private String snapshot() {
        if (HISTORY_FIELD == null || KS_POSE_FIELD == null || KS_RT_FIELD == null) {
            return "(no history access)";
        }
        try {
            NavigableMap<?, ?> h = (NavigableMap<?, ?>) HISTORY_FIELD.get(this);
            Object[] entries = h.descendingMap().entrySet().toArray();
            StringBuilder sb = new StringBuilder();
            int n = Math.min(24, entries.length);
            for (int i = 0; i < n; i++) {
                Map.Entry<?, ?> e = (Map.Entry<?, ?>) entries[i];
                Object ks = e.getValue();
                Pose p = (Pose) KS_POSE_FIELD.get(ks);
                Pose rt = (Pose) KS_RT_FIELD.get(ks);
                sb.append("t=").append(e.getKey())
                        .append(" h=").append(p == null ? "?"
                                : String.format(Locale.US, "(%.1f,%.1f,%.1f)", p.x(), p.y(),
                                        Math.toDegrees(Angle.normalize(p.heading()))))
                        .append(" rtH=").append(rt == null ? "?"
                                : String.format(Locale.US, "%.1f",
                                        Math.toDegrees(Angle.normalizeSigned(rt.heading()))))
                        .append(i == n - 1 ? "" : " | ");
            }
            return sb.toString();
        } catch (Throwable t) {
            return "(snapshot failed: " + t + ")";
        }
    }
}
