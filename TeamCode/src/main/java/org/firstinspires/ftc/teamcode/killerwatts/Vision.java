package org.firstinspires.ftc.teamcode.killerwatts;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedro.PedroFieldBridge;

import java.util.List;

/**
 * Limelight3A MegaTag pose source: MT2-primary with MT1 fallback.
 *
 * <p>Each loop the caller seeds the Limelight with the current field-relative yaw
 * (from the fused Pedro pose) and reads back the field-space botpose. MT2
 * ({@code getBotpose_MT2()}) is preferred because it is gyro-seeded and stable
 * while moving; when MT2 is unavailable the MT1 solve ({@code getBotpose()}) is
 * returned instead and flagged via {@link Reading#isMt2} so the consumer
 * ({@code VisionFusion}) can de-weight it (MT1 yaw is ambiguous, especially on
 * a single tag).
 *
 * <p>Limelight reports SI units (meters). Positions are converted to inches and
 * mapped into the FIRST field frame (center origin, +X east/right, +Y
 * north/up-field, heading 0 = +Y, CCW+), then into Pedro's corner-origin frame
 * via {@link PedroFieldBridge#fieldToPedro}.
 *
 * <p><b>Frame mapping is a GUESS until verified on your field.</b> Defaults assume
 * Limelight field X = FIRST +X, Y = +Y, and Limelight yaw 0 = facing +Y (CCW+).
 * Verify once: place the robot at field center facing up-field (+Y, away from the
 * red wall) with 2+ tags visible, then check Panels telemetry
 * {@code Fusion/visionX/Y/Hdeg} reads ~(72, 72, ~90deg Pedro). If X/Y are
 * mirrored, flip {@link #LL_X_SIGN}/{@link #LL_Y_SIGN}; if heading is off by a
 * constant, set {@link #LL_YAW_OFFSET_DEG} (and the same offset automatically
 * applies to the yaw seeded back via {@code updateRobotOrientation}).
 */
@Configurable
public class Vision {

    // ---- Frame mapping (Panels -> Vision). See class javadoc VERIFY step. ----
    /** Sign applied to Limelight X (meters) before inches conversion. */
    public static double LL_X_SIGN = 1.0;
    /** Sign applied to Limelight Y (meters) before inches conversion. */
    public static double LL_Y_SIGN = 1.0;
    /** Fine trim added to Limelight X after conversion, inches. */
    public static double LL_X_OFFSET_IN = 0.0;
    /** Fine trim added to Limelight Y after conversion, inches. */
    public static double LL_Y_OFFSET_IN = 0.0;
    /**
     * Constant added to the Limelight yaw (degrees, CCW+) to get FIRST field
     * heading (0 = +Y). E.g. if Limelight yaw 0 actually means facing +X,
     * set -90.
     */
    public static double LL_YAW_OFFSET_DEG = 0.0;
    /** Sign of the Limelight yaw (1 = CCW+, -1 = CW+). */
    public static double LL_YAW_SIGN = 1.0;
    /** Allow MT1 fallback when MT2 is unavailable (flagged, de-weighted downstream). */
    public static boolean MT1_FALLBACK = true;
    /** Minimum ms between updateRobotOrientation seeds (network POST throttle). */
    public static double SEED_INTERVAL_MS = 100.0;
    /** Re-seed early if the yaw moved more than this (degrees) since last seed. */
    public static double SEED_YAW_DELTA_DEG = 0.5;

    /** One MegaTag solve with the quality signals the Kalman R-scaler needs. */
    public static final class Reading {
        /** Vision pose already converted to Pedro frame (inches, corner origin). */
        public final Pose pedroPose;
        /** True = MT2 solve, false = MT1 fallback. */
        public final boolean isMt2;
        public final int tagCount;
        /** Mean distance to the tags used, meters (from Limelight). */
        public final double avgDistM;
        /** Mean tag area, percent of image (from Limelight). */
        public final double avgAreaPct;
        /** Max baseline between tags used, meters (from Limelight). */
        public final double spanM;
        /**
         * Limelight self-reported XY stddev, inches (-1 when unavailable).
         * Converted from meters with a magnitude heuristic (values &lt; 5 are
         * treated as meters, else inches) — see {@link Vision#parseStddevXY}.
         */
        public final double stddevXYIn;
        /** Capture + targeting latency, ms. Used to back-date the measurement. */
        public final double latencyMs;
        /** ms since the Limelight last published (staleness gate lives downstream). */
        public final long stalenessMs;

        Reading(Pose pedroPose, boolean isMt2, int tagCount, double avgDistM,
                double avgAreaPct, double spanM, double stddevXYIn,
                double latencyMs, long stalenessMs) {
            this.pedroPose = pedroPose;
            this.isMt2 = isMt2;
            this.tagCount = tagCount;
            this.avgDistM = avgDistM;
            this.avgAreaPct = avgAreaPct;
            this.spanM = spanM;
            this.stddevXYIn = stddevXYIn;
            this.latencyMs = latencyMs;
            this.stalenessMs = stalenessMs;
        }
    }

    private final Limelight3A limelight;
    private long lastSeedNs = 0L;
    private double lastSeedDeg = Double.NaN;

    /** Legacy constructor: throws if no Limelight3A is configured. */
    public Vision(HardwareMap hardwareMap) {
        this(hardwareMap.getAll(Limelight3A.class).get(0));
    }

    public Vision(Limelight3A limelight) {
        if (limelight == null) throw new IllegalArgumentException("Vision needs a non-null Limelight3A");
        this.limelight = limelight;
    }

    /**
     * Null-safe factory: returns null when no Limelight3A exists in the
     * configuration (tuning OpModes, bench testing) so callers can run
     * Pinpoint-only without crashing.
     */
    @Nullable
    public static Vision tryCreate(HardwareMap hardwareMap) {
        try {
            List<Limelight3A> all = hardwareMap.getAll(Limelight3A.class);
            if (all == null || all.isEmpty()) return null;
            if (all.get(0) == null) return null;
            return new Vision(all.get(0));
        } catch (Exception e) {
            return null;
        }
    }

    public boolean isPresent() {
        return limelight != null;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void start() {
        start(0);
    }

    public void start(int pipeline) {
        try {
            limelight.pipelineSwitch(pipeline);
        } catch (Exception ignored) {
        }
        try {
            limelight.start();
        } catch (Exception ignored) {
        }
    }

    /**
     * Seed + poll. Call once per loop with the CURRENT field-relative yaw in
     * degrees (0 = facing +Y/up-field, CCW+, -180..180 or 0..360 — normalized
     * internally). Returns null when there is no usable solve this loop.
     */
    @Nullable
    public Reading poll(double fieldYawDeg) {
        if (limelight == null) return null;
        seedYaw(fieldYawDeg);

        final LLResult r;
        try {
            r = limelight.getLatestResult();
        } catch (Exception e) {
            return null;
        }
        if (r == null || !r.isValid()) return null;

        Pose3D mt2 = null;
        try {
            mt2 = r.getBotpose_MT2();
        } catch (Exception ignored) {
        }
        final boolean isMt2 = mt2 != null;
        Pose3D raw = mt2;
        if (raw == null) {
            if (!MT1_FALLBACK) return null;
            try {
                raw = r.getBotpose();
            } catch (Exception ignored) {
            }
            if (raw == null) return null;
        }

        Pose pedro;
        try {
            pedro = toPedroPose(raw);
        } catch (Exception e) {
            return null;
        }
        if (pedro == null) return null;

        int tags = 0;
        double dist = 0, area = 0, span = 0, latMs = 0;
        long staleMs = 0;
        double stdXY = -1;
        try {
            tags = r.getBotposeTagCount();
        } catch (Exception ignored) {
        }
        try {
            dist = r.getBotposeAvgDist();
        } catch (Exception ignored) {
        }
        try {
            area = r.getBotposeAvgArea();
        } catch (Exception ignored) {
        }
        try {
            span = r.getBotposeSpan();
        } catch (Exception ignored) {
        }
        try {
            latMs = r.getCaptureLatency() + r.getTargetingLatency();
        } catch (Exception ignored) {
        }
        try {
            staleMs = r.getStaleness();
        } catch (Exception ignored) {
        }
        try {
            stdXY = parseStddevXY(isMt2 ? r.getStddevMt2() : r.getStddevMt1());
        } catch (Exception ignored) {
        }

        return new Reading(pedro, isMt2, tags, dist, area, span, stdXY, latMs, staleMs);
    }

    /**
     * Limelight self-reported stddev array -> XY stddev in inches.
     * Only the planar components are used (index 0/1); heading is handled by
     * the fusion tunables instead because the array layout is not contractual.
     */
    static double parseStddevXY(@Nullable double[] s) {
        if (s == null || s.length < 2) return -1;
        double sx = s[0], sy = s[1];
        if (!Double.isFinite(sx) || !Double.isFinite(sy) || sx < 0 || sy < 0) return -1;
        double m = Math.max(sx, sy);
        if (m <= 0) return -1;
        // Heuristic: real solves report sub-5m stddevs in meters; anything
        // larger is assumed to already be inches (or garbage -> clamped downstream).
        double asIn = m < 5.0 ? m * 39.3700787 : m;
        return Double.isFinite(asIn) ? asIn : -1;
    }

    /**
     * Limelight Pose3D (meters, field space) -> Pedro Pose (inches, corner origin).
     * Uses the LL_* mapping tunables; see class javadoc.
     */
    public static Pose toPedroPose(Pose3D botpose) {
        double xFieldIn = botpose.getPosition().toUnit(DistanceUnit.INCH).x * LL_X_SIGN + LL_X_OFFSET_IN;
        double yFieldIn = botpose.getPosition().toUnit(DistanceUnit.INCH).y * LL_Y_SIGN + LL_Y_OFFSET_IN;
        double llYawRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        double fieldYawRad = wrapPi(llYawRad * LL_YAW_SIGN + Math.toRadians(LL_YAW_OFFSET_DEG));
        return PedroFieldBridge.fieldToPedro(xFieldIn, yFieldIn, fieldYawRad);
    }

    /** Throttled gyro seed for the on-LL MT2 solver (degrees field-relative). */
    private void seedYaw(double fieldYawDeg) {
        long nowNs = System.nanoTime();
        double wantDeg = norm180(fieldYawDeg);
        double dtMs = (nowNs - lastSeedNs) / 1.0e6;
        boolean due = (nowNs - lastSeedNs) < 0
                || dtMs >= SEED_INTERVAL_MS
                || Double.isNaN(lastSeedDeg)
                || Math.abs(angleDeltaDeg(wantDeg, lastSeedDeg)) >= SEED_YAW_DELTA_DEG;
        if (!due) return;
        try {
            // Inverse of the yaw mapping: LL = (field - offset) * sign (sign = +/-1).
            double llDeg = norm180((wantDeg - LL_YAW_OFFSET_DEG) * LL_YAW_SIGN);
            if (limelight.updateRobotOrientation(llDeg)) {
                lastSeedNs = nowNs;
                lastSeedDeg = wantDeg;
            }
        } catch (Exception ignored) {
        }
    }

    // ---- legacy API (kept for compatibility) ----

    /** Raw MT1 field botpose, meters. Prefer {@link #poll(double)}. */
    @Nullable
    public Pose3D getBotpose() {
        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                return result.getBotpose();
            }
        } catch (Exception ignored) {
        }
        return null;
    }

    /** Legacy: seed from FTC orientation, return raw MT2 botpose. Prefer {@link #poll(double)}. */
    @Nullable
    public Pose3D updateRobotOrientation(YawPitchRollAngles orientation) {
        try {
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                return result.getBotpose_MT2();
            }
        } catch (Exception ignored) {
        }
        return null;
    }

    private static double norm180(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }

    private static double angleDeltaDeg(double a, double b) {
        return norm180(a - b);
    }

    private static double wrapPi(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
