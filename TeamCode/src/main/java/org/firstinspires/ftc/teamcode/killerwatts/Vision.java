package org.firstinspires.ftc.teamcode.killerwatts;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedro.PedroFieldBridge;

import java.util.ArrayList;
import java.util.List;

/**
 * Limelight3A MegaTag pose source: MT2-XY + MT1-yaw split solver.
 *
 * <p>Each loop the caller seeds the Limelight with the current field-relative yaw
 * (from the fused Pedro pose) and reads back BOTH solves the Limelight computes
 * from the same frame: MT2 ({@code getBotpose_MT2()}, gyro-seeded, stable XY
 * while moving) and MT1 ({@code getBotpose()}, full 6DOF from tag geometry alone).
 * MT2 yaw is NEVER fused — it is ~90% the seed echoed back, and fusing it
 * double-counts the gyro in a feedback loop (the rotation warping seen on
 * down-facing tag geometry, where yaw is weakly observed). MT2 supplies XY;
 * MT1 supplies yaw only, under strict gates downstream (multi-tag, slow, small
 * residual) so a flipped single-tag solve can never snap the heading.
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
 *
 * <p><b>BIOBUZZ: clusters ARE the map.</b> All 16 tags (IDs 30-45, 4 per cell,
 * 30deg-rest entries in {@code ftc2026BiobuzzTest.fmap}) ride on tipping hive
 * cells. MT2 XY off a rest cell is a first-class measurement; tipped-cell solves
 * are biased and handled downstream: {@code TipSanityFilter} (wired in, ON) drops
 * tilted/underground/teleport solves, {@code TipCompensator} (DISABLED until
 * field-tested) will rest-project them. Frame-map verification = park level in
 * front of a KNOWN-REST cell and check {@code Fusion/visionX/Y} vs Pinpoint.
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
     * heading (0 = +Y). BIOBUZZ value is 180: every cluster sticker is glued
     * with its bottom edge toward FIELD CENTER, so the tag's in-map "up"
     * points opposite the field +Y the seed assumes. This is a physical fact
     * about sticker orientation, NOT a fudge factor — it round-trips through
     * seedYaw() (subtracted) and toPedroPose() (added), so keep it at 180
     * unless the stickers get remounted.
     */
    public static double LL_YAW_OFFSET_DEG = 180.0;
    /** Sign of the Limelight yaw (1 = CCW+, -1 = CW+). */
    public static double LL_YAW_SIGN = 1.0;
    /**
     * Allow the MT1 yaw path (independent 6DOF heading). False = XY-only fusion
     * from MT2, heading rides on Pinpoint/gyro alone. Renamed semantics: this no
     * longer selects an either/or fallback — both solves are read every frame.
     */
    public static boolean MT1_FALLBACK = true;
    /** Minimum ms between updateRobotOrientation seeds (network POST throttle). */
    public static double SEED_INTERVAL_MS = 100.0;
    /** Re-seed early if the yaw moved more than this (degrees) since last seed. */
    public static double SEED_YAW_DELTA_DEG = 0.5;

    // ---- BIOBUZZ cluster IDs (from SDK 12.0 AprilTagGameDatabase) ----
    // All 16 tags live on 4 tipping hive cells (30deg-rest poses in the .fmap):
    // 30-33 RED SCORING, 34-37 RED AUDIENCE, 38-41 BLUE AUDIENCE, 42-45 BLUE SCORING.
    // poll() returns both solves per frame; TipSanityFilter drops the off-rest ones.
    /** True when this tag ID belongs to a moving BIOBUZZ hive-cell cluster. */
    public static boolean isClusterId(int id) {
        return id >= 30 && id <= 45;
    }

    /** Cluster base ID (30/34/38/42) for a member ID, or -1 when not a cluster tag. */
    public static int clusterBaseFor(int id) {
        if (!isClusterId(id)) return -1;
        return 30 + ((id - 30) / 4) * 4;
    }

    /** SDK cluster name for a base ID (null when unknown). */
    @Nullable
    public static String clusterNameFor(int baseId) {
        switch (baseId) {
            case 30: return "RED SCORING";
            case 34: return "RED AUDIENCE";
            case 38: return "BLUE AUDIENCE";
            case 42: return "BLUE SCORING";
            default: return null;
        }
    }

    /** One split-solver frame: MT2 XY + MT1 yaw, same Limelight packet. */
    public static final class Reading {
        /**
         * MT2 pose converted to Pedro frame (inches, corner origin). Its YAW
         * COMPONENT IS ECHO, NOT MEASUREMENT — downstream must NaN-mask it
         * out of every addMeasurement (fuse XY only).
         */
        public final Pose pedroPose;
        /**
         * MT1 yaw in Pedro frame, radians, or NaN when MT1 was unavailable.
         * This is the ONLY heading source: independent 6DOF solve, no gyro in.
         */
        public final double mt1HeadingRad;
        /** True when the MT2 solve existed this frame (XY source). */
        public final boolean hasMt2;
        /** True when the MT1 solve existed this frame (yaw source). */
        public final boolean hasMt1;
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
        /** IDs seen in this frame (from per-fiducial results; empty if LL omitted them). */
        public final int[] fiducialIds;
        /** True when every seen ID is a BIOBUZZ moving-cluster tag (30-45). */
        public final boolean clusterOnly;
        /** Botpose pitch/roll (deg) + camera height (m). Tilt sanity lives downstream. */
        public final double botPitchDeg;
        public final double botRollDeg;
        public final double botZMeters;
        /**
         * Untouched Limelight yaw (deg, LL map frame) behind pedroPose. Lets
         * VisionFusion compare seed-vs-return yaw agreement on Panels — the
         * MT2 yaw health signal on down-facing tag geometry.
         */
        public final double llYawDeg;

        Reading(Pose pedroPose, double mt1HeadingRad, boolean hasMt2, boolean hasMt1,
                int tagCount, double avgDistM,
                double avgAreaPct, double spanM, double stddevXYIn,
                double latencyMs, long stalenessMs,
                int[] fiducialIds, boolean clusterOnly,
                double botPitchDeg, double botRollDeg, double botZMeters,
                double llYawDeg) {
            this.pedroPose = pedroPose;
            this.mt1HeadingRad = mt1HeadingRad;
            this.hasMt2 = hasMt2;
            this.hasMt1 = hasMt1;
            this.tagCount = tagCount;
            this.avgDistM = avgDistM;
            this.avgAreaPct = avgAreaPct;
            this.spanM = spanM;
            this.stddevXYIn = stddevXYIn;
            this.latencyMs = latencyMs;
            this.stalenessMs = stalenessMs;
            this.fiducialIds = fiducialIds != null ? fiducialIds : new int[0];
            this.clusterOnly = clusterOnly;
            this.botPitchDeg = botPitchDeg;
            this.botRollDeg = botRollDeg;
            this.botZMeters = botZMeters;
            this.llYawDeg = llYawDeg;
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
     * internally). Returns null when NEITHER solve exists this loop. Both solves
     * come from the same Limelight packet: MT2 supplies XY (gyro-seeded, stable
     * in motion), MT1 supplies yaw (independent 6DOF). Either may be absent —
     * downstream NaN-masks the missing axis, so a partial Reading still fuses.
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

        // BOTH solves, same frame, no either/or. MT2 = XY source (its yaw is
        // seed echo). MT1 = yaw source (no gyro in). Each may be null.
        Pose3D mt2 = null;
        Pose3D mt1 = null;
        try {
            mt2 = r.getBotpose_MT2();
        } catch (Exception ignored) {
        }
        try {
            mt1 = r.getBotpose();
        } catch (Exception ignored) {
        }
        // MT1 disabled via tunable = pretend it was absent (XY-only fusion).
        if (!MT1_FALLBACK) mt1 = null;
        if (mt2 == null && mt1 == null) return null;

        // XY comes from MT2 when present, else MT1 position as fallback.
        Pose3D xySource = mt2 != null ? mt2 : mt1;
        Pose3D yawSource = mt1;
        Pose pedro;
        try {
            pedro = toPedroPose(xySource);
        } catch (Exception e) {
            return null;
        }
        if (pedro == null) return null;

        // MT1 yaw through the SAME mapping (offset + sign + Pedro shift) as
        // positions, so both axes share one frame. NaN when MT1 absent.
        double mt1Heading = Double.NaN;
        if (yawSource != null) {
            try {
                mt1Heading = toPedroPose(yawSource).heading();
            } catch (Exception ignored) {
                mt1Heading = Double.NaN;
            }
        }
        final double mt1HeadingRad = mt1Heading;
        final boolean hasMt2 = mt2 != null;
        final boolean hasMt1 = yawSource != null && Double.isFinite(mt1HeadingRad);

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
            // Stddev tracks the XY SOURCE: MT2 stddev when MT2 present, else MT1.
            stdXY = parseStddevXY(mt2 != null ? r.getStddevMt2() : r.getStddevMt1());
        } catch (Exception ignored) {
        }

        // Per-fiducial IDs: every BIOBUZZ tag is a cluster tag (IDs 30-45, level
        // poses in the .fmap). clusterOnly is normally true; it goes false only
        // if a foreign/test tag appears — still fused, but no longer "pure cluster".
        int[] ids = new int[0];
        boolean clusterOnly = false;
        try {
            List<LLResultTypes.FiducialResult> frs = r.getFiducialResults();
            if (frs != null && !frs.isEmpty()) {
                ArrayList<Integer> seen = new ArrayList<>();
                for (LLResultTypes.FiducialResult fr : frs) {
                    if (fr != null) seen.add(fr.getFiducialId());
                }
                ids = new int[seen.size()];
                boolean anyStatic = false;
                for (int i = 0; i < seen.size(); i++) {
                    ids[i] = seen.get(i);
                    if (!isClusterId(ids[i])) anyStatic = true;
                }
                clusterOnly = !seen.isEmpty() && !anyStatic;
            }
        } catch (Exception ignored) {
        }
        // Botpose attitude sanity off the XY source; seed-agreement yaw likewise.
        // GOOD (cell at rest, map holding the 30deg) reports flat ~= 0.
        double botPitchDeg = 0, botRollDeg = 0, botZM = 0, llYawDeg = Double.NaN;
        try {
            botPitchDeg = xySource.getOrientation().getPitch(AngleUnit.DEGREES);
            botRollDeg = xySource.getOrientation().getRoll(AngleUnit.DEGREES);
            llYawDeg = xySource.getOrientation().getYaw(AngleUnit.DEGREES);
            botZM = xySource.getPosition().toUnit(DistanceUnit.METER).z;
        } catch (Exception ignored) {
        }

        return new Reading(pedro, mt1HeadingRad, hasMt2, hasMt1, tags, dist, area, span,
                stdXY, latMs, staleMs, ids, clusterOnly, botPitchDeg, botRollDeg, botZM, llYawDeg);
    }

    /**
     * Raw per-tag detections for relative aiming (HiveCellMonitor). Empty list
     * when the Limelight has nothing. Same cluster tags the global pose uses —
     * aim with their goal-point poses, localize with the filtered botpose.
     */
    public List<LLResultTypes.FiducialResult> getLatestFiducials() {
        try {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                List<LLResultTypes.FiducialResult> frs = r.getFiducialResults();
                if (frs != null) return frs;
            }
        } catch (Exception ignored) {
        }
        return new ArrayList<>();
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
