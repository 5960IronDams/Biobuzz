package org.firstinspires.ftc.teamcode.Subsystems.Vision;

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
import org.firstinspires.ftc.teamcode.HardwareHealth;
import org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFieldBridge;

import java.util.ArrayList;
import java.util.List;

/**
 * Limelight3A MegaTag pose source using a split solver: MT2 for XY position
 * (gyro-seeded, stable while moving) and MT1 for yaw (independent 6DOF solve).
 *
 * <p>Limelight reports meters; this class converts positions to inches, maps them
 * into the FIRST field frame, and shifts them into Pedro's corner-origin frame
 * via {@link PedroFieldBridge#fieldToPedro}.
 *
 * <p>All 16 BIOBUZZ tags (IDs 30-45, 4 per hive cell) sit on tipping hive cells
 * whose 30-degree rest angle is stored in the {@code .fmap}. Downstream filters
 * ({@link TipSanityFilter}) reject tilted solves when a cell is knocked off rest.
 */
@Configurable
public class Vision {
    // Reference values only — configure the actual camera mount pose in the Limelight Web UI.
    /** Camera offset forward from robot center (meters). */
    public static double CAM_X_M = 0.110;
    /** Camera offset left from robot center (meters). */
    public static double CAM_Y_M = 0.0;
    /** Camera height above the floor (meters). */
    public static double CAM_Z_M = 0.5;
    /** Camera roll around X-axis (degrees). */
    public static double CAM_ROLL_DEG = 0.0;
    /** Camera pitch around Y-axis (degrees, positive tilts up, negative tilts down). */
    public static double CAM_PITCH_DEG = 50.0;
    /** Camera yaw around Z-axis (degrees, CCW+). */
    public static double CAM_YAW_DEG = 5.0;

    // ---- Field Frame Mapping (Panels -> Vision) ----
    /** Sign multiplier for Limelight X before inches conversion (+1 or -1). */
    public static double LL_X_SIGN = 1.0;
    /** Sign multiplier for Limelight Y before inches conversion (+1 or -1). */
    public static double LL_Y_SIGN = 1.0;
    /**
     * Set true when the Limelight .fmap frame is rotated -90 deg from the FTC field frame
     * (maps field_x = ll_y, field_y = -ll_x after signs and offsets).
     */
    public static boolean LL_SWAP_XY = true;
    /** Fine offset added to Limelight X before frame swap (inches). */
    public static double LL_X_OFFSET_IN = 0.0;
    /** Fine offset added to Limelight Y before frame swap (inches). */
    public static double LL_Y_OFFSET_IN = 0.0;
    /**
     * Constant added to Limelight yaw (degrees, CCW+) to match FIRST field heading (0 = +Y).
     * Keep the Limelight Web UI yaw offset at 0 and tune only this value; it is also
     * subtracted automatically when seeding gyro yaw back to the Limelight.
     */
    public static double LL_YAW_OFFSET_DEG = 180;
    /** Sign multiplier for Limelight yaw (+1 = CCW+, -1 = CW+). */
    public static double LL_YAW_SIGN = 1.0;
    /**
     * True = allow MT1 independent 6DOF heading fusion (and MT1 XY fallback when MT2 is absent).
     * False = XY-only vision fusion; heading relies solely on Pinpoint/gyro.
     */
    public static boolean MT1_FALLBACK = true;
    /**
     * True = read botpose pitch, roll, and Z height from MT1 (full 6DOF) for tilt/height gates.
     * False = read from the XY source (MT2 pins robot to the floor and reports 0/0/0).
     */
    public static boolean ATTITUDE_FROM_MT1 = true;
    /** Minimum time between gyro orientation seeds sent to the Limelight (ms). */
    public static double SEED_INTERVAL_MS = 100.0;
    /** Re-seed immediately if robot yaw changed by at least this many degrees since the last seed. */
    public static double SEED_YAW_DELTA_DEG = 0.5;

    // ---- Corner Hive Pipelines (whitelist 4 tags per field corner) ----
    /** Automatically switch Limelight pipeline to match the hive in the robot's current quadrant. */
    public static boolean AUTO_CORNER_PIPELINE = true;
    public static int PIPELINE_RED_SCORING = 0;   // Tags 30-33 (X < 72, Y >= 72)
    public static int PIPELINE_RED_AUDIENCE = 1;  // Tags 34-37 (X < 72, Y < 72)
    public static int PIPELINE_BLUE_AUDIENCE = 2; // Tags 38-41 (X >= 72, Y < 72)
    public static int PIPELINE_BLUE_SCORING = 3;  // Tags 42-45 (X >= 72, Y >= 72)

    // BIOBUZZ cluster tags (IDs 30-45):
    // 30-33 RED SCORING, 34-37 RED AUDIENCE, 38-41 BLUE AUDIENCE, 42-45 BLUE SCORING.
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

    /** One split-solver Limelight measurement (MT2 XY + MT1 yaw) in Pedro coordinates. */
    public static final class Reading {
        /** Measurement pose in Pedro frame (inches). XY from MT2 (or MT1 fallback); heading from MT1 when present. */
        public final Pose pedroPose;
        /** Independent MT1 6DOF heading in Pedro frame (radians), or NaN when MT1 was absent. */
        public final double mt1HeadingRad;
        /** True when the MT2 solve existed this frame. */
        public final boolean hasMt2;
        /** True when the MT1 solve existed this frame. */
        public final boolean hasMt1;
        /** Raw MT1 solve in Pedro frame (independent 6DOF), or null when absent. */
        @Nullable
        public final Pose mt1Pedro;
        /** Raw MT2 solve in Pedro frame (gyro-seeded), or null when absent. */
        @Nullable
        public final Pose mt2Pedro;
        public final int tagCount;
        /** Mean distance to visible tags (meters). */
        public final double avgDistM;
        /** Mean tag image area (% of frame). */
        public final double avgAreaPct;
        /** Max baseline distance between visible tags (meters). */
        public final double spanM;
        /** Limelight self-reported XY standard deviation (inches), or -1 if unavailable. */
        public final double stddevXYIn;
        /** Capture + targeting pipeline latency (ms), used for timestamp back-dating. */
        public final double latencyMs;
        /** Time since Limelight published this frame (ms). */
        public final long stalenessMs;
        /** AprilTag IDs seen in this frame. */
        public final int[] fiducialIds;
        /** True when all visible tags are BIOBUZZ hive-cell cluster tags (30-45). */
        public final boolean clusterOnly;
        /** Botpose pitch/roll (degrees) and Z height (meters) used by sanity gates. */
        public final double botPitchDeg;
        public final double botRollDeg;
        public final double botZMeters;
        /** Raw Limelight yaw (degrees) behind {@link #pedroPose}. */
        public final double llYawDeg;
        /** Raw MT1 yaw in Limelight map frame before offsets (degrees), or NaN. */
        public final double mt1LlRawDeg;
        /** Raw MT2 yaw in Limelight map frame before offsets (degrees), or NaN. */
        public final double mt2LlRawDeg;
        /** MT1's own 6DOF pitch/roll (degrees) and Z height (meters), or NaN when absent. */
        public final double mt1PitchDeg;
        public final double mt1RollDeg;
        public final double mt1ZMeters;

        Reading(Pose pedroPose, double mt1HeadingRad, boolean hasMt2, boolean hasMt1,
                @Nullable Pose mt1Pedro, @Nullable Pose mt2Pedro,
                int tagCount, double avgDistM,
                double avgAreaPct, double spanM, double stddevXYIn,
                double latencyMs, long stalenessMs,
                int[] fiducialIds, boolean clusterOnly,
                double botPitchDeg, double botRollDeg, double botZMeters,
                double llYawDeg, double mt1LlRawDeg, double mt2LlRawDeg,
                double mt1PitchDeg, double mt1RollDeg, double mt1ZMeters) {
            this.pedroPose = pedroPose;
            this.mt1HeadingRad = mt1HeadingRad;
            this.hasMt2 = hasMt2;
            this.hasMt1 = hasMt1;
            this.mt1Pedro = mt1Pedro;
            this.mt2Pedro = mt2Pedro;
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
            this.mt1LlRawDeg = mt1LlRawDeg;
            this.mt2LlRawDeg = mt2LlRawDeg;
            this.mt1PitchDeg = mt1PitchDeg;
            this.mt1RollDeg = mt1RollDeg;
            this.mt1ZMeters = mt1ZMeters;
        }
    }

    private final Limelight3A limelight;
    private long lastSeedNs = 0L;
    private double lastSeedDeg = Double.NaN;
    private boolean polledThisLoop = false;

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

    private int currentPipeline = -1;

    public int getCurrentPipeline() {
        return currentPipeline;
    }

    public void switchPipeline(int pipeline) {
        if (limelight == null || pipeline == currentPipeline) return;
        try {
            limelight.pipelineSwitch(pipeline);
            currentPipeline = pipeline;
        } catch (Exception ignored) {
        }
    }

    public static int cornerPipelineForPose(@Nullable Pose pose, int current) {
        if (pose == null) return PIPELINE_RED_SCORING;
        double deadband = 4.0;
        double x = pose.x();
        double y = pose.y();

        boolean isBlue;
        if (current == PIPELINE_BLUE_AUDIENCE || current == PIPELINE_BLUE_SCORING) {
            isBlue = x >= (72.0 - deadband);
        } else {
            isBlue = x >= (72.0 + deadband);
        }

        boolean isScoring;
        if (current == PIPELINE_RED_SCORING || current == PIPELINE_BLUE_SCORING) {
            isScoring = y >= (72.0 - deadband);
        } else {
            isScoring = y >= (72.0 + deadband);
        }

        if (!isBlue && isScoring) return PIPELINE_RED_SCORING;
        if (!isBlue && !isScoring) return PIPELINE_RED_AUDIENCE;
        if (isBlue && !isScoring) return PIPELINE_BLUE_AUDIENCE;
        return PIPELINE_BLUE_SCORING;
    }

    public void updateCornerPipeline(@Nullable Pose pose) {
        if (!AUTO_CORNER_PIPELINE || pose == null || limelight == null) return;
        if (NoVisionZones.isInNoVisionZone(pose)) {
            // Under the center hives in a no-vision zone: avoid switching between corners
            // and collecting degenerated data.
            return;
        }
        int target = cornerPipelineForPose(pose, currentPipeline);
        if (target != currentPipeline) {
            switchPipeline(target);
        }
    }

    public void start(int pipeline) {
        currentPipeline = pipeline;
        try {
            limelight.pipelineSwitch(pipeline);
        } catch (Exception ignored) {
        }
        //pushCameraPoseToLimelight();
        try {
            limelight.start();
        } catch (Exception ignored) {
        }
    }






    /** Frames older than this (ms) trigger a stale-camera warning on HardwareHealth. */
    public static final long LIMELIGHT_FRESH_MAX_MS = 1000;

    /** Reports Limelight frame freshness to {@link HardwareHealth} once per loop (reuses poll() packet if already read). */
    public void healthPing() {
        if (limelight == null) return;
        if (polledThisLoop) {
            polledThisLoop = false;
            return;
        }
        final LLResult r;
        try {
            r = limelight.getLatestResult();
        } catch (Exception e) {
            return;
        }
        updateHealthFromResult(r);
    }

    private void updateHealthFromResult(@Nullable LLResult r) {
        if (r == null) return;
        final long staleMs;
        try {
            staleMs = r.getStaleness();
        } catch (Exception e) {
            return;
        }
        if (staleMs <= LIMELIGHT_FRESH_MAX_MS) {
            HardwareHealth.get().ok("LIMELIGHT");
        } else {
            HardwareHealth.get().report("LIMELIGHT",
                    "no new frames (" + staleMs + "ms stale)");
        }
    }

    /**
     * Seeds MT2 with the robot's current field heading (degrees, 0 = +Y, CCW+) and
     * reads both MT2 (XY) and MT1 (yaw) from the latest Limelight packet.
     * Returns null if no tags were seen or neither solve is valid.
     */
    @Nullable
    public Reading poll(double fieldYawDeg) {
        if (limelight == null) return null;
        seedYaw(fieldYawDeg);

        polledThisLoop = true;
        final LLResult r;
        try {
            r = limelight.getLatestResult();
        } catch (Exception e) {
            return null;
        }
        updateHealthFromResult(r);
        if (r == null || !r.isValid()) return null;

        // Reject if no AprilTags were used (Limelight returns a dummy 0,0,0 pose when tagCount == 0).
        int tags = 0;
        try {
            tags = r.getBotposeTagCount();
        } catch (Exception ignored) {
        }
        if (tags <= 0) return null;

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
        if (!MT1_FALLBACK) mt1 = null;
        if (isEmptyPose(mt2)) mt2 = null;
        if (isEmptyPose(mt1)) mt1 = null;
        if (mt2 == null && mt1 == null) return null;

        Pose mt1Pedro = null;
        Pose mt2Pedro = null;
        try {
            if (mt1 != null) mt1Pedro = toPedroPose(mt1);
        } catch (Exception ignored) {
        }
        try {
            if (mt2 != null) mt2Pedro = toPedroPose(mt2);
        } catch (Exception ignored) {
        }

        // XY prefers MT2 (gyro-seeded), falling back to MT1 if MT2 is absent.
        Pose pedro = mt2Pedro != null ? mt2Pedro : mt1Pedro;
        if (pedro == null) return null;

        Pose3D xySource = mt2 != null ? mt2 : mt1;
        // MT1 provides the independent heading (MT2 heading is seed-echo and never fused).
        final double mt1HeadingRad = mt1Pedro != null ? mt1Pedro.heading() : Double.NaN;
        final boolean hasMt2 = mt2Pedro != null;
        final boolean hasMt1 = mt1Pedro != null && Double.isFinite(mt1HeadingRad);

        double dist = 0, area = 0, span = 0, latMs = 0;
        long staleMs = 0;
        double stdXY = -1;
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
            stdXY = parseStddevXY(mt2 != null ? r.getStddevMt2() : r.getStddevMt1());
        } catch (Exception ignored) {
        }

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
        double botPitchDeg = 0, botRollDeg = 0, botZM = 0, llYawDeg = Double.NaN;
        double mt1RawDeg = Double.NaN, mt2RawDeg = Double.NaN;
        double mt1PitchDeg = Double.NaN, mt1RollDeg = Double.NaN, mt1ZM = Double.NaN;
        try {
            botPitchDeg = xySource.getOrientation().getPitch(AngleUnit.DEGREES);
            botRollDeg = xySource.getOrientation().getRoll(AngleUnit.DEGREES);
            llYawDeg = xySource.getOrientation().getYaw(AngleUnit.DEGREES);
            botZM = xySource.getPosition().toUnit(DistanceUnit.METER).z;
        } catch (Exception ignored) {
        }
        try {
            if (mt1 != null) {
                mt1PitchDeg = mt1.getOrientation().getPitch(AngleUnit.DEGREES);
                mt1RollDeg = mt1.getOrientation().getRoll(AngleUnit.DEGREES);
                mt1ZM = mt1.getPosition().toUnit(DistanceUnit.METER).z;
            }
        } catch (Exception ignored) {
            mt1PitchDeg = Double.NaN;
            mt1RollDeg = Double.NaN;
            mt1ZM = Double.NaN;
        }
        if (ATTITUDE_FROM_MT1 && Double.isFinite(mt1PitchDeg) && Double.isFinite(mt1RollDeg)
                && Double.isFinite(mt1ZM)) {
            botPitchDeg = mt1PitchDeg;
            botRollDeg = mt1RollDeg;
            botZM = mt1ZM;
        }
        try {
            if (mt1 != null) mt1RawDeg = mt1.getOrientation().getYaw(AngleUnit.DEGREES);
        } catch (Exception ignored) {
        }
        try {
            if (mt2 != null) mt2RawDeg = mt2.getOrientation().getYaw(AngleUnit.DEGREES);
        } catch (Exception ignored) {
        }

        Pose measurementPose = new Pose(
                pedro.x(),
                pedro.y(),
                Double.isFinite(mt1HeadingRad) ? mt1HeadingRad : pedro.heading()
        );

        return new Reading(measurementPose, mt1HeadingRad, hasMt2, hasMt1, mt1Pedro, mt2Pedro,
                tags, dist, area, span,
                stdXY, latMs, staleMs, ids, clusterOnly, botPitchDeg, botRollDeg, botZM, llYawDeg,
                mt1RawDeg, mt2RawDeg, mt1PitchDeg, mt1RollDeg, mt1ZM);
    }

    /**
     * True when a Limelight botpose is the all-zero "no solution" placeholder
     * (x = y = z = 0 exactly). A real solve never lands on exactly (0,0,0).
     */
    private static boolean isEmptyPose(@Nullable Pose3D p) {
        if (p == null) return false;
        try {
            return p.getPosition().x == 0.0 && p.getPosition().y == 0.0
                    && p.getPosition().z == 0.0;
        } catch (Exception e) {
            return false;
        }
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
        double xIn = botpose.getPosition().toUnit(DistanceUnit.INCH).x * LL_X_SIGN + LL_X_OFFSET_IN;
        double yIn = botpose.getPosition().toUnit(DistanceUnit.INCH).y * LL_Y_SIGN + LL_Y_OFFSET_IN;
        // Frame rotation (see LL_SWAP_XY): LL frame is -90deg off the FTC frame.
        double xFieldIn = LL_SWAP_XY ? yIn : xIn;
        double yFieldIn = LL_SWAP_XY ? -xIn : yIn;
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

    /**
     * Seed MT2 with a Pedro-frame heading, converting it to Limelight frame.
     * <p>Use this when you already have the fused heading in Pedro frame and want to
     * seed the Limelight without manually converting to field frame first.
     * Pipeline: Pedro (0=+X) -> Field (0=+Y, -90deg) -> Limelight.
     */
    public void seedYawPedroFrame(double pedroHeadingRad) {
        // Convert Pedro frame -> Field frame: field = pedro - π/2
        double fieldHeadingRad = wrapPi(pedroHeadingRad - Math.PI / 2.0);
        seedYaw(Math.toDegrees(fieldHeadingRad));
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
