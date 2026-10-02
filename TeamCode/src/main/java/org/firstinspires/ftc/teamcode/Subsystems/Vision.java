package org.firstinspires.ftc.teamcode.Subsystems;

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
import java.net.HttpURLConnection;
import java.net.URL;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

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
    //SET THE CAMERA POSITION IN ROBOT SPACE VIA WEB GUI! this is for reference only!!
    //this does not work.
    /** Camera offset forward (robot +X), meters. Set before first poll. */
    public static double CAM_X_M = 0.110;
    /** Camera offset left (robot +Y), meters. */
    public static double CAM_Y_M = 0.0;
    /** Camera height above the floor, meters. */
    public static double CAM_Z_M = 0.5;
    /** Camera roll (deg).  • Roll: Rotation around the X-axis (tilting the horizon side-to-side).*/
    public static double CAM_ROLL_DEG = 0.0;
    /** Camera pitch (deg). • Pitch: Rotation around the Y-axis. Tilting the camera downward toward the floor is a negative pitch (e.g., -20.0), while angling it up is positive. */
    public static double CAM_PITCH_DEG = 50.0;
    /** Camera yaw (deg). ccw+? • Yaw: Rotation around the Z-axis (turning left or right). */
    public static double CAM_YAW_DEG = 5.0;
    //

    // ---- Frame mapping (Panels -> Vision). See class javadoc VERIFY step. ----
    /** Sign applied to Limelight X (meters) before inches conversion. */
    public static double LL_X_SIGN = 1.0;
    /** Sign applied to Limelight Y (meters) before inches conversion. */
    public static double LL_Y_SIGN = 1.0;
    /**
     * TRUE when the Limelight field frame is rotated -90deg vs the FTC frame
     * this class assumes (LL +X = field south, LL +Y = field east).
     *
     * <p>Verified 2026-09-29 against {@code fiducials(4).fmap} + a stationary
     * log: (1) mapped visY matched the robot's true Pedro X (55.36) to 0.1 in
     * — the transposition signature; (2) under the rotation, the fmap's 34/35
     * positions resolve to a physically-correct east-west tag row ~49.5 in
     * ahead of the robot; (3) MT1 raw yaw read ~±180deg for a north-facing
     * robot = facing the fmap frame's −X. Applies AFTER the per-axis
     * signs/offsets: field_x = ll_y, field_y = −ll_x. Yaw is untouched (it has
     * its own offset below), and the seed path carries yaw only, so seeding is
     * unaffected by the swap itself.
     */
    public static boolean LL_SWAP_XY = true;
    /** Fine trim added to Limelight X after conversion, inches. */
    public static double LL_X_OFFSET_IN = 0.0;
    /** Fine trim added to Limelight Y after conversion, inches. */
    public static double LL_Y_OFFSET_IN = 0.0;
    /**
     * Constant added to the Limelight yaw (degrees, CCW+) to get FIRST field
     * heading (0 = +Y). CALIBRATE ONCE, in code only — leave the Limelight web
     * UI yaw offset at 0. Procedure: park at a KNOWN field heading H (e.g.
     * facing +Y => H=0, facing +X/east => H=90), read {@code Fusion/mt1LlRawDeg}
     * (raw MT1 yaw straight off the tag solve, no mapping), then set this to
     * norm180(H − raw). The same offset round-trips through seedYaw()
     * (subtracted before sending the seed) and toPedroPose() (added on return),
     * so one value serves both MT1-yaw and the MT2 seed.
     *
     * <p>Ships at 180 (2026-09-29): the {@code fiducials(4).fmap} frame's yaw
     * zero faces the fmap +X axis (physical south under the {@link #LL_SWAP_XY}
     * rotation), so a north-facing robot reads raw ≈ ±180. field = raw + 180
     * maps both ±180 modes to 0 ✓, and the seed inverse (field − 180) sends
     * −180 = facing north ✓. Re-verify with the procedure above after any map
     * re-authoring.
     */
    public static double LL_YAW_OFFSET_DEG = 180;
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

    /** One split-solver frame: MT2 XY + MT1 yaw, same Limelight packet.
     * Raw per-solve Pedro poses for the dashboard live in
     * {@link #mt1Pedro}/{@link #mt2Pedro} (full solves, Pedro frame).
     * The fused measurement in {@link #pedroPose} is: MT2-conditioned XY + MT1 heading (or NaN).
     */
    public static final class Reading {
        /**
         * Fused measurement pose in Pedro frame (inches, corner origin).
         * XY: from MT2 when present (gyro-seeded, stable XY in motion), else MT1 fallback.
         * Heading: ALWAYS from MT1 independent 6DOF solve, or NaN when absent.
         * NEVER MT2 heading (it is seed-echo, not a measurement). This ensures
         * the heading fed to the Kalman filter is a true independent measurement,
         * not gyro-seeded feedback that would double-count rotation.
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
        /**
         * Raw MT1 full solve in Pedro frame (independent 6DOF: own XY + yaw),
         * or null when MT1 was absent/disabled. Dashboard-only: the fused XY
         * comes from {@link #pedroPose} (MT2-conditioned, gyro-seeded); this
         * is the untouched tag-geometry solve. Yellow on the Field widget.
         */
        @Nullable
        public final Pose mt1Pedro;
        /**
         * Raw MT2 full solve in Pedro frame (gyro-seeded: XY + echo yaw),
         * or null when MT2 was absent. Dashboard-only. Green on the Field
         * widget.
         */
        @Nullable
        public final Pose mt2Pedro;
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
        /**
         * Raw MT1 yaw straight off the tag solve (deg, LL map frame) BEFORE
         * any LL_* mapping. Calibration reference: park at a known field
         * heading H and read this — offset = norm180(H − this). Panels:
         * {@code Fusion/mt1LlRawDeg}. NaN when MT1 absent.
         */
        public final double mt1LlRawDeg;
        /**
         * Raw MT2 yaw straight off the tag solve (deg, LL map frame) BEFORE
         * any mapping. Normally ~= the seed echoed back (MT2's yaw output is
         * dominated by the gyro seed, NOT an independent measurement) — the
         * Panels {@code Fusion/seedAgrDeg} line is |this − seed|. NaN when
         * MT2 absent.
         */
        public final double mt2LlRawDeg;

        Reading(Pose pedroPose, double mt1HeadingRad, boolean hasMt2, boolean hasMt1,
                @Nullable Pose mt1Pedro, @Nullable Pose mt2Pedro,
                int tagCount, double avgDistM,
                double avgAreaPct, double spanM, double stddevXYIn,
                double latencyMs, long stalenessMs,
                int[] fiducialIds, boolean clusterOnly,
                double botPitchDeg, double botRollDeg, double botZMeters,
                double llYawDeg, double mt1LlRawDeg, double mt2LlRawDeg) {
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
        //pushCameraPoseToLimelight();
        try {
            limelight.start();
        } catch (Exception ignored) {
        }
    }






    /**
     * This does not work. there is no rest api for this. as not seen here https://docs.limelightvision.io/docs/docs-limelight/apis/rest-http-api
     * Push the cameraPoseRobotSpace transform to the Limelight over HTTP
     * (POST /api/configset, port 5807) so the mounting geometry is set
     * programmatically each time the OpMode boots — no web-UI step.
     * Blocking (network round-trip): call from init(), never the hot loop.
     * Values come from the CAM_* configurables above; inch-based helpers
     * {@link #setCameraPoseInches} are available too.
     */
//    public boolean pushCameraPoseToLimelight() {
//        String json = String.format(Locale.US,
//                "{\"camerapose_robotspace\":[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]}", //Old:cameraPoseRobotSpace
//                CAM_X_M, CAM_Y_M, CAM_Z_M, CAM_ROLL_DEG, CAM_PITCH_DEG, CAM_YAW_DEG);
//        return postConfig(json);
//    }

    /** Convenience: set the CAM_* constants from inches/degrees and push. */
//    public boolean setCameraPoseInches(double xIn, double yIn, double zIn,
//                                       double rollDeg, double pitchDeg, double yawDeg) {
//        CAM_X_M = xIn * 0.0254;
//        CAM_Y_M = yIn * 0.0254;
//        CAM_Z_M = zIn * 0.0254;
//        CAM_ROLL_DEG = rollDeg;
//        CAM_PITCH_DEG = pitchDeg;
//        CAM_YAW_DEG = yawDeg;
//        return pushCameraPoseToLimelight();
//    }

    /** Fire one POST /api/configset request at the Limelight web server. */
    private boolean postConfig(String json) {
        HttpURLConnection conn = null;
        try {
            conn = (HttpURLConnection) new URL("http://limelight:5807/api/configset").openConnection();
            conn.setRequestMethod("POST");
            conn.setDoOutput(true);
            conn.setConnectTimeout(1000);
            conn.setReadTimeout(1000);
            try (OutputStream os = conn.getOutputStream()) {
                os.write(json.getBytes(StandardCharsets.UTF_8));
            }
            return conn.getResponseCode() == 200;
        } catch (Exception e) {
            return false;
        } finally {
            if (conn != null) conn.disconnect();
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
        // NOTE: Heading fusing is handled entirely in VisionFusion via mt1HeadingRad.
        // pedroPose heading is not used for fusion (it's overridden per-axis in fuse()).
        // MT2 heading (seed-echo) must NEVER fuse — that's ensured by the NaN-mask
        // in VisionFusion.fuse() which builds yaw from mt1HeadingRad only.
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
        double mt1RawDeg = Double.NaN, mt2RawDeg = Double.NaN;
        try {
            botPitchDeg = xySource.getOrientation().getPitch(AngleUnit.DEGREES);
            botRollDeg = xySource.getOrientation().getRoll(AngleUnit.DEGREES);
            llYawDeg = xySource.getOrientation().getYaw(AngleUnit.DEGREES);
            botZM = xySource.getPosition().toUnit(DistanceUnit.METER).z;
        } catch (Exception ignored) {
        }
        // Raw yaw off EACH solve before mapping — the calibration reference.
        // mt2 yaw is normally seed echo; mt1 yaw is the independent solve.
        try {
            if (mt1 != null) mt1RawDeg = mt1.getOrientation().getYaw(AngleUnit.DEGREES);
        } catch (Exception ignored) {
        }
        try {
            if (mt2 != null) mt2RawDeg = mt2.getOrientation().getYaw(AngleUnit.DEGREES);
        } catch (Exception ignored) {
        }

        // Raw per-solve Pedro poses for the dashboard overlay: EACH solve mapped
        // independently through the same LL_* mapping (no axis mixing). Null when
        // that solve was absent/disabled — the renderer skips nulls. Cheap
        // (two pose mappings per frame, same math poll() already does).
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

        return new Reading(pedro, mt1HeadingRad, hasMt2, hasMt1, mt1Pedro, mt2Pedro,
                tags, dist, area, span,
                stdXY, latMs, staleMs, ids, clusterOnly, botPitchDeg, botRollDeg, botZM, llYawDeg,
                mt1RawDeg, mt2RawDeg);
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
