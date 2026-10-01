package org.firstinspires.ftc.teamcode.pedro;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.killerwatts.TipCompensator;
import org.firstinspires.ftc.teamcode.killerwatts.TipSanityFilter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;

import java.lang.reflect.Field;
import java.util.Locale;

/**
 * Vision corrector: feeds Limelight MegaTag solves into Pedro's
 * {@link FusionLocalizer} Kalman filter via {@code addMeasurement()}.
 *
 * <p>Call order per loop (see {@link RobotMain#RobotRunPeriodic}):
 * <pre>
 * follower.update();      // predict step (Pinpoint) + fuse owner
 * visionFusion.correct(); // zero or one axis-masked addMeasurement
 * </pre>
 * TeleOp corrects twice per loop ({@code RobotRunPeriodic} + the trailing
 * drive update); the second call re-polls the same ~10Hz Limelight packet and
 * is a same-frame no-op via fingerprint dedup, so each frame fuses once.
 * Telemetry is NEVER emitted here — {@link #report()} runs once per loop from
 * {@link RobotMain#flushTelemetry()}, which is why Panels shows one set of
 * {@code Fusion/*} + {@code Tip/*} lines.
 * The follower owns the prediction; this class owns the correction. Nothing
 * here calls {@code setPose()}, so Foresight's velocity estimate is never
 * teleported — corrections blend with gain K = P/(P+R) and re-propagate
 * through the timestamp history.
 *
 * <p><b>Soft gating, not hard gating.</b> Almost every reading is fused; bad
 * conditions inflate the per-measurement R (measurement-noise variance passed
 * as the 3rd {@code addMeasurement} arg) so the filter trusts vision less
 * instead of dropping it — including big XY residuals ("teleports"), which
 * grow R quadratically past the knee instead of rejecting. Only garbage is
 * skipped outright: no solve, NaNs, off-field poses, stale frames. This
 * keeps corrections flowing through tag deserts instead of going blind.
 *
 * <p><b>Split solver: MT2 owns XY, MT1 owns yaw, never MT2 yaw.</b> MT2 yaw is
 * seed echo (~90% the gyro value fed back through the Limelight), so fusing it
 * double-counts the gyro in a feedback loop — the rotation warping seen on
 * down-facing tag geometry, where yaw is weakly observed. MT2's XY is fused
 * every accepted frame; MT1's independent 6DOF yaw fuses under strict gates
 * (multi-tag, slow, small residual) with a long time constant, so it drags gyro
 * drift out without ever snapping the heading. {@code FusionLocalizer} skips
 * NaN components per axis, so each fuse() call masks the axis it doesn't own.
 *
 * <p>All tunables are Panels-live ({@code Panels -> VisionFusion}).
 */
@Configurable
public class VisionFusion {

    // ================= master switch =================
    /** Set false (Panels) to run Pinpoint-only, e.g. while debugging vision. */
    public static boolean ENABLED = true;
    /**
     * Master switch for fusing MT1's independent yaw into the heading.
     * DISABLED (2026-09-29): ANY finite-yaw measurement corrupts the heading
     * in this Pedro build — run 23-22: ONE sane MT1 yaw fuse (innovation
     * -12.9deg) flipped the heading 90 -> 255.6deg on an unmoved robot; run
     * 23-04: neutral-yaw fuses stepped it 90 -> 189.9 -> 283.7 in
     * measurement-sized steps. The corruption is inside Pedro's
     * fuse/history re-propagation, not our mapping. While false the heading
     * rides Pinpoint-only (stable to ~0.0005deg across every run) and MT1's
     * mapped yaw still logs (Fusion/mt1RawDeg) for calibration. Re-enable
     * only after a bench test proves addMeasurement's heading handling.
     */
    public static boolean MT1_YAW_FUSING = false;
    /**
     * Diagnostic/kill switch for the XY fuse itself. Keeps poll, gates, seed
     * and ALL Fusion/* logging alive but never calls addMeasurement. Pedro's
     * fuse has corrupted the heading in every run where a measurement was
     * applied (23-04 neutral-yaw steps, 23-22 sane yaw fuse, 23-47 XY-only
     * fuses with yaw NaN-masked). Bytecode says the masked axis is inert, so
     * the corruption lives in Pedro's fuse-side history re-propagation or the
     * Pinpoint feed — Fusion/pinpointHdec decides. Set false until isolated;
     * matches run with FUSE_XY=false and MT1_YAW_FUSING=false = Pinpoint-only.
     */
    public static boolean FUSE_XY = true;
    /**
     * Direct-blend XY gain per accepted LL frame (0-1). Applied as
     * alpha = FUSE_GAIN × (BASE_R_XY / rXY), so every R inflation (residual
     * ramp, seed disagreement, tilt, stddev) scales it down automatically.
     * 0.01 = conservative until the fmap is verified; raise toward 0.05-0.1
     * once Fusion/visionX/Y tracks Pinpoint truth on a verified map.
     */
    public static double FUSE_GAIN = 0.01;
    /**
     * HUNT MODE (2026-09-30): when true, accepted corrections are routed
     * through Pedro's {@code FusionLocalizer.addMeasurement} INSTEAD of the
     * direct blend, with the instrumented subclass (InstrumentedFusionLocalizer,
     * swapped in by RobotMain) dumping the filter history around every call.
     * Purpose: reproduce the one-time ~138-206deg heading injection on-robot
     * and catch which history entry it touches. Default FALSE — the direct
     * blend is the production path. Statuses are prefixed "pedro-" while
     * hunting so the two paths never blur in the logs.
     */
    public static boolean USE_PEDRO_FUSE = false;
    /**
     * Direct-blend MT1-yaw gain per accepted frame (0-1). THE SAFE injection
     * path for vision heading: a direct heading write via setPose — no Pedro
     * fuse involved, so no corruption risk. Only corrects on sane innovations
     * (|innov| <= 90deg). Default 0 = OFF (heading rides Pinpoint-only, which
     * tracked to 0.1deg through the full rotation test). Raise ~0.02-0.05 only
     * after a bench test shows acceptable Pinpoint yaw drift, and only with a
     * verified fmap (MT1's mapped yaw is only as good as the map's tag
     * orientations — currently ±4deg wobble from the hand-dragged 34/35).
     * Independent of MT1_YAW_FUSING (that switch gates the dead Pedro-fuse
     * path and is kept for reference).
     */
    public static double FUSE_GAIN_H = 0.0;

    // ================= absolute reject gates (garbage only) =================
    /** Reject frames older than this (LL poll + USB + loop delay), ms. */
    public static double MAX_STALENESS_MS = 350.0;
    /** Reject poses outside the field plus this margin, inches. */
    public static double FIELD_MARGIN_IN = 24.0;
    /**
     * Big-residual flag threshold, inches — SOFT, not a reject. When dxy exceeds
     * this the frame STILL fuses (R grows quadratically, clamped by
     * MAX_TOTAL_SCALE) and the status gains a {@code -bigjump} suffix so Panels
     * shows it. XY source is MT2 when present, else MT1-position fallback — see
     * {@code Fusion/xySrc}. MT2 yaw never fuses, so a yaw jump alone never
     * affects a good XY frame.
     */
    public static double MAX_JUMP_XY_IN = 36.0;
    /** Clamp on total pipeline latency used for back-dating, ms. */
    public static double MAX_LATENCY_MS = 250.0;
    /**
     * XY source override: true = use MT1's independent per-tag-averaged solve
     * as the measurement instead of MT2's yaw-seeded one. Added after the
     * 02-47 runs: the per-tag PnP ranges were correct (~58in ≈ true 3D) while
     * MT2's yaw-seeded botpose collapsed the depth to ~0.4x truth — if MT1's
     * XY proves sane vs Pinpoint, this is the working source.
     */
    public static boolean USE_MT1_XY = false;
    // NOTE: code-side camera lever-arm compensation (former CAM_OFFSET_FWD/LEFT_IN)
    // REMOVED — the Limelight's mount geometry (forward/side/up/pitch) is now
    // configured in the LL web UI / LL init method, so botpose already reports
    // the ROBOT CENTER. Compensating here as well would double-apply the offset.
    // Regression check: heading-sweep residual test must stay flat.

    // ================= moving-cluster / attitude gates (BIOBUZZ) =================
    // BIOBUZZ localizes off the 4 hive-cell clusters (IDs 30-45), whose REST pose
    // is 30deg tipped. Stage 1 (TipSanityFilter, always on) drops solves far from
    // the rest signature: off-rest cells, underground/floating height, teleports.
    // Stage 2 (TipCompensator, DISABLED until field-tested) tries to rescue
    // off-rest solves by rest projection; see that class for the validation plan.
    // REJECT_CLUSTER_ONLY stays as a big red switch: true = Pinpoint-only whenever
    // ONLY cluster tags are visible (conservative). False = let cluster solves
    // through the sanity filter like any other solve (your call: you WILL localize
    // off them — leave false once the fmap + compensator check out on-field).
    /**
     * Only-cluster-visible switch (default false = FUSE cluster solves, filtered).
     * True = reject cluster-only solves outright (Pinpoint-only fallback).
     * You asked to localize off the clusters, so this ships false; Stage 1
     * still drops the tipped ones, Stage 2 (disabled) will rescue them later.
     */
    public static boolean REJECT_CLUSTER_ONLY = false;
    /**
     * Backstop: |botpose pitch/roll| beyond this (deg) means off-rest.
     * GOOD (cell at rest, map matching) reports flat ~= 0 — the 30deg lives in
     * the .fmap entries, not in the solve. Mirrors TipSanityFilter (the real
     * gate); kept here so Panels shows one story. 0 disables.
     * NOTE: pitch/roll come from the MT2 XY solve (the fused position source).
     */
    public static double MAX_BOT_PITCH_DEG = 12.0;
    /** Same for |botpose roll|, degrees. 0 disables. */
    public static double MAX_BOT_ROLL_DEG = 12.0;
    /** Reject when reported camera height is outside [min, max], meters. */
    public static double BOT_Z_MIN_M = 0.05;
    public static double BOT_Z_MAX_M = 0.60;
    // ================= MT2-XY health (seed agreement repurposed) =================
    // The seed-vs-return yaw comparison used to penalize MT2 heading. Heading no
    // longer fuses from MT2 at all (NaN-masked — see fuse()), so the agreement
    // is now an MT2-XY TRUST signal: large disagreement means the gyro seed the
    // XY solve was conditioned on was wrong, so the XY is suspect. It scales sXY
    // (not sH). A healthy loop reads seedAgrDeg small (< ~10). Pinned near 180 =
    // seed frame mirrored — fix LL_YAW_OFFSET_DEG.
    /**
     * Extra XY R x when |seed − returned| yaw exceeds YAW_AGR_FULL_DEG.
     * Ramps 1x below FULL to this at YAW_AGR_MAX_DEG. 0/1 = gate off.
     */
    public static double YAW_DISAGREE_PENALTY = 4.0;
    /** Agreement below this (deg) = healthy, no penalty. */
    public static double YAW_AGR_FULL_DEG = 15.0;
    /** Agreement at/above this (deg) = full penalty. */
    public static double YAW_AGR_MAX_DEG = 60.0;
    /** Hard-reject the frame when disagreement exceeds this (deg). 0 disables. */
    public static double YAW_AGR_REJECT_DEG = 120.0;

    // ================= base trust (variances, NOT stddevs) =================
    // R is digested as Matrix.diag(x, y, heading). These match
    // Constants.fusionDefaultMeasurementNoise(); the per-reading scaler below
    // multiplies them. Heading is rad^2: 6deg stddev -> 0.011 var.
    /** Base XY measurement variance, in^2 (3in stddev). MT2 XY fuses at this. */
    public static double BASE_R_XY = 9.0;
    /**
     * MT1 yaw measurement variance, rad^2. Ships at 18deg stddev (~0.10): slow
     * drift correction, never a snap. Shrink toward ~10deg (0.03) once the
     * split proves out on-field; grow if heading ever warps again.
     */
    public static double BASE_R_HEADING = 0.10;

    // ================= soft de-weight knobs =================
    // Each factor is multiplicative on R, i.e. trust /= factor. sXY feeds the
    // MT2-XY fuse; sH feeds the MT1-yaw fuse. They are computed independently.
    /** MT1-ABSENT: no MT2 either is impossible (poll returns null); this covers MT1-XY fallback. */
    public static double MT1_XY_PENALTY = 4.0;
    /**
     * MT1 yaw variance floor, rad^2 (~30deg stddev). Single-tag MT1 yaw is
     * flip-ambiguous on down-facing tags — the strict YAW_* gates below are the
     * real protection; this floor keeps even a passing solve honest.
     */
    public static double MT1_HEADING_R = 0.27;
    // ---- MT1-yaw gates (tags/speed/turn/dist skip YAW only; residual is soft R) ----
    // COORDINATE FRAME FLOW FOR MT1 YAW FUSION:
    // Forward (measurement):
    //   1. LL reports MT1 yaw in Limelight frame
    //   2. toPedroPose() converts: ll_yaw * LL_YAW_SIGN + LL_YAW_OFFSET_DEG -> field_yaw
    //   3. fieldHeadingToPedro(): field_yaw + 90deg -> pedro_yaw (in Pedro frame)
    //   4. Fused into Kalman filter
    // Backward (seeding):
    //   1. Get fused heading (Pedro frame)
    //   2. pedroHeadingToField(): pedro_yaw - 90deg -> field_yaw
    //   3. seedYaw(): (field_yaw - LL_YAW_OFFSET_DEG) * LL_YAW_SIGN -> ll_yaw
    //   4. Send ll_yaw to Limelight via updateRobotOrientation()
    // MT1 yaw only fuses when BOTH the measurement is valid AND all gates pass.
    // If yaw is skipped, check these gates first:
    /** Fuse MT1 yaw only with >= this many tags (flip ambiguity). */
    public static int YAW_MIN_TAGS = 2;
    /** Fuse MT1 yaw only below this planar speed (in/s). */
    public static double YAW_MAX_SPEED_IPS = 20.0;
    /** Fuse MT1 yaw only below this turn rate (rad/s). */
    public static double YAW_MAX_OMEGA_RPS = 0.5;
    /** Fuse MT1 yaw only below this mean tag distance (m). */
    public static double YAW_MAX_DIST_M = 2.5;
    /** Fuse MT1 yaw with residual-scaled R; this is the knee of the ramp (deg). Soft. */
    public static double YAW_MAX_RESIDUAL_DEG = 20.0;
    /** Extra R x per tag below 2 (single-tag solves are ambiguous). */
    public static double SINGLE_TAG_PENALTY = 3.0;
    /** R x per meter of mean tag distance past CLOSE_DIST_M. */
    public static double DIST_SLOPE_PER_M = 0.6;
    /** Distance (m) inside which no distance penalty applies. */
    public static double CLOSE_DIST_M = 1.5;
    /**
     * Limelight self-reported stddev below this (inches) is trusted 1:1 as a
     * variance (std^2 replaces base); above it the reading is treated as
     * "LL is unsure" and R is scaled up instead of down. 0 disables.
     */
    public static double STDDEV_TRUST_CAP_IN = 6.0;
    /** R x per (in/s of planar speed / SPEED_REF_IPS), quadratic. XY path. */
    public static double SPEED_REF_IPS = 40.0;
    /** R x per (rad/s of turn rate / TURN_REF_RPS), quadratic, XY path. */
    public static double TURN_REF_RPS = 2.0;
    /** R x when tag area is tiny (far/small in frame), ramps below AREA_REF_PCT. XY path. */
    public static double AREA_REF_PCT = 0.5;
    /** R x when the tag baseline is narrow (poor triangulation), ramps below SPAN_REF_M. XY path. */
    public static double SPAN_REF_M = 1.0;
    // Innovation (XY residual) de-weight: smooth, never a cliff. Yaw residual
    // has its own ramp in the MT1-yaw path (same shape, deg units).
    /** Residual below this fuses at full weight, inches. */
    public static double RESIDUAL_FULL_IN = 6.0;
    /** Knee of the residual ramp, inches — past this R keeps growing (no cliff, no cap). */
    public static double RESIDUAL_MAX_IN = 24.0;
    /** R scale reached AT the knee (ramps quadratically from 1x at FULL; unbounded past MAX). */
    public static double RESIDUAL_MAX_PENALTY = 9.0;
    /** Global clamp: no single reading may scale R beyond this. */
    public static double MAX_TOTAL_SCALE = 64.0;

    // ---- last-loop debug snapshot (telemetry) ----
    private String lastStatus = "no-vision";
    private double lastScale = 1.0;
    private double lastRx = BASE_R_XY;
    private double lastRh = Double.NaN;
    private Pose lastVision = null;
    /** Raw MT1 full solve, Pedro frame (dashboard yellow). Null when MT1 absent. */
    private Pose lastMt1Pedro = null;
    /** Raw MT2 full solve, Pedro frame (dashboard green). Null when MT2 absent. */
    private Pose lastMt2Pedro = null;
    private boolean lastMt2 = false;
    private boolean lastVisionClusterOnly = false;
    private int lastTags = 0;
    private double lastResidualIn = Double.NaN;
    /** MT1 yaw residual |MT1yaw − fused| this loop, deg (NaN when yaw skipped). */
    private double lastYawResidualDeg = Double.NaN;
    /** Signed yaw innovation (MT1 − fused, wrapped ±180), deg. Shows flips/wrap. */
    private double lastYawInnovDeg = Double.NaN;
    /** Raw mapped MT1 yaw before wrap-to-fused, deg (NaN when MT1 absent). */
    private double lastMt1RawDeg = Double.NaN;
    /** Raw MT1 yaw straight off LL (no LL_* mapping), deg. The offset cal ref. */
    private double lastMt1LlRawDeg = Double.NaN;
    /** Raw MT2 yaw straight off LL (normally ~= seed echo), deg. */
    private double lastMt2LlRawDeg = Double.NaN;
    /** Seed actually sent to LL this loop, field-deg (NaN when not re-seeded). */
    private double lastSeedSentDeg = Double.NaN;
    /** True when the MT1 yaw axis fused this loop. */
    private boolean lastYawFused = false;
    /** Why yaw fused/skipped this loop ("fused", "tags", "speed", ...). */
    private String lastYawSkip = "none";
    /** MT1 yaw R this loop, rad^2 (NaN when yaw skipped). */
    private double lastYawR = Double.NaN;
    /** Seed-vs-return yaw agreement this loop, deg (NaN when MT2 absent). */
    private double lastSeedAgrDeg = Double.NaN;
    /** XY source this loop: "MT2" or "MT1fb" (MT1-position fallback). */
    private String lastXySrc = "n/a";
    /** XY residual components (meas − fused), inches. */
    private double lastDx = Double.NaN;
    private double lastDy = Double.NaN;

    private final Follower follower;
    @Nullable
    private final Vision vision;
    /** Pedro's dead-reckoning localizer (the Pinpoint), resolved reflectively once. */
    @Nullable
    private Localizer deadReckoning = null;
    /** Stage 1 sanity filter (stateless except last-accepted teleport check). */
    private final TipSanityFilter tipFilter = new TipSanityFilter();

    // ---- same-frame dedup (two correct() calls per TeleOp loop) ----
    /**
     * Max ns between polls to treat a bit-identical packet as the same LL
     * frame. Same-loop re-polls land ~1-5ms apart; next-loop polls are a full
     * loop period later, so they re-evaluate normally.
     */
    private static final long SAME_FRAME_MAX_NS = 10_000_000L; // 10ms
    private long lastPollNs = 0L;
    private boolean hasLastFrame = false;
    private double lastFrameX, lastFrameY, lastFrameH, lastFrameMt1, lastFrameLat, lastFrameYaw;
    private int lastFrameTags;
    private long lastFrameStale;

    public VisionFusion(Follower follower, @Nullable Vision vision) {
        if (follower == null) throw new IllegalArgumentException("VisionFusion needs a non-null Follower");
        this.follower = follower;
        this.vision = vision;
        try {
            Field drField = FusionLocalizer.class.getDeclaredField("deadReckoning");
            drField.setAccessible(true);
            Object dr = drField.get(follower.localizer);
            if (dr instanceof Localizer) {
                deadReckoning = (Localizer) dr;
            }
        } catch (Exception ignored) {
        }
        if (vision == null) lastStatus = "no-limelight-configured";
    }

    /** True when corrections can flow (enabled + Limelight present + follower on Fusion). */
    public boolean isActive() {
        return ENABLED && vision != null && follower.localizer instanceof FusionLocalizer;
    }

    /**
     * One correction step. Call AFTER {@code follower.update()} each loop.
     * Safe to call when inactive (reports status, fuses nothing).
     */
    public void correct() {
        if (!ENABLED) {
            lastStatus = "disabled";
            return;
        }
        if (vision == null) {
            lastStatus = "no-limelight-configured";
            return;
        }
        Localizer loc = follower.localizer;
        if (!(loc instanceof FusionLocalizer)) {
            lastStatus = "not-fusion-localizer";
            return;
        }
        FusionLocalizer fusion = (FusionLocalizer) loc;

        Pose fused = follower.pose();
        Velocity vel;
        try {
            vel = follower.velocity();
            if (vel == null) vel = Velocity.zero();
        } catch (Exception e) {
            vel = Velocity.zero();
        }
        if (fused == null) {
            lastStatus = "no-fused-pose";
            return;
        }
        
        // SANITY CHECK: if the fused pose has diverged massively (outside 2x field bounds),
        // skip vision fusion entirely until Pinpoint recovers. This prevents MT2 teleportation
        // from pushing an already-diverged filter further into invalid territory.
        if (fused.x() < -144.0 || fused.x() > 288.0
                || fused.y() < -144.0 || fused.y() > 288.0) {
            lastStatus = "skip-fused-diverged";
            return;
        }

        // Seed MT2 with the CURRENT fused heading in Limelight coordinates.
        // The Limelight MT2 solver uses the seed to condition its XY solve.
        // We send it in Limelight's native frame: Pedro -> Field -> Limelight.
        double fieldYawDeg = Math.toDegrees(PedroFieldBridge.pedroHeadingToField(fused.heading()));
        // fieldYawDeg is now in field frame (0 = +Y/up-field). poll() will convert it to LL frame.
        Vision.Reading rd;
        try {
            rd = vision.poll(fieldYawDeg);
        } catch (Exception e) {
            lastStatus = "poll-exception";
            return;
        }
        if (rd == null || rd.pedroPose == null) {
            lastStatus = "no-solve";

            return;
        }
        // Same-frame dedup: TeleOp corrects twice per loop (RobotRunPeriodic's
        // predict + the trailing drive update) off one ~10Hz LL packet. The
        // re-poll returns bit-identical doubles with an aging staleness; skip it
        // so the frame fuses once. A genuinely new frame resets staleness (or
        // changes a byte) and still fuses. Snapshot below stays from the first
        // call, so the once-per-loop report() is unaffected.
        long nowNs = System.nanoTime();
        if (hasLastFrame && (nowNs - lastPollNs) < SAME_FRAME_MAX_NS
                && rd.tagCount == lastFrameTags
                && rd.latencyMs == lastFrameLat
                && rd.stalenessMs >= lastFrameStale - 1
                && rd.pedroPose.x() == lastFrameX
                && rd.pedroPose.y() == lastFrameY
                && Double.compare(rd.pedroPose.heading(), lastFrameH) == 0
                && Double.compare(rd.mt1HeadingRad, lastFrameMt1) == 0
                && Double.compare(rd.llYawDeg, lastFrameYaw) == 0) {
            return;
        }
        lastPollNs = nowNs;
        hasLastFrame = true;
        lastFrameX = rd.pedroPose.x();
        lastFrameY = rd.pedroPose.y();
        lastFrameH = rd.pedroPose.heading();
        lastFrameMt1 = rd.mt1HeadingRad;
        lastFrameTags = rd.tagCount;
        lastFrameLat = rd.latencyMs;
        lastFrameYaw = rd.llYawDeg;
        lastFrameStale = rd.stalenessMs;
        lastVision = rd.pedroPose;
        // Raw per-solve snapshot for the dashboard overlay (drawn even when the
        // frame later rejects — a stale/dropped solve still shows its last pose).
        lastMt1Pedro = rd.mt1Pedro;
        lastMt2Pedro = rd.mt2Pedro;
        lastMt2 = rd.hasMt2;
        lastXySrc = rd.hasMt2 ? "MT2" : "MT1fb";
        lastMt1LlRawDeg = rd.mt1LlRawDeg;
        lastMt2LlRawDeg = rd.mt2LlRawDeg;
        lastSeedSentDeg = fieldYawDeg;
        lastVisionClusterOnly = rd.clusterOnly;
        lastTags = rd.tagCount;

        // ---- MT2 seed agreement (computed BEFORE any reject return) ----
        // The old location (after the residual gates) left Fusion/seedAgrDeg
        // "n/a" in exactly the high-residual stationary regime where the
        // MT2-XY trust signal matters most — every frame returned at
        // reject-residual-stationary before reaching it. Compute + log first;
        // the reject/penalty decisions below keep their old thresholds.
        double seedAgrDeg = Double.NaN;
        double seedPenalty = 1.0;
        if (rd.hasMt2 && Double.isFinite(rd.llYawDeg)) {
            // Seed as-sent was fieldYawDeg; map it back to the LL frame the
            // same way seedYaw() does: LL = (field − offset) · sign.
            double seedLlDeg = norm180Deg((fieldYawDeg - Vision.LL_YAW_OFFSET_DEG) * Vision.LL_YAW_SIGN);
            seedAgrDeg = Math.abs(norm180Deg(rd.llYawDeg - seedLlDeg));
            lastSeedAgrDeg = seedAgrDeg;
            if (YAW_AGR_REJECT_DEG > 0 && seedAgrDeg > YAW_AGR_REJECT_DEG) {
                lastStatus = "reject-seed";
                return;
            }
            if (YAW_DISAGREE_PENALTY > 1 && seedAgrDeg > YAW_AGR_FULL_DEG) {
                double t = Math.min((seedAgrDeg - YAW_AGR_FULL_DEG)
                        / Math.max(YAW_AGR_MAX_DEG - YAW_AGR_FULL_DEG, 1e-6), 1.0);
                seedPenalty = 1.0 + (YAW_DISAGREE_PENALTY - 1.0) * t * t;
            }
        } else {
            lastSeedAgrDeg = Double.NaN;
        }

        // ---- absolute reject gates (XY source: NaN / stale only) ----
        // NOTE: heading is NOT gated here — MT2 yaw never fuses (NaN-masked in
        // fuse()), and MT1 yaw has its own strict gates below. A garbage MT2 yaw
        // cannot hurt us; only garbage XY can.
        if (!Double.isFinite(rd.pedroPose.x()) || !Double.isFinite(rd.pedroPose.y())) {
            lastStatus = "reject-nan";

            return;
        }
        if (rd.stalenessMs > MAX_STALENESS_MS) {
            lastStatus = "reject-stale";

            return;
        }
        // ---- Stage 1: rest-signature sanity (TipSanityFilter, always on) ----
        // Drops off-rest / underground / teleporting solves BEFORE the old inline
        // gates. The inline gates below stay as a backstop (same thresholds, so
        // Panels shows one consistent story either way).
        TipSanityFilter.Decision sanity;
        try {
            sanity = tipFilter.filter(rd);
        } catch (Exception e) {
            lastStatus = "sanity-exception";

            return;
        }
        if (sanity == null || !sanity.accepted()) {
            lastStatus = sanity == null ? "reject-sanity"
                    : "reject-" + sanity.verdict.name().toLowerCase().replace("reject_", "");

            return;
        }
        // ---- Stage 2 (DISABLED): rest-projection rescue ----
        // TipCompensator.ENABLED ships false. When you enable it after the
        // validation plan in that class, accepted-but-deviated solves get
        // projected back to rest here and fused with inflated R.
        Pose measPose = rd.pedroPose;
        // XY source override: MT1's independent per-tag-averaged solve (the yaw
        // is still NaN-masked — XY-only). MT1's pose carries the SAME LL_* mapping
        // as MT2's (toPedroPose already applied by poll).
        if (USE_MT1_XY && rd.mt1Pedro != null) {
            measPose = rd.mt1Pedro;
            lastXySrc = "MT1";
        }
        // (No code-side camera lever-arm compensation: the Limelight mount
        // geometry lives in the LL web UI / LL init method, so botpose is
        // already robot-center — see the note at USE_MT1_XY.)
        double compRScale = 1.0;
        if (TipCompensator.ENABLED) {
            try {
                TipCompensator.Result comp = TipCompensator.compensate(
                        rd.pedroPose, rd.botPitchDeg, rd.botRollDeg, rd.botZMeters);
                if (comp == null) {
                    lastStatus = "reject-uncompensatable";
        
                    return;
                }
                measPose = comp.pose;
                compRScale = Math.max(comp.rScale, 1.0);
                lastStatus = "compensated";
            } catch (Exception e) {
                lastStatus = "compensate-exception";
    
                return;
            }
        }
        // BIOBUZZ cluster-only switch: conservative Pinpoint-only fallback.
        // Ships false (you localize off the clusters); Stage 1 still drops tipped ones.
        if (REJECT_CLUSTER_ONLY && rd.clusterOnly) {
            lastStatus = "reject-cluster";

            return;
        }
        // Rest-signature backstop: GOOD (cell at rest) reports flat ~= 0 because
        // the 30deg lives in the .fmap entries. Off-rest cells tilt the solve.
        // Mirrors TipSanityFilter (the real gate) — see note on the tunables.
        if (MAX_BOT_PITCH_DEG > 0 && Math.abs(rd.botPitchDeg) > MAX_BOT_PITCH_DEG) {
            lastStatus = "reject-tilt";

            return;
        }
        if (MAX_BOT_ROLL_DEG > 0 && Math.abs(rd.botRollDeg) > MAX_BOT_ROLL_DEG) {
            lastStatus = "reject-tilt";

            return;
        }
        //filters out tags not at the height they should be (untipped)
//        if (!(rd.botZMeters >= BOT_Z_MIN_M && rd.botZMeters <= BOT_Z_MAX_M)) {
//            lastStatus = "reject-height";
//
//            return;
//        }
        if (rd.pedroPose.x() < -FIELD_MARGIN_IN || rd.pedroPose.x() > 144.0 + FIELD_MARGIN_IN
                || rd.pedroPose.y() < -FIELD_MARGIN_IN || rd.pedroPose.y() > 144.0 + FIELD_MARGIN_IN) {
            lastStatus = "reject-off-field";

            return;
        }
        // Residuals: XY residual always; yaw residual only when MT1 present.
        double dx = measPose.x() - fused.x();
        double dy = measPose.y() - fused.y();
        double dxy = Math.hypot(dx, dy);
        lastResidualIn = dxy;
        lastDx = dx;
        lastDy = dy;
        
        // HARD REJECT: if the Kalman filter has diverged catastrophically (residual > 2x field size),
        // reject the frame. This prevents MT2 teleportation from ripping the filter apart.
        // A residual > 288in means the fused pose is way outside reality; fusing it only makes it worse.
        if (dxy > 288.0) {  // 2 * 144in field diagonal
            lastStatus = "reject-residual-huge";
            return;
        }
        double dhDeg = Double.NaN;
        if (rd.hasMt1) {
            // SIGNED innovation first: + = vision CCW of fused, − = CW. |.| feeds
            // the R ramp; the sign tells flips (±180) apart from noise on Panels.
            double innovDeg = Math.toDegrees(
                    Angle.normalizeSigned(rd.mt1HeadingRad - fused.heading()));
            lastYawInnovDeg = innovDeg;
            lastMt1RawDeg = Math.toDegrees(Angle.normalize(rd.mt1HeadingRad));
            dhDeg = Math.abs(innovDeg);
            lastYawResidualDeg = dhDeg;
        } else {
            lastYawInnovDeg = Double.NaN;
            lastMt1RawDeg = Double.NaN;
            lastYawResidualDeg = Double.NaN;
        }
        // XY "teleport" is SOFT: no reject here. Big dxy flows into the R ramp
        // below (unclamped quadratic to MAX_TOTAL_SCALE) and flags -bigjump in
        // the fused status. Yaw is decided independently in the MT1 path — a bad
        // MT2 XY never kills a good MT1 yaw or vice versa.

        // ---- soft R scaling: XY path (MT2) ----
        double speedIps = 0, omegaRps = 0;
        if (vel != null) {
            speedIps = Math.hypot(vel.vx, vel.vy);
            omegaRps = Math.abs(vel.omega);
            if (!Double.isFinite(speedIps)) speedIps = 0;
            if (!Double.isFinite(omegaRps)) omegaRps = 0;
        }

        // ADAPTIVE RESIDUAL GATE: when robot is moving slowly, reject large jumps.
        // If velocity is < 10 in/s and residual > 36in, something is very wrong
        // (either MT2 teleported or the filter state is way off).
        if (speedIps < 10.0 && dxy > 36.0) {
            lastStatus = "reject-residual-stationary";
            return;
        }

        double sXY = 1.0;

        // XY source quality: MT1-position fallback (no MT2) is less trusted.
        if (!rd.hasMt2) sXY *= MT1_XY_PENALTY;
        if (rd.tagCount < 2) {
            // tagCount is 0/1 here (2+ skips this block); 0 = LL gave no count.
            sXY *= SINGLE_TAG_PENALTY * (rd.tagCount == 1 ? 1.0 : 0.5);
        }
        // Narrow multi-tag baseline triangulates poorly.
        if (rd.hasMt2 && rd.tagCount >= 2 && rd.spanM > 0 && rd.spanM < SPAN_REF_M) {
            sXY *= 1.0 + 2.0 * (1.0 - rd.spanM / SPAN_REF_M);
        }
        // Distance falloff past close range.
        if (rd.avgDistM > CLOSE_DIST_M) {
            sXY *= 1.0 + DIST_SLOPE_PER_M * (rd.avgDistM - CLOSE_DIST_M);
        }
        // Tiny tags in frame.
        if (rd.avgAreaPct > 0 && rd.avgAreaPct < AREA_REF_PCT) {
            sXY *= 1.0 + (AREA_REF_PCT / Math.max(rd.avgAreaPct, 0.05) - 1.0);
        }
        // Motion blur / seed lag: quadratic in speed and turn rate.
        double speedK = speedIps / Math.max(SPEED_REF_IPS, 1e-6);
        sXY *= 1.0 + speedK * speedK;
        double turnK = omegaRps / Math.max(TURN_REF_RPS, 1e-6);
        sXY *= 1.0 + turnK * turnK;
        // Seed-agreement penalty was computed EARLY (before the reject gates —
        // see top of correct()): a wrong seed poisons the MT2 XY solve, so
        // disagreement scales sXY (never heading — heading doesn't fuse from MT2).
        sXY *= seedPenalty;
        // XY innovation: smooth ramp, never a cliff, never a cap — past the knee
        // the penalty keeps growing quadratically into the global clamp, so a
        // 36in jump fuses at ~23x R and a 48in jump at ~44x instead of dropping.
        if (dxy > RESIDUAL_FULL_IN) {
            double t = (dxy - RESIDUAL_FULL_IN)
                    / Math.max(RESIDUAL_MAX_IN - RESIDUAL_FULL_IN, 1e-6);
            sXY *= 1.0 + (RESIDUAL_MAX_PENALTY - 1.0) * t * t;
        }

        // ---- MT1 yaw path: strict gates, then its own R ----
        // A flipped single-tag solve is worse than none: default is SKIP yaw.
        // MT1_YAW_FUSING=false disables the axis entirely (Pedro heading-fuse
        // corruption — see the tunable's javadoc).
        boolean fuseYaw = rd.hasMt1 && MT1_YAW_FUSING;
        String yawSkip = rd.hasMt1 && !MT1_YAW_FUSING ? "yaw-off" : "none";
        double yawResidDeg = dhDeg;
        if (fuseYaw && rd.tagCount < YAW_MIN_TAGS) {
            fuseYaw = false;
            yawSkip = "tags";
        }
        if (fuseYaw && speedIps > YAW_MAX_SPEED_IPS) {
            fuseYaw = false;
            yawSkip = "speed";
        }
        if (fuseYaw && omegaRps > YAW_MAX_OMEGA_RPS) {
            fuseYaw = false;
            yawSkip = "turn";
        }
        if (fuseYaw && rd.avgDistM > YAW_MAX_DIST_M) {
            fuseYaw = false;
            yawSkip = "dist";
        }
        // Corruption guard: a >90deg yaw innovation is a flipped/garbage solve
        // or an already-corrupted state — never a real heading measurement.
        // (With a healthy state, MT1 innovations stay under ~30deg.) Skip it:
        // fusing it either poisons the heading or, against a poisoned state,
        // "confirms" the poison via the wrap-shortest path.
        if (fuseYaw && Double.isFinite(dhDeg) && dhDeg > 90.0) {
            fuseYaw = false;
            yawSkip = "innov";
        }
        // Yaw residual is SOFT like XY: no skip here, the sH ramp below (same
        // unclamped shape) de-weights it. tags/speed/turn/dist above still skip
        // the YAW AXIS only — XY always fuses.
        lastYawFused = fuseYaw;
        lastYawSkip = fuseYaw ? "fused" : yawSkip;
        double rH = Double.NaN;
        if (fuseYaw) {
            // Slow pull only: base 18deg stddev + floor + residual ramp.
            // NOTE: sH starts at 1.0 here — the old motion-blur/turn terms applied
            // to MT2-echo yaw and must NOT touch independent MT1 yaw. MT1 yaw at
            // speed is handled by the YAW_MAX_* hard gates above, not soft scaling.
            double sH = 1.0;
            if (Double.isFinite(yawResidDeg) && yawResidDeg > RESIDUAL_FULL_IN) {
                double t = yawResidDeg / Math.max(YAW_MAX_RESIDUAL_DEG, 1e-6);
                sH *= 1.0 + (RESIDUAL_MAX_PENALTY - 1.0) * t * t;
            }
            rH = BASE_R_HEADING * Math.min(sH, MAX_TOTAL_SCALE);
            rH = Math.max(rH, MT1_HEADING_R);
            rH = Math.min(rH * compRScale, BASE_R_HEADING * MAX_TOTAL_SCALE);
        }
        lastYawR = rH;

        // Limelight self-reported stddev: trusted 1:1 only when small (XY path).
        double rXY = BASE_R_XY * Math.min(sXY, MAX_TOTAL_SCALE);
        if (rd.stddevXYIn > 0 && Double.isFinite(rd.stddevXYIn)) {
            if (rd.stddevXYIn <= STDDEV_TRUST_CAP_IN) {
                double v = rd.stddevXYIn * rd.stddevXYIn;
                // Blend: LL variance and our scaled base, take the WORSE (larger).
                rXY = Math.max(v, Math.min(rXY, BASE_R_XY * MAX_TOTAL_SCALE));
            } else {
                rXY *= 1.0 + (rd.stddevXYIn / STDDEV_TRUST_CAP_IN - 1.0);
                rXY = Math.min(rXY, BASE_R_XY * MAX_TOTAL_SCALE);
            }
        }
        // Stage 2 rescue inflates XY R for its residual pivot-arc error (1.0 when off).
        rXY = Math.min(rXY * compRScale, BASE_R_XY * MAX_TOTAL_SCALE);

        lastScale = rXY / BASE_R_XY;
        lastRx = rXY;
        lastRh = rH;

        // Diagnostic/kill switch — see the FUSE_XY javadoc.
        if (!FUSE_XY) {
            lastStatus = "fuse-off";
            return;
        }

        // ===== HUNT MODE: route through Pedro's addMeasurement =====
        // The instrumented subclass (swapped in by RobotMain) snapshots the
        // history around every call and dumps any heading change > 2deg to
        // logcat (IronLog-Fuse) + Fuse/injDh / injBefore / injAfter telemetry.
        if (USE_PEDRO_FUSE) {
            double huntLatMs = Math.min(Math.max(rd.latencyMs, 0.0), MAX_LATENCY_MS);
            long huntStampNs = System.nanoTime() - (long) (huntLatMs * 1.0e6);
            try {
                fuse(fusion, measPose, rd.mt1HeadingRad, fused.heading(), fuseYaw,
                        huntStampNs, rXY, rH);
                if (!"compensated".equals(lastStatus)) {
                    String base = fuseYaw ? "fused-xy-yaw" : "fused-xy";
                    lastStatus = "pedro-" + (dxy > MAX_JUMP_XY_IN ? base + "-bigjump" : base);
                }
            } catch (Exception e) {
                lastStatus = "pedro-addMeasurement-exception";
            }
            return;
        }

        // ===== DIRECT POSE CORRECTION (bypasses Pedro's addMeasurement) =====
        // EMPIRICAL VERDICT (logs 23-04/23-22/23-47/00-23): ANY addMeasurement
        // call injects a one-time ~+166deg heading offset — even XY-only fuses
        // with the yaw NaN-masked (bytecode-proven inert in the gain math!),
        // and the offset is CONSTANT thereafter (fused = pinpoint + 166.7
        // through the whole rotation sequence, log 00-23-05 @2499ms). The
        // 00-23-59 run with FUSE_XY off proved the Pinpoint innocent: fused ≡
        // pinpoint to 0.1 through the full CW/CCW rotation sequence. The
        // corruption lives inside Pedro's fuse/history machinery, so we stop
        // using it: blend the accepted vision XY directly into the pose via
        // setPose (no history walk, no gain matrix), and NEVER touch the
        // heading — it rides Pinpoint-only.
        //
        // Cost of dropping the back-dated stamp: ~latencyMs of lag, ~0.1in at
        // 30in/s — negligible next to Pedro's corruption risk.
        double alphaXY = FUSE_GAIN * (BASE_R_XY / Math.max(rXY, BASE_R_XY));
        try {
            Pose fusedNow = follower.pose();
            double h = fusedNow.heading();
            // Optional MT1-yaw drift correction — the SAFE injection path: a
            // direct heading write via setPose, no Pedro fuse involved.
            // Default 0 = off. Only corrects on sane innovations (|innov| <=
            // 90deg; the flip guard above already rejects wilder ones).
            // Independent of MT1_YAW_FUSING (that switch gates the dead
            // Pedro-fuse path). Raise ~0.02-0.05 after a drift bench test.
            if (FUSE_GAIN_H > 0 && Double.isFinite(lastYawInnovDeg)
                    && Math.abs(lastYawInnovDeg) <= 90.0) {
                h += FUSE_GAIN_H * Math.toRadians(lastYawInnovDeg);
            }
            Pose blended = new Pose(
                    fusedNow.x() + alphaXY * lastDx,
                    fusedNow.y() + alphaXY * lastDy,
                    h);
            follower.setPose(blended);
            // Preserve the compensated marker when Stage 2 actually rescued.
            if (!"compensated".equals(lastStatus)) {
                lastStatus = dxy > MAX_JUMP_XY_IN ? "fused-xy-bigjump" : "fused-xy";
            }
        } catch (Exception e) {
            lastStatus = "setPose-exception";
        }
    }

    /**
     * Once-per-loop Panels snapshot (addData only — {@link RobotMain#flushTelemetry()}
     * calls this right before the single {@code update()}). Includes the latest
     * {@link TipSanityFilter} decision so {@code Tip/*} also appears exactly once.
     * Safe to call when inactive (reports the sticky last-loop snapshot).
     */
    public void report() {
        TipSanityFilter.Decision sanity = tipFilter.lastDecision();
        if (sanity != null) {
            RobotMain.DashTelemetry.addData("Tip/verdict", sanity.verdict);
            RobotMain.DashTelemetry.addData("Tip/pitchDeg", "%.1f", sanity.pitchDeg);
            RobotMain.DashTelemetry.addData("Tip/rollDeg", "%.1f", sanity.rollDeg);
            RobotMain.DashTelemetry.addData("Tip/zM", "%.3f", sanity.zMeters);
            if (Double.isFinite(sanity.speedIps)) {
                RobotMain.DashTelemetry.addData("Tip/solveSpeedIps", "%.1f", sanity.speedIps);
            }
        }
        RobotMain.DashTelemetry.addData("Fusion/status", lastStatus);
        RobotMain.DashTelemetry.addData("Fusion/active", isActive());
        RobotMain.DashTelemetry.addData("Fusion/xySrc", lastXySrc);
        if (lastVision != null) {
            RobotMain.DashTelemetry.addData("Fusion/visionX", "%.2f", lastVision.x());
            RobotMain.DashTelemetry.addData("Fusion/visionY", "%.2f", lastVision.y());
            RobotMain.DashTelemetry.addData("Fusion/visionHdeg", "%.1f",
                    Math.toDegrees(Angle.normalize(
                            lastVision.heading())));
            RobotMain.DashTelemetry.addData("Fusion/mt2", lastMt2);
            RobotMain.DashTelemetry.addData("Fusion/tags", lastTags);
            RobotMain.DashTelemetry.addData("Fusion/clusterOnly", lastVisionClusterOnly);
        }
        // MT1's independent solve XY (the per-tag PnP average, no seed) —
        // compare against pinpointXY: if sane while visionX/Y (MT2) collapses
        // depth, flip USE_MT1_XY.
        if (lastMt1Pedro != null) {
            RobotMain.DashTelemetry.addData("Fusion/mt1X", "%.2f", lastMt1Pedro.x());
            RobotMain.DashTelemetry.addData("Fusion/mt1Y", "%.2f", lastMt1Pedro.y());
        }
        RobotMain.DashTelemetry.addData("Fusion/scale", "%.2fx", lastScale);
        RobotMain.DashTelemetry.addData("Fusion/Rxy", "%.2f", lastRx);
        RobotMain.DashTelemetry.addData("Fusion/Rh", Double.isFinite(lastRh) ? String.format(Locale.US, "%.4f", lastRh) : "skip");
        RobotMain.DashTelemetry.addData("Fusion/yaw", lastYawSkip
                + (Double.isFinite(lastYawResidualDeg)
                        ? String.format(Locale.US, " res=%.1f", lastYawResidualDeg) : ""));
        if (Double.isFinite(lastYawInnovDeg)) {
            RobotMain.DashTelemetry.addData("Fusion/yawInnov", "%.1f", lastYawInnovDeg);
        }
        if (Double.isFinite(lastMt1RawDeg)) {
            RobotMain.DashTelemetry.addData("Fusion/mt1RawDeg", "%.1f", lastMt1RawDeg);
        }
        // Raw solve yaws straight off LL (no LL_* mapping) + the seed sent:
        // the offset calibration rig. Park at known field heading H, then
        // offset = norm180(H − mt1LlRaw). mt2LlRaw should ~= seed (echo check).
        if (Double.isFinite(lastMt1LlRawDeg)) {
            RobotMain.DashTelemetry.addData("Fusion/mt1LlRawDeg", "%.1f", lastMt1LlRawDeg);
        }
        if (Double.isFinite(lastMt2LlRawDeg)) {
            RobotMain.DashTelemetry.addData("Fusion/mt2LlRawDeg", "%.1f", lastMt2LlRawDeg);
        }
        if (Double.isFinite(lastSeedSentDeg)) {
            RobotMain.DashTelemetry.addData("Fusion/seedSentDeg", "%.1f", lastSeedSentDeg);
        }
        RobotMain.DashTelemetry.addData("Fusion/seedAgrDeg", Double.isFinite(lastSeedAgrDeg)
                ? String.format(Locale.US, "%.1f", lastSeedAgrDeg) : "n/a");
        if (Double.isFinite(lastResidualIn)) {
            RobotMain.DashTelemetry.addData("Fusion/residualIn", "%.2f", lastResidualIn);
        }
        if (Double.isFinite(lastDx) && Double.isFinite(lastDy)) {
            RobotMain.DashTelemetry.addData("Fusion/dxy", "%.1f, %.1f", lastDx, lastDy);
        }
        Pose fused = null;
        try {
            fused = follower.pose();
        } catch (Exception ignored) {
        }
        if (fused != null) {
            RobotMain.DashTelemetry.addData("Fusion/fusedX", "%.2f", fused.x());
            RobotMain.DashTelemetry.addData("Fusion/fusedY", "%.2f", fused.y());
            RobotMain.DashTelemetry.addData("Fusion/fusedHdeg", "%.1f",
                    Math.toDegrees(Angle.normalize(fused.heading())));
        }
        // Decisive twist diagnostic: the Pinpoint's RAW pose every loop.
        // pinpointHdeg jumping at a twist = IMU/hardware side; holding steady
        // while fusedHdeg twists = Pedro's fuse/history re-propagation.
        if (deadReckoning != null) {
            try {
                Pose pp = deadReckoning.pose();
                if (pp != null) {
                    RobotMain.DashTelemetry.addData("Fusion/pinpointX", "%.2f", pp.x());
                    RobotMain.DashTelemetry.addData("Fusion/pinpointY", "%.2f", pp.y());
                    RobotMain.DashTelemetry.addData("Fusion/pinpointHdeg", "%.2f",
                            Math.toDegrees(Angle.normalize(pp.heading())));
                }
            } catch (Exception ignored) {
            }
        }
    }

    // ---- accessors (tuning OpModes / tests) ----
    public String lastStatus() {
        return lastStatus;
    }

    public double lastScale() {
        return lastScale;
    }

    @Nullable
    public Pose lastVisionPose() {
        return lastVision;
    }

    /**
     * Raw MT1 full solve in Pedro frame for the dashboard overlay (yellow).
     * Null when MT1 was absent/disabled this loop. Snapshot from the last
     * accepted frame — NOT gated by the MT1-yaw fuse gates (a rejected yaw
     * still draws, so you can SEE why the filter distrusts it).
     */
    @Nullable
    public Pose lastMt1Pedro() {
        return lastMt1Pedro;
    }

    /**
     * Raw MT2 full solve in Pedro frame for the dashboard overlay (green).
     * Null when MT2 was absent this loop.
     */
    @Nullable
    public Pose lastMt2Pedro() {
        return lastMt2Pedro;
    }

    /**
     * Axis-masked fuse. MT2 yaw is NEVER fused (NaN-masked): it is seed echo,
     * not measurement — fusing it double-counts the gyro in a feedback loop.
     * XY always fuses from measPose; yaw fuses from the independent MT1 solve
     * only when fuseYaw passed the gates. FusionLocalizer skips NaN components
     * per axis (verified in 3.0.1 bytecode). Single addMeasurement call carries
     * both axes; the yaw component is NaN when yaw is skipped.
     *
     * <p><b>Angle wrap:</b> the MT1 yaw is wrapped to ±π of the CURRENT fused
     * heading before fusing. The Kalman innovation is angular — feeding an
     * unwrapped measurement (e.g. fused at 179deg, vision mapped at −179deg)
     * would look like a ~358deg jump instead of a 2deg correction. Wrapping
     * here makes the innovation the true shortest-arc error; the residual R
     * ramp downstream already saw the wrapped |residual|.
     */
    private void fuse(FusionLocalizer fusion, Pose xyPose, double mt1HeadingRad,
                      double fusedHeadingRad,
                      boolean fuseYaw, long stampNs, double rXY, double rH) {
        // Yaw-skipped frames pass NaN — bytecode-verified in FusionLocalizer
        // 3.0.1: the per-axis mask (hasX/hasY/hasH) zeroes the innovation AND
        // the gain's H-row, so a masked fuse cannot move the heading. DO NOT
        // pass a finite "neutral" yaw instead: runs 23-04/23-22 proved any
        // finite-yaw measurement corrupts the heading inside Pedro's
        // fuse/history re-propagation (90 -> 255.6deg on an unmoved robot).
        double yaw = Double.NaN;
        if (fuseYaw && Double.isFinite(mt1HeadingRad)) {
            yaw = fusedHeadingRad + Angle.normalizeSigned(mt1HeadingRad - fusedHeadingRad);
        }
        // R MUST stay finite on ALL axes, even the NaN-masked yaw. Verified in
        // FusionLocalizer 3.0.1 bytecode: a NaN in the R diagonal poisons the
        // whole correction — S = P + R gets NaN at [2,2], S.invert() returns
        // all-NaN, K = P * S^-1 goes all-NaN, and the XY rows pick up NaN via
        // the yaw column (NaN * 0 = NaN). One yaw-skipped fuse (the COMMON
        // case: single tag, moving, far — i.e. the FIRST tag you ever see)
        // NaNs the fused pose forever, freezing the field at its last good
        // frame. The yaw R value is irrelevant when yaw is masked (H mask
        // zeroes the yaw row), so use a finite placeholder; only a finite yaw
        // measurement ever fuses.
        double yawR;
        if (fuseYaw && Double.isFinite(rH)) {
            yawR = rH;
        } else {
            yawR = BASE_R_HEADING;
        }
        double safeRXY = Double.isFinite(rXY) ? rXY : BASE_R_XY;
        fusion.addMeasurement(
                new Pose(xyPose.x(), xyPose.y(), yaw),
                stampNs,
                new Pose(safeRXY, safeRXY, yawR));
    }

    private static double norm180Deg(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }

    /**
     * Convert Pedro frame heading to Limelight frame heading for seeding MT2.
     * Pipeline: Pedro (0=+X) -> Field (0=+Y, -90deg) -> Limelight (apply offset & sign).
     * This ensures the Limelight gets its heading in its own native coordinate frame.
     */
    private static double pedroHeadingToLimelightDeg(double pedroHeadingRad) {
        // Convert Pedro -> Field
        double fieldHeadingRad = PedroFieldBridge.pedroHeadingToField(pedroHeadingRad);
        double fieldHeadingDeg = Math.toDegrees(fieldHeadingRad);
        // Convert Field -> Limelight (using Vision's mapping)
        double llHeadingDeg = norm180Deg((fieldHeadingDeg - Vision.LL_YAW_OFFSET_DEG) * Vision.LL_YAW_SIGN);
        return llHeadingDeg;
    }
}
