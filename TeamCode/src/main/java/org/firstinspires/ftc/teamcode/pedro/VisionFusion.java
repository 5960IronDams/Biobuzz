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
import org.firstinspires.ftc.teamcode.killerwatts.Vision;

/**
 * Vision corrector: feeds Limelight MegaTag solves into Pedro's
 * {@link FusionLocalizer} Kalman filter via {@code addMeasurement()}.
 *
 * <p>Call order per loop (see {@link RobotMain#RobotRunPeriodic}):
 * <pre>
 * follower.update();      // predict step (Pinpoint) + fuse owner
 * visionFusion.correct(); // zero or one addMeasurement()
 * </pre>
 * The follower owns the prediction; this class owns the correction. Nothing
 * here calls {@code setPose()}, so Foresight's velocity estimate is never
 * teleported — corrections blend with gain K = P/(P+R) and re-propagate
 * through the timestamp history.
 *
 * <p><b>Soft gating, not hard gating.</b> Almost every reading is fused; bad
 * conditions inflate the per-measurement R (measurement-noise variance passed
 * as the 3rd {@code addMeasurement} arg) so the filter trusts vision less
 * instead of dropping it. Only garbage is skipped outright: no solve, NaNs,
 * off-field poses, stale frames, impossible jumps on every axis at once. This
 * keeps corrections flowing through tag deserts instead of going blind.
 *
 * <p><b>Heading blending.</b> MT2 yaw is only as good as the gyro seed behind
 * it. At speed the seed lags and MT2 yaw smears, so the heading component is
 * up-weighted (higher R) with motion while XY stays trusted — position from
 * tags is still good when yaw isn't. MT1 yaw (single-tag ambiguity) is trusted
 * even less.
 *
 * <p>All tunables are Panels-live ({@code Panels -> VisionFusion}).
 *
 *
 */
     /** Fused XY creeps slightly toward vision; heading essentially ignores it. Same reading standing still, 2 tags, close: s ≈ 1, R ≈ 9, full correction. That's the whole philosophy — bad conditions bend the blend, they don't shut it off, except the four garbage gates (NaN, stale, off-field, whole-pose teleport).
    Knobs worth knowing by feel:
    SINGLE_TAG_PENALTY, MT1_XY_PENALTY, MT1_HEADING_R — source quality. MT1 yaw ambiguity is why heading gets a floor, not a multiplier.
    DIST_SLOPE_PER_M, AREA_REF_PCT, SPAN_REF_M — geometry quality (far / small / narrow-baseline tags triangulate poorly).
    SPEED_REF_IPS, TURN_REF_RPS, HEADING_* — motion blur + MT2 seed lag. Note heading has an extra linear-in-speed term: even driving straight fast smears the gyro seed MT2 depends on, while tag position stays fine. That's why XY and heading split.
    RESIDUAL_* — innovation gate as a ramp (6"→24", up to ×9), not a cliff. A real drift correction (medium residual) still fuses; a hallucination (huge residual) gets muffled.
    STDDEV_TRUST_CAP_IN — Limelight's own stddev is only believed when small; when LL itself says "I'm unsure," that increases R instead.
    MAX_TOTAL_SCALE = 64 — no single reading can exceed 64× base. Safety net against multiplied penalties exploding to "never trust vision at all."
    Tuning it on-field (Fusion Tune)
    Watch three Panels lines while driving:
    1.Fusion/scale pinned at 1.0x everywhere → you're over-trusting; raise BASE_R_XY.
    2.Fusion/residualIn shrinks after tags appear → fusion is working. Residual stuck large → under-trusting; lower BASE_R_XY.
    3.Pose jumps when a tag pops in → raise base R or lower RESIDUAL_MAX_PENALTY.
    And the A/B test: flip VisionFusion.ENABLED live — same drive, Pinpoint-only vs fused. If fused is worse, don't touch the penalties first; re-verify the LL_* frame map in  Vision.java. A mirrored axis looks exactly like "Kalman is broken" but no R value can fix a wrong frame.
  */
@Configurable
public class VisionFusion {

    // ================= master switch =================
    /** Set false (Panels) to run Pinpoint-only, e.g. while debugging vision. */
    public static boolean ENABLED = true;

    // ================= absolute reject gates (garbage only) =================
    /** Reject frames older than this (LL poll + USB + loop delay), ms. */
    public static double MAX_STALENESS_MS = 350.0;
    /** Reject poses outside the field plus this margin, inches. */
    public static double FIELD_MARGIN_IN = 24.0;
    /**
     * Reject when BOTH |dXY| exceeds this AND |dHeading| exceeds
     * {@link #MAX_JUMP_HEADING_DEG}. Single-axis jumps still fuse (de-weighted
     * below) — a real correction usually moves one axis more than the other,
     * while garbage jumps everywhere at once.
     */
    public static double MAX_JUMP_XY_IN = 36.0;
    /** Heading half of the impossible-jump gate, degrees. */
    public static double MAX_JUMP_HEADING_DEG = 45.0;
    /** Clamp on total pipeline latency used for back-dating, ms. */
    public static double MAX_LATENCY_MS = 250.0;

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
     */
    public static double MAX_BOT_PITCH_DEG = 12.0;
    /** Same for |botpose roll|, degrees. 0 disables. */
    public static double MAX_BOT_ROLL_DEG = 12.0;
    /** Reject when reported camera height is outside [min, max], meters. */
    public static double BOT_Z_MIN_M = 0.05;
    public static double BOT_Z_MAX_M = 0.60;

    // ================= base trust (variances, NOT stddevs) =================
    // R is digested as Matrix.diag(x, y, heading). These match
    // Constants.fusionDefaultMeasurementNoise(); the per-reading scaler below
    // multiplies them. Heading is rad^2: 6deg stddev -> 0.011 var.
    /** Base XY measurement variance, in^2 (3in stddev). */
    public static double BASE_R_XY = 9.0;
    /** Base heading measurement variance, rad^2 (6deg stddev). */
    public static double BASE_R_HEADING = 0.011;

    // ================= soft de-weight knobs =================
    // Each factor is multiplicative on R, i.e. trust /= factor.
    /** MT1 fallback: XY R x this (MT1 yaw handled by MT1_HEADING_R below). */
    public static double MT1_XY_PENALTY = 4.0;
    /** MT1 fallback heading variance floor, rad^2 (~30deg stddev). */
    public static double MT1_HEADING_R = 0.27;
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
    /** R x per (in/s of planar speed / SPEED_REF_IPS), quadratic. */
    public static double SPEED_REF_IPS = 40.0;
    /** R x per (rad/s of turn rate / TURN_REF_RPS), quadratic, XY only. */
    public static double TURN_REF_RPS = 2.0;
    /** Heading R x per (rad/s of turn rate / HEADING_TURN_REF_RPS), quadratic. */
    public static double HEADING_TURN_REF_RPS = 1.0;
    /** Heading R x when moving fast even if not turning (MT2 seed lag), linear. */
    public static double HEADING_SPEED_REF_IPS = 30.0;
    /** R x when tag area is tiny (far/small in frame), ramps below AREA_REF_PCT. */
    public static double AREA_REF_PCT = 0.5;
    /** R x when the tag baseline is narrow (poor triangulation), ramps below SPAN_REF_M. */
    public static double SPAN_REF_M = 1.0;
    // Innovation (vision - fused residual) de-weight: smooth, never a cliff.
    /** Residual below this fuses at full weight, inches. */
    public static double RESIDUAL_FULL_IN = 6.0;
    /** Residual at/above this is heavily down-weighted, inches. */
    public static double RESIDUAL_MAX_IN = 24.0;
    /** Max extra R x at RESIDUAL_MAX_IN (ramps quadratically between). */
    public static double RESIDUAL_MAX_PENALTY = 9.0;
    /** Global clamp: no single reading may scale R beyond this. */
    public static double MAX_TOTAL_SCALE = 64.0;

    // ---- last-loop debug snapshot (telemetry) ----
    private String lastStatus = "no-vision";
    private double lastScale = 1.0;
    private double lastRx = BASE_R_XY, lastRh = BASE_R_HEADING;
    private Pose lastVision = null;
    private boolean lastMt2 = false;
    private boolean lastVisionClusterOnly = false;
    private int lastTags = 0;
    private double lastResidualIn = Double.NaN;

    private final Follower follower;
    @Nullable
    private final Vision vision;
    /** Stage 1 sanity filter (stateless except last-accepted teleport check). */
    private final TipSanityFilter tipFilter = new TipSanityFilter();

    public VisionFusion(Follower follower, @Nullable Vision vision) {
        if (follower == null) throw new IllegalArgumentException("VisionFusion needs a non-null Follower");
        this.follower = follower;
        this.vision = vision;
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

        // Seed MT2 with the CURRENT fused field heading (0 = +Y/up-field).
        double fieldYawDeg = Math.toDegrees(PedroFieldBridge.pedroHeadingToField(fused.heading()));
        Vision.Reading rd;
        try {
            rd = vision.poll(fieldYawDeg);
        } catch (Exception e) {
            lastStatus = "poll-exception";
            return;
        }
        if (rd == null || rd.pedroPose == null) {
            lastStatus = "no-solve";
            report();
            return;
        }
        lastVision = rd.pedroPose;
        lastMt2 = rd.isMt2;
        lastVisionClusterOnly = rd.clusterOnly;
        lastTags = rd.tagCount;

        // ---- absolute reject gates (garbage only) ----
        if (!Double.isFinite(rd.pedroPose.x()) || !Double.isFinite(rd.pedroPose.y())
                || !Double.isFinite(rd.pedroPose.heading())) {
            lastStatus = "reject-nan";
            report();
            return;
        }
        if (rd.stalenessMs > MAX_STALENESS_MS) {
            lastStatus = "reject-stale";
            report();
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
            report();
            return;
        }
        if (sanity == null || !sanity.accepted()) {
            lastStatus = sanity == null ? "reject-sanity"
                    : "reject-" + sanity.verdict.name().toLowerCase().replace("reject_", "");
            report();
            return;
        }
        // ---- Stage 2 (DISABLED): rest-projection rescue ----
        // TipCompensator.ENABLED ships false. When you enable it after the
        // validation plan in that class, accepted-but-deviated solves get
        // projected back to rest here and fused with inflated R.
        Pose measPose = rd.pedroPose;
        double compRScale = 1.0;
        if (TipCompensator.ENABLED) {
            try {
                TipCompensator.Result comp = TipCompensator.compensate(
                        rd.pedroPose, rd.botPitchDeg, rd.botRollDeg, rd.botZMeters);
                if (comp == null) {
                    lastStatus = "reject-uncompensatable";
                    report();
                    return;
                }
                measPose = comp.pose;
                compRScale = Math.max(comp.rScale, 1.0);
                lastStatus = "compensated";
            } catch (Exception e) {
                lastStatus = "compensate-exception";
                report();
                return;
            }
        }
        // BIOBUZZ cluster-only switch: conservative Pinpoint-only fallback.
        // Ships false (you localize off the clusters); Stage 1 still drops tipped ones.
        if (REJECT_CLUSTER_ONLY && rd.clusterOnly) {
            lastStatus = "reject-cluster";
            report();
            return;
        }
        // Rest-signature backstop: GOOD (cell at rest) reports flat ~= 0 because
        // the 30deg lives in the .fmap entries. Off-rest cells tilt the solve.
        // Mirrors TipSanityFilter (the real gate) — see note on the tunables.
        if (MAX_BOT_PITCH_DEG > 0 && Math.abs(rd.botPitchDeg) > MAX_BOT_PITCH_DEG) {
            lastStatus = "reject-tilt";
            report();
            return;
        }
        if (MAX_BOT_ROLL_DEG > 0 && Math.abs(rd.botRollDeg) > MAX_BOT_ROLL_DEG) {
            lastStatus = "reject-tilt";
            report();
            return;
        }
        if (!(rd.botZMeters >= BOT_Z_MIN_M && rd.botZMeters <= BOT_Z_MAX_M)) {
            lastStatus = "reject-height";
            report();
            return;
        }
        if (rd.pedroPose.x() < -FIELD_MARGIN_IN || rd.pedroPose.x() > 144.0 + FIELD_MARGIN_IN
                || rd.pedroPose.y() < -FIELD_MARGIN_IN || rd.pedroPose.y() > 144.0 + FIELD_MARGIN_IN) {
            lastStatus = "reject-off-field";
            report();
            return;
        }
        // Residuals are computed against the FUSED pose but the MEASUREMENT is
        // measPose (== raw now; compensated once Stage 2 is enabled/tested).
        double dx = measPose.x() - fused.x();
        double dy = measPose.y() - fused.y();
        double dxy = Math.hypot(dx, dy);
        double dhDeg = Math.abs(Math.toDegrees(
                Angle.normalizeSigned(measPose.heading() - fused.heading())));
        lastResidualIn = dxy;
        if (dxy > MAX_JUMP_XY_IN && dhDeg > MAX_JUMP_HEADING_DEG) {
            lastStatus = "reject-jump";
            report();
            return;
        }

        // ---- soft R scaling (everything below multiplies trust down) ----
        double speedIps = 0, omegaRps = 0;
        if (vel != null) {
            speedIps = Math.hypot(vel.vx, vel.vy);
            omegaRps = Math.abs(vel.omega);
            if (!Double.isFinite(speedIps)) speedIps = 0;
            if (!Double.isFinite(omegaRps)) omegaRps = 0;
        }

        double sXY = 1.0;
        double sH = 1.0;

        // Source quality: MT1 fallback + single tag.
        if (!rd.isMt2) sXY *= MT1_XY_PENALTY;
        if (rd.tagCount < 2) {
            // tagCount is 0/1 here (2+ skips this block); 0 = LL gave no count.
            double p = SINGLE_TAG_PENALTY * (rd.tagCount == 1 ? 1.0 : 0.5);
            sXY *= p;
            sH *= p;
        }
        // Multi-tag MT2 with a narrow baseline triangulates poorly.
        if (rd.isMt2 && rd.tagCount >= 2 && rd.spanM > 0 && rd.spanM < SPAN_REF_M) {
            double k = 1.0 + 2.0 * (1.0 - rd.spanM / SPAN_REF_M);
            sXY *= k;
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
        double hTurnK = omegaRps / Math.max(HEADING_TURN_REF_RPS, 1e-6);
        sH *= 1.0 + hTurnK * hTurnK;
        double hSpeedK = speedIps / Math.max(HEADING_SPEED_REF_IPS, 1e-6);
        sH *= 1.0 + hSpeedK;
        // Innovation: smooth ramp, never a cliff.
        if (dxy > RESIDUAL_FULL_IN) {
            double t = Math.min((dxy - RESIDUAL_FULL_IN)
                    / Math.max(RESIDUAL_MAX_IN - RESIDUAL_FULL_IN, 1e-6), 1.0);
            double p = 1.0 + (RESIDUAL_MAX_PENALTY - 1.0) * t * t;
            sXY *= p;
            sH *= p;
        }

        // Limelight self-reported stddev: trusted 1:1 only when small.
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
        double rH = BASE_R_HEADING * Math.min(sH, MAX_TOTAL_SCALE);
        if (!rd.isMt2) rH = Math.max(rH, MT1_HEADING_R);
        // Stage 2 rescue inflates R for its residual pivot-arc error (1.0 when disabled).
        rXY = Math.min(rXY * compRScale, BASE_R_XY * MAX_TOTAL_SCALE);
        rH = Math.min(rH * compRScale, BASE_R_HEADING * MAX_TOTAL_SCALE);

        lastScale = Math.max(rXY / BASE_R_XY, rH / BASE_R_HEADING);
        lastRx = rXY;
        lastRh = rH;

        // Back-date to the exposure midpoint so the filter interpolates history.
        double latMs = Math.min(Math.max(rd.latencyMs, 0.0), MAX_LATENCY_MS);
        long stampNs = System.nanoTime() - (long) (latMs * 1.0e6);

        try {
            fusion.addMeasurement(measPose, stampNs, new Pose(rXY, rXY, rH));
            // Preserve the compensated marker when Stage 2 actually rescued.
            if (!"compensated".equals(lastStatus)) {
                lastStatus = rd.isMt2 ? "fused-mt2" : "fused-mt1";
            }
        } catch (Exception e) {
            lastStatus = "addMeasurement-exception";
        }
        report();
    }

    /** Panels telemetry for this loop (addData only — caller flushes once). */
    public void report() {
        RobotMain.DashTelemetry.addData("Fusion/status", lastStatus);
        RobotMain.DashTelemetry.addData("Fusion/active", isActive());
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
        RobotMain.DashTelemetry.addData("Fusion/scale", "%.2fx", lastScale);
        RobotMain.DashTelemetry.addData("Fusion/Rxy", "%.2f", lastRx);
        RobotMain.DashTelemetry.addData("Fusion/Rh", "%.4f", lastRh);
        if (Double.isFinite(lastResidualIn)) {
            RobotMain.DashTelemetry.addData("Fusion/residualIn", "%.2f", lastResidualIn);
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
}
