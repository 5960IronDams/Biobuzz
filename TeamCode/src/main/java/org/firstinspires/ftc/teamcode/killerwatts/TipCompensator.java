package org.firstinspires.ftc.teamcode.killerwatts;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Stage 2 (EXPERIMENTAL, DISABLED): rest-signature projection of off-rest solves.
 *
 * <p>DO NOT ENABLE YET — Stage 1 ({@link TipSanityFilter}, wired in and ON) drops
 * off-rest solves; this class tries to RESCUE them instead. It is written, builds,
 * and has a disabled hook in {@code VisionFusion}, but its sign conventions and
 * lever-arm model are unverified against real field data.
 *
 * <p>Model (first-order, documented so it can be checked on-field):
 * the .fmap maps every tag at its 30deg REST pose, so a solve off a REST cell
 * reports flat (pitch ~= 0, roll ~= 0) with correct XY. A cell knocked off rest
 * by (dp, dr) degrees mismatches the map; the solver thinks the camera tilted
 * by (dp, dr), displacing it ~h·tan(deviation) on the lens-height lever arm.
 * The compensator inverts that deviation:
 * <pre>
 *   dp = pitch − 0, dr = roll − 0  (deviation from flat, degrees, field frame)
 *   ex = CAM_HEIGHT_M · tan(dp) · PITCH_SIGN      (meters, field X)
 *   ey = CAM_HEIGHT_M · tan(dr) · ROLL_SIGN       (meters, field Y)
 *   compensated = (x − ex, y − ey, yaw unchanged, in inches)
 *   Z prescribed = CAM_HEIGHT_M (not estimated)
 * </pre>
 * At a REST cell dp = dr = 0: no shift, rScale = 1, solve fuses at full weight.
 * Yaw is passed through: tip about a horizontal axis couples only second-order
 * into yaw. Whatever deviation remains unmodeled (the tag face also physically
 * swings on an arc about the cell pivot, inches of real displacement) is covered
 * by inflating R: {@code rScale = 1 + TILT_R_PER_DEG · devDeg}.
 *
 * <p>Validation plan (FusionTune, before enabling):
 * <ol>
 *   <li>Park level in front of a REST cell: confirm Tip/pitchDeg + Tip/rollDeg
 *       read ~= 0 (flat). If they sit at a constant offset, the map's rest
 *       direction is mirrored — fix the .fmap block, not the signs here.</li>
 *   <li>Knock the cell off rest (hold it tipped), compare raw vs compensated
 *       XY against Pinpoint truth.</li>
 *   <li>If compensated pushes AWAY from truth, flip PITCH_SIGN / ROLL_SIGN
 *       (Panels-live) — the magnitudes are geometry, the signs are convention.</li>
 *   <li>If residual error still exceeds ~4in at 20° deviation, the pivot-arc
 *       term dominates: leave this DISABLED and let Stage 1 keep dropping.</li>
 * </ol>
 */
@Configurable
public class TipCompensator {

    /** Master switch. False = hook in VisionFusion is a no-op. Flip only after the plan above. */
    public static boolean ENABLED = false;
    /**
     * Lens height above the tiles, meters. MEASURE YOUR ROBOT (tape from tile
     * to lens center). Used as the lever arm AND the prescribed Z.
     */
    public static double CAM_HEIGHT_M = 0.25;
    /** Refuse to rescue beyond this deviation from rest (deg) — return null (drop). */
    public static double MAX_COMP_TILT_DEG = 25.0;
    /** Refuse when |reported Z − CAM_HEIGHT_M| exceeds this (m) — not our error model. */
    public static double CAM_Z_TOL_M = 0.15;
    /** Extra R scale per degree of deviation from rest (residual pivot-arc error). */
    public static double TILT_R_PER_DEG = 0.15;
    /** Sign of the pitch correction. VERIFY (see class javadoc step 3). */
    public static double PITCH_SIGN = 1.0;
    /** Sign of the roll correction. VERIFY (see class javadoc step 3). */
    public static double ROLL_SIGN = 1.0;

    /** Compensated measurement + how much extra R it needs. */
    public static final class Result {
        /** Rest-projected pose (inches, Pedro frame — same frame as input). */
        public final Pose pose;
        /** Multiply the fusion R scales by this (>= 1). */
        public final double rScale;
        /** Deviation from flat that was removed (deg). */
        public final double tiltDeg;
        /** Planar shift applied (in). */
        public final double shiftIn;

        Result(Pose pose, double rScale, double tiltDeg, double shiftIn) {
            this.pose = pose;
            this.rScale = rScale;
            this.tiltDeg = tiltDeg;
            this.shiftIn = shiftIn;
        }
    }

    /**
     * Project one off-rest solve back toward flat. Pure function of its
     * arguments — unit-testable without hardware. Returns null when the solve
     * is outside this model (too far off flat / Z inconsistent) — caller must
     * DROP, not fuse.
     *
     * @param rawPedro raw vision pose (inches, Pedro frame)
     * @param pitchDeg botpose pitch, degrees (field frame)
     * @param rollDeg  botpose roll, degrees (field frame)
     * @param zMeters  botpose camera height, meters
     */
    @Nullable
    public static Result compensate(Pose rawPedro, double pitchDeg, double rollDeg, double zMeters) {
        if (rawPedro == null) return null;
        if (!Double.isFinite(pitchDeg) || !Double.isFinite(rollDeg) || !Double.isFinite(zMeters)) return null;
        // Deviation from flat: the map already holds the 30deg rest.
        double dp = pitchDeg;
        double dr = rollDeg;
        double tiltDeg = Math.hypot(dp, dr);
        if (!Double.isFinite(tiltDeg) || tiltDeg > MAX_COMP_TILT_DEG) return null;
        if (Math.abs(zMeters - CAM_HEIGHT_M) > CAM_Z_TOL_M) return null;
        if (!(CAM_HEIGHT_M > 0)) return null;

        double exM = CAM_HEIGHT_M * Math.tan(Math.toRadians(dp)) * PITCH_SIGN;
        double eyM = CAM_HEIGHT_M * Math.tan(Math.toRadians(dr)) * ROLL_SIGN;
        if (!Double.isFinite(exM) || !Double.isFinite(eyM)) return null;
        double exIn = exM * 39.3700787;
        double eyIn = eyM * 39.3700787;
        double shiftIn = Math.hypot(exIn, eyIn);

        Pose out = new Pose(rawPedro.x() - exIn, rawPedro.y() - eyIn, rawPedro.heading());
        double rScale = 1.0 + TILT_R_PER_DEG * tiltDeg;
        return new Result(out, rScale, tiltDeg, shiftIn);
    }
}
