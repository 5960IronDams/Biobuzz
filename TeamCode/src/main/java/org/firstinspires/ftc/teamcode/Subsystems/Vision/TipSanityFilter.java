package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Sanity filter for vision poses off tipping BIOBUZZ hive cells.
 *
 * <p>The hive cells' 30-degree rest angle is stored in the {@code .fmap}, so a good
 * solve off an undisturbed cell reports flat (pitch ~ 0, roll ~ 0, Z ~ lens height).
 * Cells knocked off rest bend the solve (tilted, underground, or teleporting), so this
 * filter rejects solves outside the tilt, height, and speed gates below.
 */
@Configurable
public class TipSanityFilter {

    /** Reject solves when |botpose pitch| exceeds this (degrees). 0 disables. */
    public static double MAX_BOT_PITCH_DEG = 12.0;
    /** Reject solves when |botpose roll| exceeds this (degrees). 0 disables. */
    public static double MAX_BOT_ROLL_DEG = 12.0;
    /** True = reject solves outside [{@link #CAM_Z_MIN_M}, {@link #CAM_Z_MAX_M}]. */
    public static boolean CHECK_HEIGHT = true;
    /** Minimum valid botpose Z height (meters). */
    public static double CAM_Z_MIN_M = -0.15;
    /** Maximum valid botpose Z height (meters). */
    public static double CAM_Z_MAX_M = 0.70;
    /** Reject solves implying planar speed above this since the last accepted solve (in/s). 0 disables. */
    public static double MAX_SOLVE_SPEED_IPS = 200.0;
    /** Maximum time gap between accepted solves (seconds) before resetting speed-jump checks. */
    public static double MAX_TELEPORT_GAP_S = 1.5;

    /** Verdict for one reading. */
    public enum Verdict {
        /** Passed untouched — safe to fuse. */
        ACCEPT,
        /** Tilted beyond pitch/roll gates — tipped cell corrupted the solve. */
        REJECT_TILT,
        /** Camera height outside the physical band — underground/floating. */
        REJECT_HEIGHT,
        /** Teleported faster than the robot can move. */
        REJECT_TELEPORT
    }

    /** One filter decision (telemetry + tests). */
    public static final class Decision {
        public final Verdict verdict;
        /** Accepted (possibly compensated) pose, or the raw pose when rejected. */
        public final Pose pose;
        /** |pitch|, |roll| (deg) and Z (m) of the raw solve. */
        public final double pitchDeg, rollDeg, zMeters;
        /** Planar speed vs last accepted solve, in/s (NaN on first solve). */
        public final double speedIps;

        Decision(Verdict verdict, Pose pose, double pitchDeg, double rollDeg,
                 double zMeters, double speedIps) {
            this.verdict = verdict;
            this.pose = pose;
            this.pitchDeg = pitchDeg;
            this.rollDeg = rollDeg;
            this.zMeters = zMeters;
            this.speedIps = speedIps;
        }

        public boolean accepted() {
            return verdict == Verdict.ACCEPT;
        }
    }

    private Pose lastAcceptedXY = null;
    private long lastAcceptedNs = 0L;
    private Decision lastDecision = null;

    /** Reset filter history (call on OpMode start or pose relocalize). */
    public void reset() {
        lastAcceptedXY = null;
        lastAcceptedNs = 0L;
        lastDecision = null;
    }

    /**
     * Filter one reading using {@code rd.pedroPose} as the XY pose to check.
     */
    public Decision filter(Vision.Reading rd) {
        return filter(rd, rd != null ? rd.pedroPose : null);
    }

    /**
     * Filter one reading against an explicit {@code xyPose} (e.g. {@code rd.mt1Pedro}
     * when {@code VisionFusion.USE_MT1_XY} is enabled, so MT2 bimodal jumps
     * cannot trigger false {@link Verdict#REJECT_TELEPORT} drops on steady MT1 solves).
     */
    public Decision filter(Vision.Reading rd, @Nullable Pose xyPose) {
        if (rd == null || xyPose == null) {
            return decided(new Decision(Verdict.REJECT_TILT, null, 0, 0, 0, Double.NaN));
        }
        double pitch = Math.abs(rd.botPitchDeg);
        double roll = Math.abs(rd.botRollDeg);
        // GOOD ~= flat (map holds the 30deg): absolute tilt from 0.
        if ((MAX_BOT_PITCH_DEG > 0 && pitch > MAX_BOT_PITCH_DEG)
                || (MAX_BOT_ROLL_DEG > 0 && roll > MAX_BOT_ROLL_DEG)) {
            return decided(new Decision(Verdict.REJECT_TILT, xyPose,
                    pitch, roll, rd.botZMeters, Double.NaN));
        }
        // Check if robot/camera height makes sense (tipped solves project underground or skyward)
        if (CHECK_HEIGHT && Double.isFinite(rd.botZMeters)) {
            if (rd.botZMeters < CAM_Z_MIN_M || rd.botZMeters > CAM_Z_MAX_M) {
                return decided(new Decision(Verdict.REJECT_HEIGHT, xyPose,
                        pitch, roll, rd.botZMeters, Double.NaN));
            }
        }
        double speedIps = Double.NaN;
        long nowNs = System.nanoTime();
        if (lastAcceptedXY != null && MAX_SOLVE_SPEED_IPS > 0) {
            double dtS = (nowNs - lastAcceptedNs) / 1.0e9;
            if (dtS > 1e-3 && dtS <= MAX_TELEPORT_GAP_S) {
                double d = Math.hypot(xyPose.x() - lastAcceptedXY.x(),
                        xyPose.y() - lastAcceptedXY.y());
                speedIps = d / dtS;
                if (speedIps > MAX_SOLVE_SPEED_IPS) {
                    return decided(new Decision(Verdict.REJECT_TELEPORT, xyPose,
                            pitch, roll, rd.botZMeters, speedIps));
                }
            }
        }
        lastAcceptedXY = xyPose;
        lastAcceptedNs = nowNs;
        return decided(new Decision(Verdict.ACCEPT, xyPose,
                pitch, roll, rd.botZMeters, speedIps));
    }

    private Decision decided(Decision d) {
        // No telemetry here: VisionFusion.report() emits the snapshot once per
        // loop (correct() runs twice per TeleOp loop — see VisionFusion).
        lastDecision = d;
        return d;
    }

    /** Last decision (null before the first reading). */
    @Nullable
    public Decision lastDecision() {
        return lastDecision;
    }

}
