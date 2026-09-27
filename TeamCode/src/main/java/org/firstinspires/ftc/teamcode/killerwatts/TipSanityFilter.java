package org.firstinspires.ftc.teamcode.killerwatts;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Stage 1 (WIRED IN, ON by default): rest-pose sanity filter for vision poses.
 *
 * <p>The hive cells REST at 30deg tipped, and that 30deg lives IN THE MAP (each
 * tag entry is rotated 30deg off straight-down). So a GOOD solve — cell at rest,
 * map matching reality — reports a FLAT botpose: pitch ~= 0, roll ~= 0. A cell
 * knocked off rest mismatches the map and the solver bends the whole botpose to
 * compensate: tilted, underground, or teleported. Those get dropped before the
 * Kalman corrector. This filter sits between {@link Vision#poll} and the
 * corrector and:
 * <ol>
 *   <li>REJECTS impossible solves outright (tilt / height / speed gates below).</li>
 *   <li>PASSES sane solves through untouched (same object, zero math).</li>
 *   <li>Stashes every decision in {@link #lastDecision()} — {@code VisionFusion.report()}
 *       mirrors it to Panels ({@code Tip/*}) once per loop for tuning.</li>
 * </ol>
 *
 * <p>Stage 2 (the compensator) lives in {@link TipCompensator} and is DISABLED
 * until tested — it attempts to rescue tilted solves by projecting them back
 * to the floor instead of dropping them. This class never rescues; it only
 * passes or drops, so its failure mode is "fuse nothing this loop", which the
 * Kalman predictor (Pinpoint) rides through harmlessly.
 */
@Configurable
public class TipSanityFilter {

    // ================= gates (Panels -> TipSanityFilter) =================
    // Reference frame, read carefully: the 30deg REST tilt is encoded IN THE MAP
    // (each tag entry is rotated 30deg off straight-down). When the observed cell
    // is at rest, map matches reality, so a GOOD solve reports a FLAT botpose:
    // pitch ~= 0, roll ~= 0, Z ~= lens height. A cell knocked OFF rest mismatches
    // the map, and the solver bends the whole botpose to compensate — THAT is the
    // tilted/underground solve we drop. So these gates are ABSOLUTE tilt from flat,
    // not deviation from 30. If good solves ever show a constant non-zero pitch,
    // the map's rest direction is mirrored — fix the .fmap block rotation, not this.
    /**
     * Reject when |botpose pitch| exceeds this (deg). GOOD ~= 0 (map holds the
     * 30deg). Knocked cell -> whole solve tilts. 0 disables.
     */
    public static double MAX_BOT_PITCH_DEG = 12.0;
    /** Same for |botpose roll|, degrees. 0 disables. */
    public static double MAX_BOT_ROLL_DEG = 12.0;
    /**
     * Reject when reported camera height (botpose Z, meters) is outside this
     * band. Set to YOUR Limelight lens height +/- slack: a tipped solve shoves
     * Z toward the floor (underground = the "robot inside the ground" bug) or
     * skyward. Measure lens height once, allow +/- 4in (0.10m) for map/floor slop.
     */
    public static double CAM_Z_MIN_M = 0.10;
    public static double CAM_Z_MAX_M = 0.45;
    /**
     * Reject when the solve moved faster than this since the last ACCEPTED
     * solve (in/s). Tipped-cell solves teleport as cells rock; the real robot
     * cannot teleport. 0 disables.
     */
    public static double MAX_SOLVE_SPEED_IPS = 120.0;

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

    /**
     * Filter one reading. Returns the decision; when accepted, fuse
     * {@code decision.pose} (== the input pose, untouched). Pure function of
     * the reading + last accepted solve — safe to unit test without hardware.
     */
    public Decision filter(Vision.Reading rd) {
        if (rd == null || rd.pedroPose == null) {
            return decided(new Decision(Verdict.REJECT_TILT, null, 0, 0, 0, Double.NaN));
        }
        double pitch = Math.abs(rd.botPitchDeg);
        double roll = Math.abs(rd.botRollDeg);
        // GOOD ~= flat (map holds the 30deg): absolute tilt from 0.
        if ((MAX_BOT_PITCH_DEG > 0 && pitch > MAX_BOT_PITCH_DEG)
                || (MAX_BOT_ROLL_DEG > 0 && roll > MAX_BOT_ROLL_DEG)) {
            return decided(new Decision(Verdict.REJECT_TILT, rd.pedroPose,
                    pitch, roll, rd.botZMeters, Double.NaN));
        }
        //check if tag height makes sense;
//        if (!(rd.botZMeters >= CAM_Z_MIN_M && rd.botZMeters <= CAM_Z_MAX_M)) {
//            return decided(new Decision(Verdict.REJECT_HEIGHT, rd.pedroPose,
//                    pitch, roll, rd.botZMeters, Double.NaN));
//        }
        double speedIps = Double.NaN;
        long nowNs = System.nanoTime();
        if (lastAcceptedXY != null && MAX_SOLVE_SPEED_IPS > 0) {
            double dtS = (nowNs - lastAcceptedNs) / 1.0e9;
            if (dtS > 1e-3) {
                double d = Math.hypot(rd.pedroPose.x() - lastAcceptedXY.x(),
                        rd.pedroPose.y() - lastAcceptedXY.y());
                speedIps = d / dtS;
                if (speedIps > MAX_SOLVE_SPEED_IPS) {
                    return decided(new Decision(Verdict.REJECT_TELEPORT, rd.pedroPose,
                            pitch, roll, rd.botZMeters, speedIps));
                }
            }
        }
        lastAcceptedXY = rd.pedroPose;
        lastAcceptedNs = nowNs;
        return decided(new Decision(Verdict.ACCEPT, rd.pedroPose,
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
