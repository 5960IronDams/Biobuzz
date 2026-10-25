package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotMain;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Relative hive-cell state from cluster tags. AIMING ONLY (shares the Limelight
 * with the global pose — see {@link VisionFusion}).
 *
 * <p>Why a separate class: the same cluster tags (IDs 30-45) feed the global
 * botpose in the .fmap, but their goal-point poses are ALSO the best drive-to-cell
 * signal. This monitor reads the per-tag detections and answers:
 * <ol>
 *   <li>Which cells can I see right now, and how tipped is each?</li>
 *   <li>Is a cell scorable for my alliance (right-side-up + my color)?</li>
 * </ol>
 *
 * <p>Per the FTC tech tip on clusters: a detection carries the cluster
 * <i>goal-point</i> pose ({@code targetPoseRobotSpace}), not the printed-tag
 * pose. Use that for alignment (drive-to-cell), and the per-face normal math
 * below for tip angle. Roll convention: |roll| &lt; 90deg = right-side-up =
 * scorable; outside = upside-down / pointed away = not scorable.
 *
 * <p>Tip math ({@link CellSight#tipAngleDeg}) is DEVIATION FROM 30° REST, not from
 * level: the 4 member tags are printed coplanar with known in-plane offsets
 * (SDK 12.0: x = −6.5/−2.75/+2.75/+6.5 in) on a face that rests 30° tipped. The
 * .fmap entries encode that rest tilt, so the range gradient across the face's
 * x extent is whatever rest produces — the reported tip is the gradient-implied
 * angle MINUS the rest-implied gradient ({@code asin(g) − asin(gRest)}): ~0 at
 * rest, signed deviation when knocked off. Computed from per-tag
 * {@code targetPoseRobotSpace} ranges — no extra solve, no matrix code.
 * ({@code gRest = sin(30°) ≈ 0.5} by default; measured once on-field via
 * {@code REST_GRADIENT} if the rest face isn't exactly 30° to the camera.)
 */
@Configurable
public class HiveCellMonitor {

    // ---- validity gates (Panels -> HiveCellMonitor) ----
    /** Minimum tag area (% of image) to trust a face for tip math. */
    public static double MIN_AREA_PCT = 0.05;
    /** |roll| above this (deg) = cell upside-down / not scorable. */
    public static double ROLL_SCORABLE_DEG = 90.0;
    /** Minimum members seen (of 4) before tip angle is reported. */
    public static int MIN_MEMBERS_FOR_TIP = 2;
    /** In-plane x extent of the 4-tag face, inches (6.5 - -6.5). */
    public static double FACE_X_EXTENT_IN = 13.0;
    /** Rest tilt of the cell face off straight-down, degrees (Fig 9-10). */
    public static double REST_TIP_DEG = 30.0;
    /**
     * Range gradient (dRange/dxExtent) measured at a REST cell. Default sin(30°).
     * Re-measure on-field: park square to a rest cell, read Cell/tipRaw, set this
     * so Cell/tip reads ~0. Absorbs camera height + face geometry in one number.
     */
    public static double REST_GRADIENT = 0.5;

    /** One visible cluster face this loop. */
    public static final class CellSight {
        /** Cluster base ID: 30, 34, 38 or 42. */
        public final int baseId;
        /** SDK cluster name ("RED SCORING", ...). */
        public final String name;
        /** Member IDs actually seen. */
        public final int[] memberIds;
        /** Range to the cluster goal point, inches (for drive-to-cell). */
        public final double rangeIn;
        /** Bearing to the goal point, degrees (+ = target right of center). */
        public final double bearingDeg;
        /** Face roll, degrees. |roll| < 90 = scorable orientation. */
        public final double rollDeg;
        /**
         * Estimated tip DEVIATION FROM 30° REST, degrees. ~0 = resting (good);
         * sign = knocked direction. NaN when fewer than {@link #MIN_MEMBERS_FOR_TIP}
         * members visible (no baseline for the gradient).
         */
        public final double tipAngleDeg;
        /** Raw gradient-implied face angle, degrees (for REST_GRADIENT setup). */
        public final double tipRawDeg;
        /** Mean tag area (% image) across seen members. */
        public final double meanAreaPct;

        CellSight(int baseId, String name, int[] memberIds, double rangeIn,
                  double bearingDeg, double rollDeg, double tipAngleDeg,
                  double tipRawDeg, double meanAreaPct) {
            this.baseId = baseId;
            this.name = name;
            this.memberIds = memberIds;
            this.rangeIn = rangeIn;
            this.bearingDeg = bearingDeg;
            this.rollDeg = rollDeg;
            this.tipAngleDeg = tipAngleDeg;
            this.tipRawDeg = tipRawDeg;
            this.meanAreaPct = meanAreaPct;
        }

        /** True when the face is right-side-up (cell level-ish, scorable side). */
        public boolean isUpright() {
            return Math.abs(rollDeg) < ROLL_SCORABLE_DEG;
        }

        /**
         * True when this cluster belongs to the given alliance letter
         * ('R'/'B') AND is upright. Name check first (cheap), roll second.
         */
        public boolean isScorableFor(char allianceLetter) {
            if (name == null || name.isEmpty()) return false;
            if (Character.toUpperCase(name.charAt(0)) != Character.toUpperCase(allianceLetter)) return false;
            return isUpright();
        }
    }

    @Nullable
    private final Vision vision;
    private final List<CellSight> lastSights = new ArrayList<>();
    private char lastAlliance = 'R';

    public HiveCellMonitor(@Nullable Vision vision) {
        this.vision = vision;
    }

    public HiveCellMonitor(HardwareMap hw) {
        this(Vision.tryCreate(hw));
    }

    /** Re-poll the Limelight. Call once per loop (cheap: parses cached result). */
    public void update() {
        update('R');
    }

    /**
     * Re-poll, remembering the alliance letter for telemetry + closestScorable().
     * Pass 'R' or 'B' from {@code RobotMain.CurrentAlliance}.
     */
    public void update(char allianceLetter) {
        lastAlliance = Character.toUpperCase(allianceLetter) == 'B' ? 'B' : 'R';
        lastSights.clear();
        if (vision == null) return;
        List<LLResultTypes.FiducialResult> frs;
        try {
            frs = vision.getLatestFiducials();
        } catch (Exception e) {
            return;
        }
        if (frs == null || frs.isEmpty()) return;

        // Group member detections by cluster base.
        // Order: base 30, 34, 38, 42.
        for (int base = 30; base <= 42; base += 4) {
            ArrayList<LLResultTypes.FiducialResult> members = new ArrayList<>();
            for (LLResultTypes.FiducialResult fr : frs) {
                if (fr == null) continue;
                int id;
                try {
                    id = fr.getFiducialId();
                } catch (Exception e) {
                    continue;
                }
                if (id >= base && id < base + 4) members.add(fr);
            }
            if (members.isEmpty()) continue;
            CellSight sight = buildSight(base, members);
            if (sight != null) lastSights.add(sight);
        }
        report();
    }

    /** All cluster faces seen on the last update(). */
    public List<CellSight> sights() {
        return new ArrayList<>(lastSights);
    }

    /**
     * Closest scorable face for the last update()'s alliance, by goal-point range.
     * Null when none visible/scorable. This is the auto's "which cell do I
     * drive to" answer.
     */
    @Nullable
    public CellSight closestScorable() {
        return closestScorable(lastAlliance);
    }

    /** Same, with an explicit alliance letter ('R' or 'B'). */
    @Nullable
    public CellSight closestScorable(char allianceLetter) {
        CellSight best = null;
        for (CellSight s : lastSights) {
            if (!s.isScorableFor(allianceLetter)) continue;
            if (best == null || s.rangeIn < best.rangeIn) best = s;
        }
        return best;
    }

    @Nullable
    private static CellSight buildSight(int base, List<LLResultTypes.FiducialResult> members) {
        String name = Vision.clusterNameFor(base);
        int n = members.size();
        int[] ids = new int[n];
        double rangeSum = 0;
        double areaSum = 0;
        double bx = 0, by = 0;
        double rollDeg = 0;
        boolean havePose = false;
        // For tip gradient: range at min-x member vs max-x member. Member
        // x-order in the cluster plane is ID order (SDK: -6.5..+6.5 by ID).
        double rangeAtMinX = Double.NaN, rangeAtMaxX = Double.NaN;
        int minId = Integer.MAX_VALUE, maxId = Integer.MIN_VALUE;
        for (int i = 0; i < n; i++) {
            LLResultTypes.FiducialResult fr = members.get(i);
            int id;
            try {
                id = fr.getFiducialId();
            } catch (Exception e) {
                continue;
            }
            ids[i] = id;
            double area = 0;
            try {
                area = fr.getTargetArea();
            } catch (Exception ignored) {
            }
            areaSum += area;
            // Per-tag robot-space pose -> range + bearing + roll.
            try {
                Pose3D rp =
                        fr.getTargetPoseRobotSpace();
                if (rp != null) {
                    double rx = rp.getPosition().toUnit(DistanceUnit.INCH).x;
                    double ry = rp.getPosition().toUnit(DistanceUnit.INCH).y;
                    double rz = rp.getPosition().toUnit(DistanceUnit.INCH).z;
                    double range = Math.sqrt(rx * rx + ry * ry + rz * rz);
                    rangeSum += range;
                    // Bearing from forward axis: atan2(right, forward). FTC
                    // robot space: x right, y forward (matches ftcPose key).
                    bx += Math.toDegrees(Math.atan2(rx, ry));
                    by += 1; // count of valid poses
                    try {
                        rollDeg = rp.getOrientation().getRoll(
                                AngleUnit.DEGREES);
                    } catch (Exception ignored) {
                    }
                    havePose = true;
                    if (id < minId) {
                        minId = id;
                        rangeAtMinX = range;
                    }
                    if (id > maxId) {
                        maxId = id;
                        rangeAtMaxX = range;
                    }
                }
            } catch (Exception ignored) {
            }
        }
        if (!havePose || by < 1) return null;
        double meanRange = rangeSum / by;
        double meanBearing = bx / by;
        double meanArea = areaSum / n;
        double tip = Double.NaN;
        double tipRawDeg = Double.NaN;
        if (n >= MIN_MEMBERS_FOR_TIP && !Double.isNaN(rangeAtMinX) && !Double.isNaN(rangeAtMaxX)
                && maxId > minId && FACE_X_EXTENT_IN > 0) {
            // Gradient-implied face angle minus rest gradient: ~0 at rest.
            double g = (rangeAtMaxX - rangeAtMinX) / FACE_X_EXTENT_IN;
            g = Math.max(-1.0, Math.min(1.0, g));
            tipRawDeg = Math.toDegrees(Math.asin(g));
            double gR = Math.max(-1.0, Math.min(1.0, REST_GRADIENT));
            tip = tipRawDeg - Math.toDegrees(Math.asin(gR));
        }
        return new CellSight(base, name, ids, meanRange, meanBearing, rollDeg, tip, tipRawDeg, meanArea);
    }

    private void report() {
        RobotMain.DashTelemetry.addData("Cell/faces", lastSights.size());
        CellSight best = closestScorable();
        RobotMain.DashTelemetry.addData("Cell/best", best == null
                ? "none-" + lastAlliance
                : String.format(Locale.US, "%s %.1fin b=%.1f tip=%s",
                        best.name, best.rangeIn, best.bearingDeg,
                        Double.isNaN(best.tipAngleDeg) ? "n/a"
                                : String.format(Locale.US, "%.1f", best.tipAngleDeg)));
        for (CellSight s : lastSights) {
            String key = "Cell/" + s.baseId;
            RobotMain.DashTelemetry.addData(key,
                    "%s ids=%s r=%.1fin b=%.1f roll=%.1f tip=%s raw=%s",
                    s.name, idsToString(s.memberIds), s.rangeIn, s.bearingDeg, s.rollDeg,
                    Double.isNaN(s.tipAngleDeg) ? "n/a"
                            : String.format(Locale.US, "%.1f", s.tipAngleDeg),
                    Double.isNaN(s.tipRawDeg) ? "n/a"
                            : String.format(Locale.US, "%.1f", s.tipRawDeg));
        }
    }

    private static String idsToString(int[] ids) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < ids.length; i++) {
            if (i > 0) sb.append(',');
            sb.append(ids[i]);
        }
        return sb.append(']').toString();
    }
}
