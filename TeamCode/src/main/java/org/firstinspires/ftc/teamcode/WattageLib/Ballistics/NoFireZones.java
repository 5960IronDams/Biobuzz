package org.firstinspires.ftc.teamcode.WattageLib.Ballistics;

import org.firstinspires.ftc.teamcode.RobotMain;

import com.pedropathing.math.Pose;

/**
 * Direct port of AlphaBots 2026-IndianaJoe {@code ballistics.NoFireZones}
 * (FRC/WPILib, meters) to FTC/Pedro (inches, {@link Pose}).
 *
 * <p>This class is for defining areas of the field that we do not want to
 * shoot into, such as the driver station. It is used by the shooter (the
 * ShootAtGoal compound in {@code CommandFactory}) to keep the shoot gate
 * CLOSED while the shooter is inside one of these zones, and to open it once
 * the shooter is clear again.
 *
 * <p>Axis-aligned / arbitrary regions on the field (in INCHES, Pedro field
 * frame — blue-alliance origin, matching {@code FieldXYMaxPoint} in
 * {@code IronConstants}) where the <b>shooter</b> (not the robot center) must
 * NOT fire. The readiness check {@link #isInNoFireZone(Pose)} goes true
 * whenever the shooter position falls inside any of these zones.
 *
 * <p>Three ways to define a zone:
 * <ul>
 *   <li>{@code NoFireZone.fromCorners(x1, y1, x2, y2)} — opposite corners of an axis-aligned rectangle</li>
 *   <li>{@code NoFireZone.fromCenter(cx, cy, sizeIn)} — square from center + side length</li>
 *   <li>{@code NoFireZone.fromQuad(x0,y0, x1,y1, x2,y2, x3,y3)} — arbitrary 4-point convex polygon (CW or CCW)</li>
 * </ul>
 */
public final class NoFireZones {

    private NoFireZones() {}

    // ====================================================================
    //  No-Fire Zones
    // ====================================================================
    /**
     * List of no-fire zones. Add/remove entries here.
     *
     * <p>TODO: edit these coordinates to the Biobuzz field (inches, Pedro
     * frame). The two hive squares below are PLACEHOLDERS centered on the
     * field so the wiring can be tested; replace the centers with the real
     * hive/goal positions from AimPoints.pp, and add any trapezoid / trench
     * zones the same way the IndianaJoe version did.
     */
    public static double HiveNofireZoneSizeIn = 30.0; // 15" in each direction from center
    public static final java.util.List<NoFireZone> NO_FIRE_ZONES = java.util.List.of(
        /*Center No Fire Zone - Blocks out firing anywhere underneath the hives.  */
            NoFireZone.fromCorners(46.0,48.0,95.0,93.0)//should encompass the entire middle no firezone corner to corner.
    );

    /**
     * Robot-relative shooter offset (inches) from the pose Pedro tracks, applied
     * (rotated by heading) before the zone test — the IndianaJoe version used
     * {@code constants.Turret.ShooterPivotRobotRelative2d} for this. Defaults to
     * 0,0 (test the robot center); set to the shoot-gate/shooter position when
     * the mechanism geometry is known.
     */
    public static double SHOOTER_OFFSET_X_IN = 0.0;
    public static double SHOOTER_OFFSET_Y_IN = 0.0;

    // -----------------------------------------------------------------------
    //  No-Fire Zone check
    // -----------------------------------------------------------------------

    /**
     * Returns {@code true} when the <b>shooter</b> (not the robot center) is
     * inside any of the configured {@link #NO_FIRE_ZONES}.
     *
     * <p>Call this with the current robot pose; the shooter offset is applied
     * internally using {@link #SHOOTER_OFFSET_X_IN}/{@link #SHOOTER_OFFSET_Y_IN}.
     *
     * @param robotPose Current field-relative robot pose (Pedro, inches)
     * @return true if the shooter is inside a no-fire zone
     */
    public static boolean isInNoFireZone(Pose robotPose) {
        if (robotPose == null) return false;

        // Rotate the robot-relative offset into the field frame ( Pedro
        // heading is radians), then test the shooter point against all zones.
        double cos = Math.cos(robotPose.heading());
        double sin = Math.sin(robotPose.heading());
        double sx = robotPose.x() + SHOOTER_OFFSET_X_IN * cos - SHOOTER_OFFSET_Y_IN * sin;
        double sy = robotPose.y() + SHOOTER_OFFSET_X_IN * sin + SHOOTER_OFFSET_Y_IN * cos;

        for (NoFireZone zone : NO_FIRE_ZONES) {
            if (zone.contains(sx, sy)) {
                // addData only — RobotMain.flushTelemetry() does the single flush.
                RobotMain.DashTelemetry.addData("Ballistics/InNoFireZone", true);
                return true;
            }
        }
        RobotMain.DashTelemetry.addData("Ballistics/InNoFireZone", false);
        return false;
    }

    /**
     * A 4-vertex convex polygon on the field where shooting is prohibited.
     * Vertices must be specified in order (CW or CCW).
     * The shooter pose (not robot center) is tested.
     *
     * <p>(The IndianaJoe original was a Java 16 {@code record}; FTC builds at
     * source level 8, so this is a plain final class with the same shape.)
     */
    public static final class NoFireZone {
        private final double x0, y0, x1, y1, x2, y2, x3, y3;

        /** Build directly from 4 vertices in order (CW or CCW). */
        public NoFireZone(
                double x0, double y0,
                double x1, double y1,
                double x2, double y2,
                double x3, double y3) {
            this.x0 = x0; this.y0 = y0;
            this.x1 = x1; this.y1 = y1;
            this.x2 = x2; this.y2 = y2;
            this.x3 = x3; this.y3 = y3;
        }

        /**
         * Define an arbitrary 4-point polygon (trapezoid, parallelogram, etc.).
         * Vertices must be in order (clockwise or counter-clockwise).
         */
        public static NoFireZone fromQuad(
                double x0, double y0,
                double x1, double y1,
                double x2, double y2,
                double x3, double y3) {
            return new NoFireZone(x0, y0, x1, y1, x2, y2, x3, y3);
        }

        /**
         * Define an axis-aligned rectangle from two opposite corners
         * (order doesn't matter). Corners are sorted internally.
         */
        public static NoFireZone fromCorners(double xa, double ya, double xb, double yb) {
            double mnX = Math.min(xa, xb), mnY = Math.min(ya, yb);
            double mxX = Math.max(xa, xb), mxY = Math.max(ya, yb);
            return new NoFireZone(
                    mnX, mnY,   // bottom-left
                    mxX, mnY,   // bottom-right
                    mxX, mxY,   // top-right
                    mnX, mxY);  // top-left
        }

        /**
         * Define a square zone from its center point and side length.
         */
        public static NoFireZone fromCenter(double cx, double cy, double sizeIn) {
            double half = sizeIn / 2.0;
            return fromCorners(cx - half, cy - half, cx + half, cy + half);
        }

        /**
         * Returns true if the given field-frame XY point is inside this
         * 4-vertex polygon. Uses the winding-number / cross-product
         * method which works for any convex quad.
         *
         * <p>Performance: ~40 floating-point operations per call — negligible
         * at once per robot loop.
         */
        public boolean contains(double px, double py) {
            // For a convex polygon the point is inside iff the cross
            // products of (edge x point-to-vertex) all share the same
            // sign. We store vertices in an array for easy looping.
            double[] xs = {x0, x1, x2, x3};
            double[] ys = {y0, y1, y2, y3};
            boolean positive = false;
            boolean negative = false;
            for (int i = 0; i < 4; i++) {
                int j = (i + 1) % 4;
                double cross = (xs[j] - xs[i]) * (py - ys[i])
                             - (ys[j] - ys[i]) * (px - xs[i]);
                if (cross > 0) positive = true;
                if (cross < 0) negative = true;
                if (positive && negative) return false; // outside
            }
            return true; // all same sign -> inside (or on edge)
        }
    }
}
