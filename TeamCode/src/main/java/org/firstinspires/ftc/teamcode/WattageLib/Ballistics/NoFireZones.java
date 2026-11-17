package org.firstinspires.ftc.teamcode.WattageLib.Ballistics;

import org.firstinspires.ftc.teamcode.Commands.AimAtGoal;
import org.firstinspires.ftc.teamcode.RobotMain;

import com.pedropathing.math.Pose;

/**
 * Defines field areas (in inches, Pedro frame) where the shooter must not fire.
 * Checked by shooting commands to keep the gate closed whenever the shooter pose
 * is inside any configured zone.
 */
public final class NoFireZones {

    private NoFireZones() {}

    /**
     * Field regions where shooting is blocked (inches, Pedro frame).
     * Ways to define a zone:
     * - {@code NoFireZone.fromCorners(x1, y1, x2, y2)} for an axis-aligned rectangle
     * - {@code NoFireZone.fromCenter(cx, cy, sizeIn)} for a square
     * - {@code NoFireZone.fromQuad(x0, y0, x1, y1, x2, y2, x3, y3)} for a 4-point convex polygon
     */
    public static final java.util.List<NoFireZone> NO_FIRE_ZONES = java.util.List.of(
            NoFireZone.fromCorners(0.7, 44.0, 141.0, 97.0), // Center band between flowers/hives
            NoFireZone.fromCorners(46.0, 48.0, 95.0, 93.0)  // Under the hive area
    );

    /**
     * Robot-relative shooter offset (inches) from the pose Pedro tracks.
     * Tuned in {@link AimAtGoal#SHOOTER_OFFSET_X_IN} / {@link AimAtGoal#SHOOTER_OFFSET_Y_IN}.
     * Defaults match AimAtGoal (5.5 in towards the rear: X = -5.5, Y = 0.0).
     */
    public static double SHOOTER_OFFSET_X_IN = -5.5;
    public static double SHOOTER_OFFSET_Y_IN = 0.0;

    // -----------------------------------------------------------------------
    //  No-Fire Zone check
    // -----------------------------------------------------------------------

    /**
     * Returns {@code true} when the <b>shooter</b> (not the robot center) is
     * inside any of the configured {@link #NO_FIRE_ZONES}.
     *
     * <p>Call this with the current robot pose; the shooter offset is applied
     * internally using {@link AimAtGoal#getShooterPose(Pose)}.
     *
     * @param robotPose Current field-relative robot pose (Pedro, inches)
     * @return true if the shooter is inside a no-fire zone
     */
    public static boolean isInNoFireZone(Pose robotPose) {
        if (robotPose == null) return false;

        Pose shooter = AimAtGoal.getShooterPose(robotPose);
        if (shooter == null) return false;
        double sx = shooter.x();
        double sy = shooter.y();

        for (NoFireZone zone : NO_FIRE_ZONES) {
            if (zone.contains(sx, sy)) {
                if (RobotMain.DASHBOARD) {
                    RobotMain.DashTelemetry.addData("Ballistics/InNoFireZone", true);
                }
                return true;
            }
        }
        if (RobotMain.DASHBOARD) {
            RobotMain.DashTelemetry.addData("Ballistics/InNoFireZone", false);
        }
        return false;
    }

    /**
     * A 4-vertex convex polygon on the field where shooting is prohibited.
     * Vertices must be specified in order (CW or CCW).
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
