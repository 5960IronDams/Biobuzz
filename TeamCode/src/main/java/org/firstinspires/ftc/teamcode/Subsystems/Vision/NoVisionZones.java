package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * No-vision zones for the Biobuzz field (inches, Pedro field frame).
 *
 * <p>Reuses the geometric quad/box logic from {@link org.firstinspires.ftc.teamcode.WattageLib.Ballistics.NoFireZones}.
 * Defines areas of the field where AprilTag vision data is known to degenerate
 * (such as underneath the center hives where corner pipelines rapidly switch and
 * obstructed tag geometries cause solver corruption), as well as filtering out
 * erroneous solves produced when the robot is at the field perimeter looking across.
 *
 * <p>Defaults to the same big center square as the no-fire zone
 * (46.0, 48.0) to (95.0, 93.0) — covering the center area under the hives where
 * shooting is impossible and vision data degenerates.
 */
@Configurable
public final class NoVisionZones {

    private NoVisionZones() {}

    /** Master switch for no-vision zones (Panels-live). */
    public static boolean ENABLED = true;

    /**
     * Configured no-vision zones.
     * Default: the same big center square as the no-fire zone under the hives.
     */
    public static final List<NoVisionZone> NO_VISION_ZONES = new ArrayList<>(List.of(
            /* Center No-Vision Zone — identical to NoFireZones under-the-hive square */
            NoVisionZone.fromCorners(46.0, 48.0, 95.0, 93.0)
    ));

    /**
     * Checks if the given field-relative pose is inside any configured no-vision zone.
     *
     * @param pose Pose in Pedro frame (inches)
     * @return true if inside an active no-vision zone
     */
    public static boolean isInNoVisionZone(@Nullable Pose pose) {
        if (!ENABLED || pose == null) return false;
        return isInNoVisionZone(pose.x(), pose.y());
    }

    /**
     * Checks if the given field coordinates (inches, Pedro frame) fall inside any
     * configured no-vision zone.
     *
     * @param x Field X in inches
     * @param y Field Y in inches
     * @return true if inside an active no-vision zone
     */
    public static boolean isInNoVisionZone(double x, double y) {
        if (!ENABLED) return false;
        for (int i = 0; i < NO_VISION_ZONES.size(); i++) {
            NoVisionZone zone = NO_VISION_ZONES.get(i);
            if (zone != null && zone.contains(x, y)) {
                return true;
            }
        }
        return false;
    }

    /**
     * A 4-vertex convex polygon on the field where vision data is rejected.
     * Vertices must be specified in order (CW or CCW).
     */
    public static final class NoVisionZone {
        private final double x0, y0, x1, y1, x2, y2, x3, y3;

        public NoVisionZone(
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
         * Define an arbitrary 4-point polygon (CW or CCW).
         */
        public static NoVisionZone fromQuad(
                double x0, double y0,
                double x1, double y1,
                double x2, double y2,
                double x3, double y3) {
            return new NoVisionZone(x0, y0, x1, y1, x2, y2, x3, y3);
        }

        /**
         * Define an axis-aligned rectangle from two opposite corners.
         */
        public static NoVisionZone fromCorners(double xa, double ya, double xb, double yb) {
            double mnX = Math.min(xa, xb), mnY = Math.min(ya, yb);
            double mxX = Math.max(xa, xb), mxY = Math.max(ya, yb);
            return new NoVisionZone(
                    mnX, mnY,   // bottom-left
                    mxX, mnY,   // bottom-right
                    mxX, mxY,   // top-right
                    mnX, mxY);  // top-left
        }

        /**
         * Define a square zone from center point and side length.
         */
        public static NoVisionZone fromCenter(double cx, double cy, double sizeIn) {
            double half = sizeIn / 2.0;
            return fromCorners(cx - half, cy - half, cx + half, cy + half);
        }

        /**
         * Returns true if the given XY point is inside this convex polygon.
         */
        public boolean contains(double px, double py) {
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
                if (positive && negative) return false;
            }
            return true;
        }

        public boolean contains(@Nullable Pose pose) {
            return pose != null && contains(pose.x(), pose.y());
        }
    }
}
