package org.firstinspires.ftc.teamcode.killerwatts.lib;

/**
 * Single absolute field measurement from a vision source (e.g. Limelight 3A).
 *
 * <p>Units: x/y in inches in the field frame, heading in radians wrapped to
 * [-pi, pi]. Std devs express 1-sigma confidence; larger = trusted less by
 * the Kalman filter. This mirrors how Limelight's MegaTag reports pose +
 * uncertainty, so the future LL3A class can construct this directly.</p>
 */
public class VisionMeasurement {
    public final double xIn;
    public final double yIn;
    public final double headingRad;
    public final double xyStdIn;
    public final double headingStdRad;
    public final long nanoTime;
    public final String source;

    public VisionMeasurement(double xIn, double yIn, double headingRad,
                             double xyStdIn, double headingStdRad,
                             long nanoTime, String source) {
        this.xIn = xIn;
        this.yIn = yIn;
        this.headingRad = PoseKalmanFilter.wrapAngle(headingRad);
        this.xyStdIn = xyStdIn;
        this.headingStdRad = headingStdRad;
        this.nanoTime = nanoTime;
        this.source = source == null ? "vision" : source;
    }
}
