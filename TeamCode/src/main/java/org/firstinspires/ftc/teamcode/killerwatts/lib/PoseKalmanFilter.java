package org.firstinspires.ftc.teamcode.killerwatts.lib;

/**
 * Tiny 3-state (x, y, heading) Kalman filter for field tracking.
 *
 * <p>Each axis is an independent 1D Kalman filter with a constant-velocity
 * prediction step. This is intentionally matrix-free so it runs cheaply on the
 * Control Hub and is easy to tune from the dashboard.</p>
 *
 * <p>Units: x/y in inches, heading in radians (wrapped to [-pi, pi]).
 * Variances are in inches^2 / rad^2.</p>
 */
public class PoseKalmanFilter {
    private double x;
    private double y;
    private double heading;

    private double varX;
    private double varY;
    private double varH;

    /** Process noise added per second during prediction. */
    private double procXy;
    private double procH;

    public PoseKalmanFilter(double xIn, double yIn, double headingRad,
                            double initXyStdIn, double initHeadingStdRad,
                            double procXyStdInPerSec, double procHeadingStdRadPerSec) {
        this.x = xIn;
        this.y = yIn;
        this.heading = wrapAngle(headingRad);
        this.varX = initXyStdIn * initXyStdIn;
        this.varY = initXyStdIn * initXyStdIn;
        this.varH = initHeadingStdRad * initHeadingStdRad;
        this.procXy = procXyStdInPerSec * procXyStdInPerSec;
        this.procH = procHeadingStdRadPerSec * procHeadingStdRadPerSec;
    }

    /** Constant-velocity prediction. Velocities in inches/sec and rad/sec. */
    public synchronized void predict(double vxInPerSec, double vyInPerSec, double omegaRadPerSec, double dtSec) {
        if (dtSec <= 0) return;
        x += vxInPerSec * dtSec;
        y += vyInPerSec * dtSec;
        heading = wrapAngle(heading + omegaRadPerSec * dtSec);
        varX += procXy * dtSec;
        varY += procXy * dtSec;
        varH += procH * dtSec;
    }

    /** Fuse an absolute (x, y) measurement with the given std dev (inches). */
    public synchronized void updateXY(double measXIn, double measYIn, double xyStdIn) {
        double r = Math.max(xyStdIn * xyStdIn, 1e-9);
        double kx = varX / (varX + r);
        double ky = varY / (varY + r);
        x += kx * (measXIn - x);
        y += ky * (measYIn - y);
        varX *= (1.0 - kx);
        varY *= (1.0 - ky);
    }

    /** Fuse an absolute heading measurement with the given std dev (radians). */
    public synchronized void updateHeading(double measHeadingRad, double headingStdRad) {
        double r = Math.max(headingStdRad * headingStdRad, 1e-12);
        double k = varH / (varH + r);
        heading = wrapAngle(heading + k * angleDiff(wrapAngle(measHeadingRad), heading));
        varH *= (1.0 - k);
    }

    public synchronized void reset(double xIn, double yIn, double headingRad,
                                   double xyStdIn, double headingStdRad) {
        this.x = xIn;
        this.y = yIn;
        this.heading = wrapAngle(headingRad);
        this.varX = xyStdIn * xyStdIn;
        this.varY = xyStdIn * xyStdIn;
        this.varH = headingStdRad * headingStdRad;
    }

    public synchronized double getX() { return x; }
    public synchronized double getY() { return y; }
    public synchronized double getHeading() { return heading; }
    public synchronized double getStdX() { return Math.sqrt(Math.max(varX, 0)); }
    public synchronized double getStdY() { return Math.sqrt(Math.max(varY, 0)); }
    public synchronized double getStdH() { return Math.sqrt(Math.max(varH, 0)); }

    public static double wrapAngle(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    /** Signed smallest difference target - current, in [-pi, pi]. */
    public static double angleDiff(double target, double current) {
        return wrapAngle(target - current);
    }
}
