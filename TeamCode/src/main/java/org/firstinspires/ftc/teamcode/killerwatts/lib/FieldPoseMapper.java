package org.firstinspires.ftc.teamcode.killerwatts.lib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Single boundary between robot-frame odometry and the canonical field frame.
 *
 * <p>Robot frame (Pinpoint native): X = forward, Y = left (strafe), H = CCW+.
 * Never changes with alliance or field position.</p>
 *
 * <p>Field frame (FIRST / Road Runner / Pedro): X = left/right across the
 * field (from red-alliance-wall view), Y = forward/back up the field,
 * H = CCW+, origin at field center, inches. This is the ONLY frame anything
 * downstream of this class may use.</p>
 *
 * <p>Conversion is start-pose anchored: field = startPose (+) pinpointDelta,
 * where pinpointDelta is measured from the Pinpoint zero taken at init.
 * Heading 0 with start heading 0 faces field +Y (up-field, away from red wall).
 * At init the robot is assumed to sit exactly at the start pose, so delta is 0.</p>
 */
public final class FieldPoseMapper {
    private FieldPoseMapper() {}

    /** Rotate a robot-frame (fwd,left) delta into field-frame (x,y) by start heading.
     * Field +X = east (right viewed from red wall), +Y = north (up-field).
     * Facing +Y, robot-left = -X (west), so at h=0: dx = -dLeft, dy = +dFwd.
     * forward(h) = (-sin h, cos h), left(h) = (-cos h, -sin h) [forward rotated CCW]. */
    public static double[] robotDeltaToField(double dFwdIn, double dLeftIn, double startHeadingRad) {
        double c = Math.cos(startHeadingRad);
        double s = Math.sin(startHeadingRad);
        double dx = -dFwdIn * s - dLeftIn * c;
        double dy = dFwdIn * c - dLeftIn * s;
        return new double[]{dx, dy};
    }

    /** Full pose: field = start (+) rotated pinpoint delta. Inches + radians. */
    public static double[] pinpointDeltaToField(double dFwdIn, double dLeftIn, double dHeadingRad,
                                                double startXIn, double startYIn, double startHeadingRad) {
        double[] d = robotDeltaToField(dFwdIn, dLeftIn, startHeadingRad);
        return new double[]{startXIn + d[0], startYIn + d[1],
                PoseKalmanFilter.wrapAngle(startHeadingRad + dHeadingRad)};
    }

    /** Same rotation for velocities (start heading only, no position offset). */
    public static double[] robotVelToField(double vFwdInPerSec, double vLeftInPerSec,
                                           double startHeadingRad) {
        return robotDeltaToField(vFwdInPerSec, vLeftInPerSec, startHeadingRad);
    }

    /** Pinpoint Pose2D (mm, robot frame) -> delta inches (fwd, left, rad). */
    public static double[] pinpointPoseToDeltaIn(Pose2D pinpointPose) {
        if (pinpointPose == null) return new double[]{0, 0, 0};
        return new double[]{
                DistanceUnit.MM.toInches(pinpointPose.getX(DistanceUnit.MM)),
                DistanceUnit.MM.toInches(pinpointPose.getY(DistanceUnit.MM)),
                pinpointPose.getHeading(AngleUnit.RADIANS)};
    }

    /** Pinpoint velocity Pose2D (mm/s, robot frame) -> inches/sec (fwd, left, rad/s). */
    public static double[] pinpointVelToDelta(Pose2D pinpointVel) {
        if (pinpointVel == null) return new double[]{0, 0, 0};
        return new double[]{
                DistanceUnit.MM.toInches(pinpointVel.getX(DistanceUnit.MM)),
                DistanceUnit.MM.toInches(pinpointVel.getY(DistanceUnit.MM)),
                pinpointVel.getHeading(AngleUnit.RADIANS)};
    }

    /** Field inches pose -> Pose2D in mm, for path followers that want metric. */
    public static Pose2D fieldInToPose2D(double xIn, double yIn, double headingRad) {
        return new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.RADIANS, headingRad);
    }
}
