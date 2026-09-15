package org.firstinspires.ftc.teamcode.killerwatts.lib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Converts robot-local poses (e.g. from Pinpoint, which starts at 0,0,0 and
 * reports X-forward mm / Y-left mm / heading-CCW) into FTC Dashboard field
 * inches (X-right, Y-up from field center, heading CCW).
 */
public final class FieldPoseMapper {
    private FieldPoseMapper() {}

    /** Pinpoint pose (mm, deg) -> field inches (X-right, Y-up, rad CCW). */
    public static double[] pinpointToFieldIn(Pose2D pinpointPose) {
        if (pinpointPose == null) return new double[]{0, 0, 0};
        double pinXmm = pinpointPose.getX(DistanceUnit.MM); // forward
        double pinYmm = pinpointPose.getY(DistanceUnit.MM); // left
        double headingRad = pinpointPose.getHeading(AngleUnit.RADIANS);
        // Pinpoint: +forward, +left. Field overlay: +right, +up.
        // At heading 0 they face field +X(up): X_field = forward, Y_field = -left(strafe-right).
        double xIn = DistanceUnit.MM.toInches(pinXmm);
        double yIn = DistanceUnit.MM.toInches(-pinYmm);
        return new double[]{xIn, yIn, headingRad};
    }

    /** Pinpoint velocity (mm/s, deg/s) -> field inches/sec + rad/sec. */
    public static double[] pinpointVelToField(Pose2D pinpointVel) {
        if (pinpointVel == null) return new double[]{0, 0, 0};
        double vxIn = DistanceUnit.MM.toInches(pinpointVel.getX(DistanceUnit.MM));
        double vyIn = DistanceUnit.MM.toInches(-pinpointVel.getY(DistanceUnit.MM));
        double omega = pinpointVel.getHeading(AngleUnit.RADIANS);
        return new double[]{vxIn, vyIn, omega};
    }
}
