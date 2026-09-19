package org.firstinspires.ftc.teamcode.killerwatts.lib;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * @deprecated Renamed to {@link PanelsFieldRenderer} when FTC Dashboard was
 * removed in favor of FTControl Panels. Kept as a thin subclass so old
 * imports/call sites keep compiling; new code should use
 * {@link PanelsFieldRenderer} directly.
 */
@Deprecated
public class DashboardFieldRenderer extends PanelsFieldRenderer {

    public DashboardFieldRenderer() {
        super();
    }

    @Override
    public void drawFieldPose(double xFieldIn, double yFieldIn, double headingFieldRad) {
        super.drawFieldPose(xFieldIn, yFieldIn, headingFieldRad);
    }

    @Override
    public void drawFieldPose(double xFieldIn, double yFieldIn, double headingFieldRad,
                              String stroke, String fill) {
        super.drawFieldPose(xFieldIn, yFieldIn, headingFieldRad, stroke, fill);
    }

    @Override
    public void drawFtcPose(Pose2D fieldPoseInches) {
        super.drawFtcPose(fieldPoseInches);
    }

    @Override
    public void drawPedroPose(Pose pedroPose) {
        super.drawPedroPose(pedroPose);
    }

    @Override
    public void drawRobot(double xFieldIn, double yFieldIn, double headingFieldRad) {
        super.drawRobot(xFieldIn, yFieldIn, headingFieldRad);
    }
}
