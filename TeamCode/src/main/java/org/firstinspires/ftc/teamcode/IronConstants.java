package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class IronConstants {
    public static final Pose FieldXYMaxPoint = new Pose(141.0,141.0,0);
    public static final Pose FieldCenter = new Pose(FieldXYMaxPoint.x()/2,FieldXYMaxPoint.y()/2,0);

    public static final String IntakeMotorName = "intake";
    public static final String FrontServoName = "eeerrr";

    //TODO: Test todo.
    //TODO: panels custom to make website open fast also make 1.0.7 biobuzz version work.
    //todo: try ftclib/Dairy/solverslib
    //todo: commandgamepad? triggers? linear-interpolation table.
}
