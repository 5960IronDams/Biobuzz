package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class IronConstants {
    public static final Pose FieldXYMaxPoint = new Pose(141.0,141.0,0);
    public static final Pose FieldCenter = new Pose(FieldXYMaxPoint.x()/2,FieldXYMaxPoint.y()/2,0);

    public static final String FLMotor = "leftFront";
    public static final String FRMotor = "rightFront";
    public static final String RLMotor = "leftBack";
    public static final String RRMotor = "rightBack";
    public static final String PinpointOdometryName = "odo";


    public static final String IntakeMotorName = "intake";
    public static final String FlywheelMotorName = "flywheel";
    public static final String FrontServoName = "shootgate";

    // Alliance reset poses for extreme localization loss, should be in a known location.
    public static final Pose RED_RESET_POSE = new Pose(10.5, 10.5, Math.toRadians(0));
    public static final Pose BLUE_RESET_POSE = new Pose(20.5, 20.5, Math.toRadians(180));


    //TODO: Test todo.
    //TODO: panels custom to make website open fast also make 1.0.7 biobuzz version work.
    //todo: try ftclib/Dairy/solverslib
    //todo: commandgamepad? triggers? linear-interpolation table.
    //todo: real Limelight OS and fmaps.
    //todo LL draw onto dashboard map our estimated Pose.
}
