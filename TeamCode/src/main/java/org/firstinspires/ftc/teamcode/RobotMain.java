package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.clearAll;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.killerwatts.Intake;
import org.firstinspires.ftc.teamcode.killerwatts.PositionalServo;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.killerwatts.lib.DashboardFieldRenderer;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class RobotMain {
    public static ALLIANCE_COLOR CurrentAlliance = ALLIANCE_COLOR.ALLIANCE_RED;

    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static Telemetry DashTelemetry = dashboard.getTelemetry();
    public Follower follower;// = SharedObjects.follower;

    public final DashboardFieldRenderer fieldRenderer = new DashboardFieldRenderer();
    public Intake intake;
    public PositionalServo PosServ;
    public static Pose autonomousEndPose = new Pose(0, 0, 0);
//    private static RobotMain robot;
//    public static RobotMain getInstance(OpMode opmode)
//    {
//        if(robot == null)
//        {
//            robot = new RobotMain(opmode);
//        }
//        robot.follower = Constants.create(opmode.hardwareMap);//shouldnt fix as much as it does. pedro pathing 3.0.0 issue?
//        return robot;
//    }
    public RobotMain(OpMode opmode)
    {
        clearAll(); // avoid double-registration on re-run
        follower = Constants.create(opmode.hardwareMap);
        fieldRenderer.drawPedroPose(follower.pose());
        intake = new Intake(opmode);
        PosServ = new PositionalServo(opmode);
    }

}
