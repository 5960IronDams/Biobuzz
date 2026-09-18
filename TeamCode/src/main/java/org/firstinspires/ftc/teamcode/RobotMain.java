package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.RunPeriodic;
import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.clearAll;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
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
    public final DashboardFieldRenderer fieldRenderer = new DashboardFieldRenderer();
    //subsystems
    public Follower follower;// = SharedObjects.follower;
    public Intake intake;
    public PositionalServo PosServ;
    //
    public CommandFactory CommandF;
    public static Pose autonomousEndPose = new Pose(0, 0, 0);
    public RobotMain(OpMode opmode)
    {
        clearAll(); // avoid double-registration of subsystem on re-run or Opmode switch
        Scheduler.reset(); //clears all scheduler commands in ivy after opmode switch.
        //subsystems
        follower = Constants.create(opmode.hardwareMap);
        fieldRenderer.drawPedroPose(follower.pose());
        intake = new Intake(opmode);
        PosServ = new PositionalServo(opmode);


        //after all subsystems are started (i.e their variables point to an object). Build the command factory
        CommandF = new CommandFactory(follower,intake,PosServ);

    }

    public void RobotRunPeriodic()
    {
        follower.update();//updates this robots pedro Followers
        RunPeriodic();//run all registered subsystems periodic
        Scheduler.execute(); //eun the Ivy scheduler periodic
    }

}
