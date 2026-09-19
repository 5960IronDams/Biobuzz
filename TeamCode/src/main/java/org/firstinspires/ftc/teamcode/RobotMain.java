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
        CommandF = new CommandFactory(follower,intake,PosServ, opmode.hardwareMap);

        //declare what alliance we are on to BOTH dashboards
        //this is for one good final double check for the driver and co pilot
        opmode.telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        opmode.telemetry.update();
        RobotMain.DashTelemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        RobotMain.DashTelemetry.update();
    }


    public void RobotRunPeriodic()
    {
        follower.update();//updates this robots pedro Followers
        RunPeriodic();//run all registered subsystems periodic
        Scheduler.execute(); //eun the Ivy scheduler periodic
        looptime();
    }
    long lastTime = System.nanoTime();
    public void looptime() {
        long currentTime = System.nanoTime();

        // Calculate loop time in milliseconds
        double loopTimeMs = (currentTime - lastTime) / 1_000_000.0;
        lastTime = currentTime;

        // ... Your Robot Logic Here ...

        RobotMain.DashTelemetry.addData("Loop Time (ms)", "%.2f ms", loopTimeMs);
        RobotMain.DashTelemetry.addData("Hz", "%.1f Hz", 1000.0 / loopTimeMs);
        RobotMain.DashTelemetry.update();
    }
}
