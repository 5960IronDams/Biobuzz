package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GameConst;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.clearAll;

@Autonomous(name = "Red_ExampleAuto1", group = "Autos")
public class Red_ExampleAuto extends OpMode {
    public RobotMain robot;
    public PoseFactory poseFactory = PoseFactory.degrees();//we design for red, then mirror for blue
    public ALLIANCE_COLOR SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for

    @Override
    public void init() {
        RobotMain.CurrentAlliance = SetRobotToThisColor;
        if(RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE)
        {
            poseFactory = PoseFactory.degrees().mirrorY(GameConst.FieldCenter.y()).mirrorX(GameConst.FieldCenter.x());//we design for red, then mirror for blue
        }

        setupPathPoints();
        robot = new RobotMain(this);
        clearAll();
        Scheduler.reset();
        //robot.follower = Constants.create(hardwareMap);
        robot.follower.setPose(start);
        robot.follower.update();
        UpdateTelemetry();
        RobotMain.DashTelemetry.addData("AutonEndPose", "Unsaved");
        telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        telemetry.update();
        RobotMain.DashTelemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        RobotMain.DashTelemetry.update();
    }



    @Override
    public void start() {
        //schedule(follow(follower, parkonly()));
        schedule(autoRoutine());
    }
    @Override
    public void stop() {
        RobotMain.autonomousEndPose = robot.follower.pose(); //saves your position in that file
        telemetry.addData("AutonEndPose", "saved");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.follower.update();
        Scheduler.execute();
        UpdateTelemetry();

    }
    private void UpdateTelemetry()
    {
        telemetry.addData("X", robot.follower.pose().x());
        telemetry.addData("Y", robot.follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.pose().heading()));
        telemetry.addData("Follower Mode", robot.follower.mode());
        telemetry.update();
        // Draw the Pedro pose on the dashboard field overlay (corner-origin -> field frame
        // conversion handled inside the renderer).
        if (robot.follower != null && robot.follower.pose() != null) {
            robot.fieldRenderer.drawPedroPose(robot.follower.pose());
        }
    }
    private Command autoRoutine() {
        return sequential(
                follow(robot.follower, path1()),
                // Add mechanism commands here.
                parallel(
                        RunIntake,
                        follow(robot.follower, path2())
                        ),
                StopIntake
        );
    }

    Command RunIntake = instant(() -> robot.intake.runIntake());
    Command StopIntake = instant(() -> robot.intake.stop());

    private void setupPathPoints() {//this exist to ensure Blue works. this pattern needs to be changed.
        start = poseFactory.of(55, 9, 90);
        path1 = poseFactory.of(36.0, 9.75, 60);
        point2 = poseFactory.of(36.0, 27.0, 0);
    }
    //points used in this auton.
    private Pose start = null;
    private Pose path1 = null;
    private Pose point2 = null;
    
    //path creators. 
    public Path path1() {
        return line(start, path1).linear(path1,start );//why is linear backwards !?
    }

    public Path path2() {
        return line(path1, point2).linear(point2,path1);//why is linear backwards !?
    }
}