package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.RunPeriodic;
import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.clearAll;

@Autonomous(name = "Red_ExampleAuto1", group = "Autos")
public class Red_ExampleAuto extends OpMode {
    public RobotMain robot;
    public PoseFactory poseFactory = PoseFactory.degrees();//we design for red, then mirror for blue
    public ALLIANCE_COLOR SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    public PPFile pp;//loads and manages our pathFiles we create in the pedro path visualizer. (visualizer.pedropathing.com)
    @Override
    public void init() {
        //assign our auto our alliance color
        RobotMain.CurrentAlliance = SetRobotToThisColor;
        //if blue rotate our heading and factory.
        if(RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE)
        {
            // 180-degree rotation of the Red design around the field center.
            // Do NOT use mirrorY().mirrorX() here: Pedro's mirrorX maps heading
            // h -> -h, which is only correct for +/-90 deg (Red 90 -> Blue 270
            // works, but Red 60 -> Blue 300 instead of 240, Red 0 -> Blue 0
            // instead of 180). blueRotationFactory maps h -> h + PI, correct for
            // all headings. See PPFile.blueRotationFactory for the full analysis.
            poseFactory = PPFile.blueRotationFactory(IronConstants.FieldCenter.x(), IronConstants.FieldCenter.y());
        }
        // after poseFactory is configured for Red/Blue load the pathfile:
        try {
            pp = PPFile.fromAsset(hardwareMap, "pathfiles/exampleAuto1.pp", poseFactory);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        //setupPathPoints();

        //now that Alliance is settled : setup all our subsystems and command factory
        robot = new RobotMain(this);

        //for autons, set our robot in the known starting place
        robot.follower.setPose(pp.getStartPose());
        robot.follower.update();

        //update telemetry and draw where our robot is on the field.
        UpdateTelemetry();
        RobotMain.DashTelemetry.addData("AutonEndPose", "Unsaved");
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
        robot.RobotRunPeriodic();
        UpdateTelemetry();

    }

    private Command autoRoutine() {
        return sequential(
                robot.CommandF.ServoToPos(0.5),
                follow(robot.follower, pp.getPath("StartPoint")),
                // Add mechanism commands here.
                parallel(
                        robot.CommandF.RunIntake(),
                        follow(robot.follower, pp.getPath("StartToOffset"))
                        ),
                robot.CommandF.StopIntake(),
                robot.CommandF.ServoToPos(1.0),
                waitMs(1500),
                robot.CommandF.ServoToPos(0.0)

        );
    }
    private void UpdateTelemetry()
    {
        telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
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



//    private void setupPathPoints() {//this exist to ensure Blue works. this pattern needs to be changed. //blue requires posefactory to be mirrored.
//        start = poseFactory.of(55, 9, 90);
//        path1 = poseFactory.of(36.0, 9.75, 60);
//        point2 = poseFactory.of(36.0, 27.0, 0);
//    }
//    //points used in this auton.
//    private Pose start = null;
//    private Pose path1 = null;
//    private Pose point2 = null;
//
//    //path creators.
//    public Path path1() {
//        return line(start, path1).linear(path1,start );//why is linear backwards !?
//    }
//
//    public Path path2() {
//        return line(path1, point2).linear(point2,path1);//why is linear backwards !?
//    }
}