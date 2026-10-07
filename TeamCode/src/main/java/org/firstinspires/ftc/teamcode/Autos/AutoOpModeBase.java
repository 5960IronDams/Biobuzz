package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.PoseFactory;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PPAssets;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PPFile;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionFusion;

//@Autonomous(name = "Red_AutoOpMode", group = "Autos")
@Disabled
public abstract class AutoOpModeBase extends OpMode {
    public RobotMain robot;
    public PoseFactory poseFactory = PoseFactory.degrees();//we design for red, then mirror for blue
    /**
     *  <em>Must</em> be overridden inside Constructor Or Example  paths Will be Loaded!
     */
    public String assetPath = "pathfiles/exampleAuto1.pp";;
    /**
     *  The Alliance This Auton is for, Default Red! Must Be Overridden in child constructor (unless red)
     */
    public ALLIANCE_COLOR SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    public PPFile pp;//loads and manages our pathFiles we create in the pedro path visualizer. (visualizer.pedropathing.com)

    /**
     *  <em>Must</em> be overridden for auton to have any actions!
     */
    public abstract Command autoRoutine();//Must be overridden, this is the command that runs, this is your auto.

    @Override
    public void init() {
        //assign our auto our alliance color
        RobotMain.CurrentAlliance = SetRobotToThisColor;
        //if blue rotate our heading and factory.
        if(RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE)
        {
            // 180-degree rotation of the Red design around the field center.
            poseFactory = PoseFactory.degrees().mirrorAroundPoint(IronConstants.FieldCenter.x(), IronConstants.FieldCenter.y());
        }
        // after poseFactory is configured for Red/Blue load the pathfile:
        pp = PPAssets.fromAsset(hardwareMap, assetPath, poseFactory);

        //setupPathPoints();

        //now that Alliance is settled : setup all our subsystems and command factory
        robot = new RobotMain(this);

        //for autons, set our robot in the known starting place
        robot.follower.setPose(pp.startPoint());
        robot.follower.update();

        //update telemetry and draw where our robot is on the field.
        UpdateTelemetry();
        RobotMain.DashTelemetry.addData("AutonEndPose", "Unsaved");
        robot.flushTelemetry(); // init() runs once — flush here is correct
    }

    @Override
    public void init_loop() {
        // init_loop runs every cycle while sitting on the init screen.
        // Vision ticks here (LL connect + solve + fuse) so the drivers see the
        // real fused pose on the Field widget BEFORE pressing play.
        robot.InitRunPeriodic();

        // Re-assert the start pose AFTER the vision tick so the Panels Field
        // always shows THIS auto's start — even if another auto was previewed
        // before, or the (parked) odometry/vision drifted while waiting. This
        // keeps init corrections display-only: the pose the auto launches from
        // is always the path's start pose.
        //
        // OR disable to accept the vision and not reset the pose from auton init
        if(!VisionFusion.ENABLED){}robot.follower.setPose(pp.startPoint());
        robot.follower.update();
        UpdateTelemetry();
        robot.flushTelemetry();
    }



    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(autoRoutine());
    }
    @Override
    public void stop() {
        if (robot != null) {
            try {
                RobotMain.autonomousEndPose = robot.follower.pose(); //saves your position in that file
            } catch (Exception ignored) {
            }
            robot.shutdown();
        }
        telemetry.addData("AutonEndPose", "saved");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.RobotRunPeriodic();
        UpdateTelemetry();
        robot.flushTelemetry(); // single flush LAST — see RobotMain.flushTelemetry()
    }


    private void UpdateTelemetry()
    {
        telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        telemetry.addData("X", robot.follower.pose().x());
        telemetry.addData("Y", robot.follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.pose().heading()));
        telemetry.addData("Follower Mode", robot.follower.mode());
        telemetry.update();
        // Mirror to Panels (addData only — flushTelemetry() draws the field once
        // at the end of the tick, so init()/init_loop() paint too).
        RobotMain.DashTelemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        RobotMain.DashTelemetry.addData("X", robot.follower.pose().x());
        RobotMain.DashTelemetry.addData("Y", robot.follower.pose().y());
        RobotMain.DashTelemetry.addData("Heading", Math.toDegrees(robot.follower.pose().heading()));
        RobotMain.DashTelemetry.addData("Follower Mode", robot.follower.mode());
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