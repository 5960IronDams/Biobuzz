package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

//@Autonomous(name = "Red_AutoOpMode", group = "Autos")
@Disabled
public abstract class Red_AutoOpMode extends OpMode {
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
            // See PPFile.blueRotationFactory (built on Pedro's mirrorAroundPoint).
            poseFactory = PPFile.blueRotationFactory(IronConstants.FieldCenter.x(), IronConstants.FieldCenter.y());
        }
        // after poseFactory is configured for Red/Blue load the pathfile:
        try {
            pp = PPFile.fromAsset(hardwareMap, assetPath, poseFactory);
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
        robot.flushTelemetry(); // init() runs once — flush here is correct
    }

    @Override
    public void init_loop() {
        // init_loop runs every cycle while sitting on the init screen.
        // Re-assert the start pose each time so the Panels Field always shows
        // THIS auto's start — even if another auto was previewed before,
        // or the localizer drifted while waiting.
        if (robot == null || pp == null || robot.follower == null) return;
        robot.follower.setPose(pp.getStartPose());
        robot.follower.update();
        UpdateTelemetry();
        robot.flushTelemetry();
    }



    @Override
    public void start() {
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
        // Mirror to Panels + draw (addData only — caller flushes once at the end).
        RobotMain.DashTelemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        RobotMain.DashTelemetry.addData("X", robot.follower.pose().x());
        RobotMain.DashTelemetry.addData("Y", robot.follower.pose().y());
        RobotMain.DashTelemetry.addData("Heading", Math.toDegrees(robot.follower.pose().heading()));
        RobotMain.DashTelemetry.addData("Follower Mode", robot.follower.mode());
        // Draw the Pedro pose on the Panels field widget (Pedro-native;
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