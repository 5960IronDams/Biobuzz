package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.PoseFactory;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HardwareHealth;
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
    public String assetPath = "pathfiles/exampleAuto1.pp.json";;
    /**
     *  The Alliance This Auton is for, Default Red! Must Be Overridden in child constructor (unless red)
     */
    public ALLIANCE_COLOR SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    public PPFile pp;//loads and manages our pathFiles we create in the pedro path visualizer. (visualizer.pedropathing.com)
    private long lastDsTelemetryMs = 0L;

    /**
     * Must be overridden to return the autonomous command sequence.
     */
    public abstract Command autoRoutine();

    /** Returns a fresh auto-shoot command for this auto's command group (null if turret is absent). */
    public Command autoShoot() {
        return robot == null ? null : robot.CommandF.AutoShootAtGoal();
    }

    /**
     * A FRESH manual-shoot command for this auto's command group (fixed RPM, no turret movement).
     */
    public Command manualShoot() {
        return robot == null ? null : robot.CommandF.ManualShootAtGoal();
    }

    /**
     * A fresh ramp-down command for this auto's command group (no-op when ramp servos are absent).
     */
    public Command rampDown() {
        return robot == null || robot.ramp == null ? new InstantCommand() : robot.ramp.downCmd();
    }

    /**
     * A fresh ramp-up command for this auto's command group (no-op when ramp servos are absent).
     */
    public Command rampUp() {
        return robot == null || robot.ramp == null ? new InstantCommand() : robot.ramp.upCmd();
    }

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
        UpdateTelemetry(true);
        RobotMain.DashTelemetry.addData("AutonEndPose", "Unsaved");
        robot.flushTelemetry(); // init() runs once — flush here is correct
    }

    @Override
    public void init_loop() {
        robot.InitRunPeriodic();
        // When vision is disabled, keep re-asserting the path start pose during init.
        if (!VisionFusion.ENABLED) {
            robot.follower.setPose(pp.startPoint());
        }
        robot.follower.update();
        UpdateTelemetry(false);
        robot.flushTelemetry();
    }

    @Override
    public void start() {
        // Re-solve turret angle at PLAY in case the turret was moved during init.
        if (robot != null && robot.turret != null) {
            robot.turret.rezeroAtStart();
        }
        CommandScheduler.getInstance().schedule(autoRoutine());
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.saveFusedPoseToPinpoint();
            robot.shutdown();
        }
        telemetry.addData("AutonEndPose", "saved");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.RobotRunPeriodic();
        UpdateTelemetry(false);
        robot.flushTelemetry();
    }

    private void UpdateTelemetry(boolean force)
    {
        long nowMs = System.currentTimeMillis();
        // Rate-limit Driver Station updates to ~10 Hz.
        if (force || nowMs - lastDsTelemetryMs >= 100L) {
            telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
            telemetry.addData("X", robot.follower.pose().x());
            telemetry.addData("Y", robot.follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(robot.follower.pose().heading()));
            telemetry.addData("Follower Mode", robot.follower.mode());
            telemetry.addData("HW", HardwareHealth.get().dsLine());
            telemetry.update();
            lastDsTelemetryMs = nowMs;
        }
        if (RobotMain.DASHBOARD) {
            RobotMain.DashTelemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
            RobotMain.DashTelemetry.addData("X", robot.follower.pose().x());
            RobotMain.DashTelemetry.addData("Y", robot.follower.pose().y());
            RobotMain.DashTelemetry.addData("Heading", Math.toDegrees(robot.follower.pose().heading()));
            RobotMain.DashTelemetry.addData("Follower Mode", robot.follower.mode());
        }
    }
}