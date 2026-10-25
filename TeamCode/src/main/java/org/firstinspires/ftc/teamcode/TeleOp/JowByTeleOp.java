package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareHealth;
import org.firstinspires.ftc.teamcode.RobotMain;

@TeleOp(name = "JowBy_TeleOp", group = "TeleOp")//red_

public class JowByTeleOp extends OpMode {

    public RobotMain robot;
    public KeyBindings keys;

    @Override
    public void init() {
        //now that Alliance is settled - setup all our subsystems and command factory
        robot = new RobotMain(this);
        keys = new KeyBindings(this, robot);
        //setAutonPose();

        // Flush once so the telemetry log captures the init phase too (without
        // this, the log jumps straight from "RobotMain initialized" to the
        // first loop tick after play is pressed).
        robot.flushTelemetry();
    }

    @Override
    public void init_loop() {
        // Vision ticks during init: the LL connects, solves, and the Field
        // widget shows the real fused pose (drivers can verify the seed pose
        // against MT1/MT2 BEFORE pressing play).
        robot.InitRunPeriodic();
        UpdateTelemetry();
        robot.flushTelemetry();
    }

    @Override
    public void start() {
        // SESSION 5d: between init and PLAY the periodic loop is not running,
        // so the turret can be hand-spun in that gap without the tracker seeing
        // it. Re-run the CRT boot solve so the tracked position is re-anchored
        // to wherever the turret actually is when the match begins.
        if (robot != null && robot.turret != null) {
            robot.turret.rezeroAtStart();
        }
    }
    @Override
    public void loop() {
        robot.RobotRunPeriodic();
        keys.update();
        UpdateTelemetry();
        robot.flushTelemetry(); // single flush LAST — see RobotMain.flushTelemetry()
    }
    @Override
    public void stop() {
        if (keys != null) {
            keys.stop();
        }
        if (robot != null) {
            // Fused pose -> Pinpoint: the next OpMode seeds from where we
            // actually are (vision-corrected), not the raw drifted odometry.
            robot.saveFusedPoseToPinpoint();
            robot.shutdown();
        }
    }


//    public void setAutonPose()
//    {
//        robot.follower.setPose(RobotMain.autonomousEndPose);
//        robot.follower.update();
//        robot.fieldRenderer.drawPedroPose(robot.follower.pose());
//        RobotMain.DashTelemetry.addData("AutonEndPose", "SetInTeleOp");
//        RobotMain.DashTelemetry.update();
//    }


    public void UpdateTelemetry()
    {
        // DS screen: re-added every tick because update() clears the buffer.
        // "HW" is the driver-station failure board (see HardwareHealth) — the
        // game-day surface when the Panels laptop is in the pits.
        telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        telemetry.addData("HW", HardwareHealth.get().dsLine());
        telemetry.update();

        // addData only — flushed once via flushTelemetry() at end of loop().
        RobotMain.DashTelemetry.addData("Status", "Running");
    }
}
