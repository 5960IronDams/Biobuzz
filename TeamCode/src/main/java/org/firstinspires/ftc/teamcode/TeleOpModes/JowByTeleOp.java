package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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


    }
    @Override
    public void start() {

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
        // addData only — flushed once via flushTelemetry() at end of loop().
        RobotMain.DashTelemetry.addData("Status", "Running");
    }
}
