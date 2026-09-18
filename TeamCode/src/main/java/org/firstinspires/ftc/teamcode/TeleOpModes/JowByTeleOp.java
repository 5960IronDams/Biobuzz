package org.firstinspires.ftc.teamcode.TeleOpModes;

import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.RunPeriodic;
import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.clearAll;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

@TeleOp(name = "JowBy_TeleOp", group = "TeleOp")//red_

public class JowByTeleOp extends OpMode {

    public RobotMain robot;

    @Override
    public void init() {

        robot = new RobotMain(this);
        telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        telemetry.update();
        RobotMain.DashTelemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        RobotMain.DashTelemetry.update();
        //setAutonPose();


    }
    public void PedroFollowerFieldCentricDrivetrainloop()
    {
        if(RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_RED) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    robot.follower.pose().heading()
            );
            robot.follower.manual(powers);
        }
        else{//we are on blue alliance
            DrivePowers powers = ManualDrive.fieldCentric(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    robot.follower.pose().heading()
            );
            robot.follower.manual(powers);
        }


        // relocalise button
        if (gamepad1.startWasPressed()) {
            Pose ResetPose = new Pose(10.5, 10.5, Math.toRadians(0));//red alliance reset pose
            if(RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE) {ResetPose = new Pose(20.5, 20.5, Math.toRadians(180));}//blue alliance reset pose
            robot.follower.setPose(ResetPose); // overrides our pose
        }

        robot.follower.update();
        robot.fieldRenderer.drawPedroPose(robot.follower.pose());
        Pose robotPose = robot.follower.pose(); // returns a Pose object
        RobotMain.DashTelemetry.addData("Robot X", robotPose.x());
        RobotMain.DashTelemetry.addData("Robot Y", robotPose.y());
        RobotMain.DashTelemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }
    @Override
    public void start() {

    }

    public void setAutonPose()
    {
        robot.follower.setPose(RobotMain.autonomousEndPose);
        robot.follower.update();
        robot.fieldRenderer.drawPedroPose(robot.follower.pose());
        RobotMain.DashTelemetry.addData("AutonEndPose", "SetInTeleOp");
        RobotMain.DashTelemetry.update();
    }

    @Override
    public void loop() {
        RunPeriodic();
        PedroFollowerFieldCentricDrivetrainloop();
        UpdateTelemetry();
    }
    public void UpdateTelemetry()
    {
        RobotMain.DashTelemetry.addData("Status", "Running");
        RobotMain.DashTelemetry.update();
    }
}
