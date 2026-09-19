package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

@Autonomous(name = "Red_RightAuto",group = "Autos")
public class Red_RightAuto extends Red_AutoOpMode {

    public Red_RightAuto()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/exampleAuto1.pp";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
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
}