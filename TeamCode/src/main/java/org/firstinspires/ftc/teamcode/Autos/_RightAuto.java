package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.killerwatts.lib.AutoOpMode;

//@Autonomous(name = "Red_RightAuto",group = "Autos")
public class _RightAuto extends AutoOpMode {

    public _RightAuto()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/exampleAuto1.pp";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
        return sequential(
                //fire or whatever.
                robot.CommandF.ServoToPos(0.5),
                //move out of way
                follow(robot.follower, pp.getPathByLineName("StartToOffset")),
                // move to next location
                parallel(
                        robot.CommandF.RunIntake(),
                        follow(robot.follower, pp.getPathByLineName("OffsetToPark"))
                ),
                //stop intakeing and other functions.
                robot.CommandF.StopIntake(),
                robot.CommandF.ServoToPos(1.0),
                waitMs(1500),
                robot.CommandF.ServoToPos(0.0)

        );
    }
}