package org.firstinspires.ftc.teamcode.Autos;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import static org.firstinspires.ftc.teamcode.pedro.PedroFollow.follow;

import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

//@Autonomous(name = "Red_RightAuto",group = "Autos")
public class _RightAuto extends AutoOpModeBase {

    public _RightAuto()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/exampleAuto1.pp";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                //fire or whatever.
                robot.CommandF.ServoToPos(0.5),
                //move out of way
                follow(robot.follower, pp.getPathByLineName("StartToOffset")),
                // move to next location
                new ParallelCommandGroup(
                        robot.CommandF.RunIntake(),
                        follow(robot.follower, pp.getPathByLineName("OffsetToPark"))
                ),
                //stop intakeing and other functions.
                robot.CommandF.StopIntake(),
                robot.CommandF.ServoToPos(1.0),
                new WaitCommand(1500),
                robot.CommandF.ServoToPos(0.0)

        );
    }
}
