package org.firstinspires.ftc.teamcode.Autos;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import static org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFollow.follow;

import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

public class L_ShootLFlowerShootL extends AutoOpModeBase {

    public L_ShootLFlowerShootL()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/_L_ShootLFlowerShootL.pp";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                //wait for hive to tip to left
                new WaitCommand(3000),
                //shoot pre-load into hive
                robot.CommandF.ServoToPos(0.5),
                //drive to flower ready position
                follow(robot.follower, pp.getPathByLineName("ReadyFlowerPos")),
                // Drive into flower command group
                new ParallelCommandGroup(
                        //drop servo for flowerintake scoop
                        //activate intake
                        robot.CommandF.RunIntake(),
                        //go into flower
                        follow(robot.follower, pp.getPath("IntoFlower1"))
                ),
                //wait for all pollen to be intaken
                new WaitCommand(500),
                robot.CommandF.StopIntake(),
                //drive back to second shot on left hive
                new SequentialCommandGroup(
                        follow(robot.follower, pp.getPath("Shot2Left"))
                ),
                //drive to park while intaking
                robot.CommandF.RunIntake(),
                follow(robot.follower, pp.getPath("ToPark")),
                //ready for teleop?
                robot.CommandF.ServoToPos(1.0),
                new WaitCommand(1500),
                robot.CommandF.ServoToPos(0.0)

        );
    }
}
