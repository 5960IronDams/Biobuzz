package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

public class _L_ShootLFlowerShootL extends AutoOpModeBase {

    public _L_ShootLFlowerShootL()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/_L_ShootLFlowerShootL.pp";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
        return sequential(
                //wait for hive to tip to left
                waitMs(3000),
                //shoot pre-load into hive
                robot.CommandF.ServoToPos(0.5),
                //drive to flower ready position
                follow(robot.follower, pp.getPathByLineName("ReadyFlowerPos")),
                // Drive into flower command group
                parallel(
                        //drop servo for flowerintake scoop
                        //activate intake
                        robot.CommandF.RunIntake(),
                        //go into flower
                        follow(robot.follower, pp.getPath("IntoFlower1"))
                ),
                //wait for all pollen to be intaken
                waitMs(500),
                robot.CommandF.StopIntake(),
                //drive back to second shot on left hive
                sequential(
                        follow(robot.follower, pp.getPath("Shot2Left"))
                ),
                //drive to park while intaking
                robot.CommandF.RunIntake(),
                follow(robot.follower, pp.getPath("ToPark")),
                //ready for teleop?
                robot.CommandF.ServoToPos(1.0),
                waitMs(1500),
                robot.CommandF.ServoToPos(0.0)

        );
    }
}