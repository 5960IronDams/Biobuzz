package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFollow.follow;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

//@Autonomous(name = "Red_R_Shoot_Park",group = "Autos")
public class R_Shoot_Under_Park extends AutoOpModeBase {

    public R_Shoot_Under_Park()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/R_Shoot_Under_park.pp.json";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
        return new SequentialCommandGroup(

                //goto easier shot
                follow(robot.follower, pp.path("Toshoot")),
                //fire or whatever.
                autoShoot().withTimeout(6000),
                //get out ot twisted devils way
                follow(robot.follower, pp.path("ToUnderHive")),
                //goto park
                follow(robot.follower, pp.path("ToPark"))
        );
    }
}
