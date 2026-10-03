package org.firstinspires.ftc.teamcode.Autos;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import static org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFollow.follow;

import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

//@Autonomous(name = "Red_R_Shoot_Park",group = "Autos")
public class R_ShootPark extends AutoOpModeBase {

    public R_ShootPark()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/Red_R_Shoot_Park.pp";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                //shoot
                robot.PosServ.toPos(0.5),
                robot.intake.runCmd(),
                //goto park
                follow(robot.follower, pp.getPathByLineName("ParkPlace"))
        );
    }
}
