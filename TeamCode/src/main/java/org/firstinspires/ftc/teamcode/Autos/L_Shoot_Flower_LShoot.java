package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFollow.follow;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

//@Autonomous(name = "Red_R_Shoot_Park",group = "Autos")
public class L_Shoot_Flower_LShoot extends AutoOpModeBase {

    public L_Shoot_Flower_LShoot()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/L_Shoot_Flower_LShoot.pp.json";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitCommand(9000),
                //goto easier shot
                follow(robot.follower, pp.path("Toshoot")),
                //fire or whatever.
                autoShoot().withTimeout(5000),
                //goto flower and intake
                new ParallelCommandGroup(
                        rampDown(),
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("ToFlower")).withTimeout(5000)
                ),
                //intake at flower for a bit
                new WaitCommand(3500),
                //turn off intake, ramp up, and goto left shoot.
                new ParallelCommandGroup(
                        rampUp(),
                        follow(robot.follower, pp.path("ToLeftShoot")).withTimeout(5000)
                ),
                //fire at Hive L.
                autoShoot().withTimeout(5000),
                //stop after shooting
                robot.intake.stopCmd()

        );
    }
}
