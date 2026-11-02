package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFollow.follow;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

//@Autonomous(name = "Red_RightAuto",group = "Autos")
public class R_ShootR_Garden_ShootR_Park extends AutoOpModeBase {

    public R_ShootR_Garden_ShootR_Park()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/R_ShootR_Garden_ShootR_Park.pp";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
        // Build the command group up-front. autoShoot() hands back a FRESH
        // instance per call (each one may join exactly ONE command group), so
        // every shot below needs its own — do not hoist a single local.
        return new SequentialCommandGroup(
                //fire with the auto fire command for 6 seconds.
                autoShoot().withTimeout(4000),
                //move into garden and pickup the 4 pollen there, run intake and drive into corner
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("GardenCorner"))
                ),

                // move to shoot location on right side for second attempt.
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("shootcloseleft"))
                ),
                //stop intaking to shoot
                robot.intake.stopCmd(),
                //shoot for 6 seconds (withtimeout?)
                autoShoot().withTimeout(4000),
                //after shooting intake over to park place.
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("parkplace"))
                ),
                //stop intakeing and be parked.
                robot.intake.stopCmd()

        );
    }
}
