package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFollow.follow;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

public class ShootAndParkFlowerSide extends AutoOpModeBase {

    public ShootAndParkFlowerSide() {
        super.assetPath = "pathfiles/shootandparkflowerside.pp.json";
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;
    }

    @Override
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                // Preload shoot (uncomment/customize if needed)
                autoShoot().withTimeout(4000),

                // Path 0: Drive to flower pre-step
                follow(robot.follower, pp.path("flower pre-step")),

                // Path 1: Collect from flower / shoot
                rampDown(),
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("flower collects / shoots"))
                ),
                robot.intake.stopCmd(),
                rampUp(),

                // Path 2: Travel to opposing side
                follow(robot.follower, pp.path("travel to opposing side")),

                // Path 3: Line up to garden
                follow(robot.follower, pp.path("line up to garden")),

                // Path 4: Collect garden
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("collect garden"))
                ),
                robot.intake.stopCmd(),

                // Path 5: Shoot
                // autoShoot().withTimeout(4000),
                follow(robot.follower, pp.path("shoot")),

                // Path 6: Collect second flower
                rampDown(),
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("collect second flower"))
                ),
                robot.intake.stopCmd(),
                rampUp(),

                // Path 7: Shoot to tip
                // autoShoot().withTimeout(4000),
                follow(robot.follower, pp.path("shoot to tip")),

                // Path 8: Traverse to opposing side
                follow(robot.follower, pp.path("traverse to opposing side")),

                // Path 9: Park line up
                follow(robot.follower, pp.path("park line up")),

                // Path 10: Park
                follow(robot.follower, pp.path("park"))
        );
    }
}
