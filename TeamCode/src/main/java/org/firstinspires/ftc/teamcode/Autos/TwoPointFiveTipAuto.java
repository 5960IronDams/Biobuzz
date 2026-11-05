package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFollow.follow;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

public class TwoPointFiveTipAuto extends AutoOpModeBase {

    public TwoPointFiveTipAuto() {
        super.assetPath = "pathfiles/two_point_five_tip auto.pp.json";
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;
    }

    @Override
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                autoShoot().withTimeout(4000),
                // Path 0: Collect from garden
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("collect from garden"))
                ),
                robot.intake.stopCmd(),

                // Path 1: Line up to flower
                follow(robot.follower, pp.path("line up to flower")),

                // Path 2: Collect from flower / shoot
                rampDown(),
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("collect from flower / shoot"))
                ),
                robot.intake.stopCmd(),
                rampUp(),

                // Path 3: Line up to flower (second flower - path index 3 avoids duplicate name lookup)
                follow(robot.follower, pp.path(3)),

                // Path 4: Collect flower
                rampDown(),
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("collect flower"))
                ),
                robot.intake.stopCmd(),
                rampUp(),

                // Path 5: Shoot
                // autoShoot().withTimeout(4000),
                follow(robot.follower, pp.path("shoot")),

                // Path 6: Get out the way
                follow(robot.follower, pp.path("get out the way")),

                // Path 7: Park ("Screw u guys I'm going home" - Cartman)
                follow(robot.follower, pp.path(7))
        );
    }
}
