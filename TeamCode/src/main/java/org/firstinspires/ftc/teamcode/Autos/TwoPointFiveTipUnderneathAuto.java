package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFollow.follow;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

public class TwoPointFiveTipUnderneathAuto extends AutoOpModeBase {

    public TwoPointFiveTipUnderneathAuto() {
        super.assetPath = "pathfiles/two_point_five_tip underneath auto.pp.json";
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

                // Path 1: Garden to flower transition
                follow(robot.follower, pp.path("garden to flower transition")),

                // Path 2: Second transition
                follow(robot.follower, pp.path("second transition")),

                // Path 3: Collect from flower / shoot
                rampDown(),
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("collect from flower / shoot"))
                ),
                robot.intake.stopCmd(),
                rampUp(),

                // Path 4: Line up to flower
                follow(robot.follower, pp.path("line up to flower")),

                // Path 5: Transition from line up to collection
                follow(robot.follower, pp.path("transition from line up to collection")),

                // Path 6: Collect flower
                rampDown(),
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("collect flower"))
                ),
                robot.intake.stopCmd(),
                rampUp(),

                // Path 7: Shoot
                // autoShoot().withTimeout(4000),
                follow(robot.follower, pp.path("shoot")),

                // Path 8: Get out the way
                follow(robot.follower, pp.path("get out the way")),

                // Path 9: Park ("Screw u guys I'm going home" - Cartman)
                follow(robot.follower, pp.path(9))
        );
    }
}
