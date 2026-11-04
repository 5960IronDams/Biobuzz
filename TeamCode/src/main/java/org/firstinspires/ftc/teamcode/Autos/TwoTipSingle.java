package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFollow.follow;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

public class TwoTipSingle extends AutoOpModeBase {

    public TwoTipSingle() {
        super.assetPath = "pathfiles/two_tip single.pp.json";
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;
    }

    @Override
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                // Path 0: Shoot 1
                autoShoot().withTimeout(4000),
                follow(robot.follower, pp.path("shoot 1")),

                // Path 1: Collect team's balls
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("collect team's balls"))
                ),
                robot.intake.stopCmd(),

                // Path 2: Line up for flower
                follow(robot.follower, pp.path("line up for flower")),
                autoShoot().withTimeout(4000),
                // Path 3: Collect flower / shoot
                rampDown(),
                new ParallelCommandGroup(
                        robot.intake.runCmd(),
                        follow(robot.follower, pp.path("collect flower / shoot"))
                ),

                // Path 4: Line up for park
                new ParallelCommandGroup(
                        robot.intake.stopCmd(),
                        rampUp(),
                        follow(robot.follower, pp.path("line up for park"))
                ),


                // Path 5: Park
                follow(robot.follower, pp.path("park"))
        );
    }
}
