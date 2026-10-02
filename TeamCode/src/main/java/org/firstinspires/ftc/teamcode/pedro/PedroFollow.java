package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * SolversLib adapter for Pedro Pathing path following (replaces Ivy's
 * {@code PedroCommands.follow}).
 *
 * <p>{@code initialize()} hands the path to the follower; execution is empty
 * because {@code RobotMain.RobotRunPeriodic()} already calls
 * {@code follower.update()} every loop (as does TeleOp's trailing update) —
 * starting a second update loop here would double-step the follower.
 * {@code isFinished()} reports {@code !follower.isBusy()}. On interrupt the
 * follower is stopped so drivetrain powers are released.
 */
public final class PedroFollow {

    private PedroFollow() {}

    /** Follow the given path until the follower is no longer busy. */
    public static Command follow(Follower follower, Path path) {
        if (follower == null || path == null) {
            throw new IllegalArgumentException("PedroFollow.follow needs a non-null Follower and Path");
        }
        // SolversLib requirements must be Subsystem instances; the Follower is
        // wrapped in an anonymous adapter so only one follow command can own it.
        Subsystem requirement = new SubsystemBase() {};
        return new FunctionalCommand(
                () -> follower.follow(path),     // initialize: hand the path to the follower
                () -> {},                        // execute: RobotRunPeriodic() calls follower.update()
                (interrupted) -> {               // end: release the drivetrain if cancelled
                    if (interrupted) {
                        follower.stop();
                    }
                },
                () -> !follower.isBusy(),        // isFinished: path complete
                requirement);
    }
}
