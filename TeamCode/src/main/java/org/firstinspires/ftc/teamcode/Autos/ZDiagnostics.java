package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFollow.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

@Autonomous(name = "ZDiagnostics",group = "Autos")
public class ZDiagnostics extends AutoOpModeBase {

    public ZDiagnostics()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/exampleAuto1.pp.json";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                //fire or whatever.
                autoShoot().withTimeout(4000),
                new WaitCommand(1500),
                rampDown(),
                new WaitCommand(1500),
                rampUp(),
                robot.intake.runCmd(),
                new WaitCommand(1500),
                robot.intake.stopCmd(),
                new WaitCommand(1500)
        );
    }
}
