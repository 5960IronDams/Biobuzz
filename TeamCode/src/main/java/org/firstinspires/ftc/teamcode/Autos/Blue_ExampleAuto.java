package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.clearAll;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GameConst;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

@Autonomous(name = "Blue_ExampleAuto1", group = "Autos")
public class Blue_ExampleAuto extends Red_ExampleAuto {//change both classes if you created this from template!
    public Blue_ExampleAuto()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_BLUE;//this must be set to alliance this auton is made for
    }
}