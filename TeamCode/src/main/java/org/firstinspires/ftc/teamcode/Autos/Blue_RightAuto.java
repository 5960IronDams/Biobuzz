package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

@Autonomous(name = "Blue_RightAuto",group = "Autos")
public class Blue_RightAuto extends Red_RightAuto {

    public Blue_RightAuto()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_BLUE;//this must be set to alliance this auton is made for
    }
}