package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

@Autonomous(name = "Blue_ExampleAuto1", group = "Autos")
public class Blue_ExampleAuto extends Red_ExampleAuto {//change both classes if you created this from template!
    public Blue_ExampleAuto()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_BLUE;//this must be set to alliance this auton is made for
    }
}