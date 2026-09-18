package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;

@TeleOp(name = "Blue-JowByTeleOp", group = "TeleOp")
@Disabled
public class BlueTeleOp extends JowByTeleOp{
    public BlueTeleOp()
    {
        RobotMain.CurrentAlliance = ALLIANCE_COLOR.ALLIANCE_BLUE;
    }
}
