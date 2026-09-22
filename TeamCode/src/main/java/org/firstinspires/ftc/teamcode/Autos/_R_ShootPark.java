package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.killerwatts.lib.AutoOpMode;

//@Autonomous(name = "Red_R_Shoot_Park",group = "Autos")
public class _R_ShootPark extends AutoOpMode {

    public _R_ShootPark()//constructor argument creates object, it should have same name as the class it constructs (the class this file is named after)
    {
        super.assetPath = "pathfiles/Red_R_Shoot_Park.pp";//this must be set to alliance this auton is made for
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;//this must be set to alliance this auton is made for
    }
    @Override
    public Command autoRoutine() {
        return sequential(
                //shoot
                robot.CommandF.ServoToPos(0.5),
                robot.CommandF.RunIntake(),
                //goto park
                follow(robot.follower, pp.getPathByLineName("ParkPlace"))
        );
    }
}