package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class SharedObjects {
    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static Telemetry telemetry = dashboard.getTelemetry();
    //public static Follower follower = Constants.createFollower(hardwareMap);
}
