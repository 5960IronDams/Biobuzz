package org.firstinspires.ftc.teamcode.irondams.killerwatts;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.irondams.killerwatts.lib.SubsystemBase;

public class ServoMove extends SubsystemBase {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    public int loop = 0;
    public ServoMove()
    {

    }
    @Override
    public void Periodic() {
        telemetry = dashboard.getTelemetry();
        telemetry.addData("ServoMove Loop", loop);
        loop++;
        telemetry.update();
    }
}
