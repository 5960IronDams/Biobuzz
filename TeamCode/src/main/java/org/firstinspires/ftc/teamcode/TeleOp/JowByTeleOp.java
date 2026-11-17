package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareHealth;
import org.firstinspires.ftc.teamcode.RobotMain;

@TeleOp(name = "JowBy_TeleOp", group = "TeleOp")//red_

public class JowByTeleOp extends OpMode {

    public RobotMain robot;
    public KeyBindings keys;

    @Override
    public void init() {
        robot = new RobotMain(this);
        keys = new KeyBindings(this, robot);
        robot.flushTelemetry();
    }

    @Override
    public void init_loop() {
        robot.InitRunPeriodic();
        UpdateTelemetry();
        robot.flushTelemetry();
    }

    @Override
    public void start() {
        // Re-solve turret angle at PLAY in case the turret was moved during init.
        if (robot != null && robot.turret != null) {
            robot.turret.rezeroAtStart();
        }
    }

    @Override
    public void loop() {
        keys.update();
        robot.RobotRunPeriodic();
        UpdateTelemetry();
        robot.flushTelemetry();
    }

    @Override
    public void stop() {
        if (keys != null) {
            keys.stop();
        }
        if (robot != null) {
            robot.saveFusedPoseToPinpoint();
            robot.shutdown();
        }
    }


    private long lastDsTelemetryMs = 0;

    public void UpdateTelemetry()
    {
        long now = System.currentTimeMillis();
        // DS screen: rate-limit to ~20 Hz (every 50 ms) so fast loops (70-100 Hz) don't congest Wi-Fi
        if (now - lastDsTelemetryMs >= 50) {
            telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
            telemetry.addData("HW", HardwareHealth.get().dsLine());

            // ---- DRIVER DASHBOARD: SHOOT READINESS & FLYWHEEL STATUS ----
            if (robot != null && robot.CommandF != null) {
                telemetry.addData("SHOOT", robot.CommandF.getShootReadinessSummary());
            }
            if (robot != null && robot.flywheel != null) {
                telemetry.addData("FLYWHEEL", robot.flywheel.getStatusLine());
            }

            telemetry.update();
            lastDsTelemetryMs = now;
        }

        // addData only — flushed once via flushTelemetry() at end of loop().
        if (keys != null) {
            keys.reportTelemetry();
        }
        if (RobotMain.DASHBOARD) {
            RobotMain.DashTelemetry.addData("Status", "Running");
        }
    }
}
