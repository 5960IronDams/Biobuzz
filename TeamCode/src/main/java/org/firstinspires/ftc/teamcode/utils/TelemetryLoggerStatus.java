package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Utility for verifying telemetry logging is working correctly.
 * 
 * Add to your OpMode's telemetry output to verify logging is active:
 * {@code new TelemetryLoggerStatus(robot).report();}
 */
public class TelemetryLoggerStatus {
    private static final String TAG = "TelemetryStatus";
    private final org.firstinspires.ftc.teamcode.RobotMain robot;
    
    public TelemetryLoggerStatus(org.firstinspires.ftc.teamcode.RobotMain robot) {
        this.robot = robot;
    }
    
    /**
     * Reports logger status to Panels telemetry.
     */
    public void report() {
        if (robot == null) return;
        
        if (robot.telemetryLogger != null) {
            org.firstinspires.ftc.teamcode.RobotMain.DashTelemetry.addData(
                    "Logging Status", "ACTIVE");
            org.firstinspires.ftc.teamcode.RobotMain.DashTelemetry.addData(
                    "Log File", robot.telemetryLogger.getLogFilePath());
            Log.i(TAG, "Telemetry logging active: " + robot.telemetryLogger.getLogFilePath());
        } else {
            org.firstinspires.ftc.teamcode.RobotMain.DashTelemetry.addData(
                    "Logging Status", "DISABLED (null)");
            Log.w(TAG, "Telemetry logger not initialized");
        }
    }
}
