package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.utils.TelemetryLoggerStatus;

import java.util.Locale;

/**
 * Example Autonomous showing how to use the telemetry logging system.
 *
 * This demonstrates:
 * 1. Automatic logging (already enabled in RobotMain)
 * 2. Manual logging specific events
 * 3. Checking logger status
 *
 * <p>Note: This is a {@link LinearOpMode} example (uses runOpMode /
 * waitForStart / opModeIsActive). The project {@code AutoOpMode} base class
 * is an iterative {@code OpMode} with an abstract {@code autoRoutine()}
 * method, so it cannot use this linear style.
 */
@Autonomous(name = "Example: Telemetry Logging Demo", group = "Examples")
public class ExampleTelemetryLogging extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Standard initialization — creates subsystems + telemetry logger.
        RobotMain robot = new RobotMain(this);

        // Show logging status in Panels
        new TelemetryLoggerStatus(robot).report();
        robot.flushTelemetry();

        // Wait for start
        waitForStart();

        // Log a specific event at the start
        if (robot.telemetryLogger != null) {
            robot.telemetryLogger.log("AUTO_EVENT", "Match started");
        }

        // Autonomous code here
        while (opModeIsActive()) {
            robot.RobotRunPeriodic();

            // Add some debug telemetry
            RobotMain.DashTelemetry.addData("Status", "Running");
            RobotMain.DashTelemetry.addData("Time", getRuntime());

            // Manually log important milestones
            if (getRuntime() > 5.0 && getRuntime() < 5.1
                    && robot.telemetryLogger != null) {
                robot.telemetryLogger.log("MILESTONE", "Reached 5 seconds");
            }

            // Flush (automatically captures telemetry snapshot)
            robot.flushTelemetry();
        }

        // Log final state (shutdown() logs the stop event, closes the file,
        // and nulls the logger — safe even if init failed).
        robot.shutdown();
    }
}

/**
 * Example showing how to conditionally enable/disable logging.
 */
class ConditionalLoggingExample {
    public static void enableDebugLogging(RobotMain robot) {
        if (robot.telemetryLogger != null) {
            robot.telemetryLogger.setEnabled(true);
            robot.telemetryLogger.log("DEBUG", "Logging enabled");
        }
    }

    public static void disableDebugLogging(RobotMain robot) {
        if (robot.telemetryLogger != null) {
            robot.telemetryLogger.log("DEBUG", "Logging disabled");
            robot.telemetryLogger.setEnabled(false);
        }
    }
}

/**
 * Example showing fine-grained logging of subsystem state.
 */
class SubsystemDebugLoggingExample {
    public static void logVisionState(RobotMain robot, String status, double x, double y, double heading) {
        if (robot.telemetryLogger != null) {
            robot.telemetryLogger.log("Vision/Status", status);
            robot.telemetryLogger.log("Vision/Pose.X", String.format(Locale.US, "%.2f", x));
            robot.telemetryLogger.log("Vision/Pose.Y", String.format(Locale.US, "%.2f", y));
            robot.telemetryLogger.log("Vision/Heading", String.format(Locale.US, "%.2f", heading));
        }
    }

    public static void logMotorState(RobotMain robot, String motorName, double velocity, double current) {
        if (robot.telemetryLogger != null) {
            robot.telemetryLogger.log("Motor/" + motorName + "/Velocity", String.format(Locale.US, "%.2f", velocity));
            robot.telemetryLogger.log("Motor/" + motorName + "/Current", String.format(Locale.US, "%.2f", current));
        }
    }
}

/**
 * Example showing error/warning logging for debugging issues.
 */
class ErrorLoggingExample {
    public static void logError(RobotMain robot, String component, String errorMsg, Exception e) {
        if (robot.telemetryLogger != null) {
            robot.telemetryLogger.log("ERROR_" + component, errorMsg);
            if (e != null) {
                robot.telemetryLogger.log("ERROR_EXCEPTION", e.getClass().getSimpleName() + ": " + e.getMessage());
            }
        }
    }

    public static void logWarning(RobotMain robot, String component, String warningMsg) {
        if (robot.telemetryLogger != null) {
            robot.telemetryLogger.log("WARN_" + component, warningMsg);
        }
    }
}
