package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;

/**
 * Distance-sensor subsystem for the transfer/hopper.
 *
 * <p>Currently reads the upper REV 2m Time-of-Flight sensor ({@code uppersense})
 * and publishes its distance and timeout status to the Panels dashboard when
 * {@link RobotMain#DASHBOARD} is enabled.
 */
public class TransferSensors extends SubsystemBase {
    private final Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    private final Rev2mDistanceSensor upperSensor;

    public TransferSensors(OpMode opMode) {
        upperSensor = opMode.hardwareMap.get(
                Rev2mDistanceSensor.class, IronConstants.UpperTransferSensorName);
    }

    /** Returns the upper sensor reading in millimeters. */
    public double getUpperDistanceMm() {
        return upperSensor.getDistance(DistanceUnit.MM);
    }

    /** Returns the upper sensor reading in inches. */
    public double getUpperDistanceInches() {
        return upperSensor.getDistance(DistanceUnit.INCH);
    }

    @Override
    public void periodic() {
        if (RobotMain.DASHBOARD) {
            // Read once in mm and convert to inches to avoid a second I2C read.
            // NOTE: addData only, no update() here. RobotMain.flushTelemetry() does the single flush.
            double mm = getUpperDistanceMm();
            double inches = mm / 25.4;
            telemetry.addData("Upper sensor (mm)", "%.1f", mm);
            telemetry.addData("Upper sensor (in)", "%.2f", inches);
            telemetry.addData("Upper sensor timeout", upperSensor.didTimeoutOccur());
        }
    }
}
