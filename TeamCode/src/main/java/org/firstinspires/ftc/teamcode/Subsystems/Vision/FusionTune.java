package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.WattageLib.lib.CommandGamepad;
import org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFieldBridge;

import java.lang.reflect.Field;

/**
 * Vision fusion tuning OpMode: drive with sticks while Pinpoint predicts and
 * Limelight corrects, displaying {@code Fusion/*} telemetry in Panels.
 * Press Start on gamepad1 to reset pose to (24, 24, 90 deg) for repeatable runs.
 */
@Disabled
@TeleOp(name = "Fusion Tune", group = "Tuning")
public class FusionTune extends OpMode {

    private RobotMain robot;
    private boolean prevStart = false;
    /** Fluent bindings facade over gamepad1 — see bindCommands(). */
    private CommandGamepad DriveController;
    public FusionTune()
    {
        DriveController = new CommandGamepad(gamepad1);
    }
    @Override
    public void init() {
        robot = new RobotMain(this);

        RobotMain.DashTelemetry.addData("FusionTune", "drive with sticks; start = teleport (24,24,90deg)");
        robot.flushTelemetry();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.shutdown();
        }
    }

    @Override
    public void loop() {
        robot.RobotRunPeriodic();
        if (robot.follower == null) {
            RobotMain.DashTelemetry.addData("FusionTune", "no follower");
            robot.flushTelemetry();
            return;
        }

        // Teleport for repeatable runs (edge-triggered).
        boolean pressed = DriveController.start().get();
        if (pressed && !prevStart) {
            robot.follower.setPose(new Pose(24, 24, Math.toRadians(90)));
        }
        prevStart = pressed;

        // Manual drive on the FUSED pose (same as match TeleOp).
        DrivePowers powers =
                ManualDrive.fieldCentric(
                        -DriveController.getLeftY(), -DriveController.getLeftX(), -DriveController.getRightX(),
                        robot.follower.pose().heading());
        robot.follower.manual(powers);

        // Extra compare telemetry: raw Pinpoint vs fused vs vision.
        try {
            Localizer loc = robot.follower.localizer;
            if (loc instanceof FusionLocalizer) {
                Field f =
                        FusionLocalizer.class.getDeclaredField("deadReckoning");
                f.setAccessible(true);
                Localizer raw =
                        (Localizer) f.get(loc);
                if (raw != null && raw.pose() != null) {
                    RobotMain.DashTelemetry.addData("Fusion/rawX", "%.2f", raw.pose().x());
                    RobotMain.DashTelemetry.addData("Fusion/rawY", "%.2f", raw.pose().y());
                    RobotMain.DashTelemetry.addData("Fusion/rawHdeg", "%.1f",
                            Math.toDegrees(Angle.normalize(raw.pose().heading())));
                }
            }
        } catch (Exception ignored) {
        }
        if (robot.vision != null) {
            RobotMain.DashTelemetry.addData("Fusion/seedYawDeg", "%.1f",
                    Math.toDegrees(PedroFieldBridge.pedroHeadingToField(robot.follower.pose().heading())));
        } else {
            RobotMain.DashTelemetry.addData("Fusion/vision", Vision.class.getSimpleName() + " not configured");
        }

        robot.flushTelemetry();
    }
}
