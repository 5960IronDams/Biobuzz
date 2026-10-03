package org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers;

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
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision;

import java.lang.reflect.Field;

/**
 * Fusion tuning OpMode: drive around (sticks) while Pinpoint predicts and
 * Limelight corrects, with full Kalman telemetry on Panels.
 *
 * <p>Use it to:
 * <ol>
 *   <li>Verify the {@code Vision} frame map: park at field center facing
 *       up-field (+Y) with 2+ tags visible — {@code Fusion/visionX/Y} must read
 *       ~(72, 72) Pedro and {@code Fusion/visionHdeg} ~90. If not, adjust the
 *       LL_* tunables (Panels -> Vision) live until it does.</li>
 *   <li>Tune trust: watch {@code Fusion/scale} and {@code Fusion/residualIn}.
 *       Scale pinned at 1.0x everywhere = over-trusting vision (raise
 *       {@code BASE_R_XY}); residuals never shrinking = under-trusting
 *       (lower {@code BASE_R_XY}); jumps on tag appearance = lower
 *       {@code RESIDUAL_MAX_PENALTY} or raise {@code BASE_R_XY}.</li>
 *   <li>Toggle {@code VisionFusion.ENABLED} live to A/B Pinpoint-only vs fused
 *       on the same drive.</li>
 * </ol>
 *
 * <p>Gamepad1 start teleports to (24, 24, 90deg Pedro) like a match start for
 * repeatable runs. The Panels Field widget draws the fused pose (body) — the
 * raw vision pose is telemetry-only so a bad frame map can't confuse the
 * drawing.
 */
@Disabled
@TeleOp(name = "Fusion Tune", group = "Tuning")
public class FusionTune extends OpMode {

    private RobotMain robot;
    private boolean prevStart = false;

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
        boolean pressed = gamepad1.start;
        if (pressed && !prevStart) {
            robot.follower.setPose(new Pose(24, 24, Math.toRadians(90)));
        }
        prevStart = pressed;

        // Manual drive on the FUSED pose (same as match TeleOp).
        DrivePowers powers =
                ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x,
                        robot.follower.pose().heading());
        robot.follower.manual(powers);
        robot.follower.update();

        // Correct AFTER the trailing update (same order as RobotMain).
        // Same-frame re-poll no-ops inside VisionFusion; a new LL frame still fuses.
        if (robot.visionFusion != null) robot.visionFusion.correct();

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

        // No field draw here — flushTelemetry() paints once per loop (fused robot
        // + raw MT1 yellow / MT2 green): park level with 2+ tags visible and all
        // three must agree (frame-map check from the class javadoc, now visual).
        robot.flushTelemetry();
    }
}
