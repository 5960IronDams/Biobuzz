package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;
import android.util.Log;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.CommandFactory;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.turret.TurretPivot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Ramp;
import org.firstinspires.ftc.teamcode.Subsystems.ShootGateServo;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.TransferSensors;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PanelsFieldRenderer;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionFusion;
import org.firstinspires.ftc.teamcode.WattageLib.PanelsTelemLogging.TelemetryFileLogger;

import com.bylazar.configurables.annotations.Configurable;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Configurable
public class RobotMain {
    public static ALLIANCE_COLOR CurrentAlliance = ALLIANCE_COLOR.ALLIANCE_RED;

    /** Master switch for telemetry file logging (true = write CSV/log per loop). */
    public static boolean TELEMETRY_IO = false;
    /** Master switch for Panels dashboard drawing and telemetry (false = lean match loop, DS only). */
    public static boolean DASHBOARD = false;

    /** Panels telemetry output (web UI). Driver Station telemetry uses opmode.telemetry. */
    public static Telemetry DashTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    public final PanelsFieldRenderer fieldRenderer = new PanelsFieldRenderer();

    // Subsystems
    public Follower follower;
    public Intake intake;
    public Transfer transfer;
    public TransferSensors transferSensors;
    public Flywheel flywheel;
    public ShootGateServo PosServ;
    /** Front ramp (null if not in RC config). */
    @Nullable
    public Ramp ramp;
    /** Turret rotation (null if not in RC config). */
    public TurretPivot turret;
    /** Adjustable launcher hood (null if not in RC config). */
    @Nullable
    public Hood hood;
    /** Limelight 3A (null if not in RC config). */
    @Nullable
    public Vision vision;
    /** Feeds Limelight solves into the Pedro FusionLocalizer. */
    public VisionFusion visionFusion;
    public TelemetryFileLogger telemetryLogger;
    public CommandFactory CommandF;
    public static Pose autonomousEndPose = new Pose(0, 0, 0);
    private List<LynxModule> allHubs = new ArrayList<>();
    private final OpMode opmode;
    /** True once RobotRunPeriodic() has run (indicates PLAY was pressed). */
    private boolean runPeriodicSeen = false;
    public RobotMain(OpMode opmode)
    {
        CommandScheduler.getInstance().reset();
        this.opmode = opmode;
        HardwareHealth.get().reset();

        // Batch REV Hub reads into one transaction per loop for faster loop times.
        try {
            allHubs = opmode.hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        } catch (Exception ignored) {
        }

        follower = Constants.create(opmode.hardwareMap);
        SetupLimelightVision(opmode);
        SetupStartingPositionFromPinPointOrAuton();
        if (DASHBOARD) {
            drawField();
        }

        intake = new Intake(opmode);
        transfer = new Transfer(opmode);
        transferSensors = new TransferSensors(opmode);
        flywheel = new Flywheel(opmode);
        PosServ = new ShootGateServo(opmode);
        ramp = Ramp.tryCreate(opmode);
        if (ramp != null) {
            HardwareHealth.get().expect(Ramp.HEALTH_NAME);
        }
        turret = TurretPivot.tryCreate(opmode);
        if (turret != null) {
            HardwareHealth.get().expect("TURRET");
        }
        hood = Hood.tryCreate(opmode);

        CommandF = new CommandFactory(follower, intake, transfer, flywheel, PosServ, turret, hood, opmode.hardwareMap);

        if (TELEMETRY_IO) {
            telemetryLogger = TelemetryFileLogger.create(
                    opmode.hardwareMap.appContext, opmode.getClass().getSimpleName());
            if (telemetryLogger != null) {
                telemetryLogger.log("Event", "RobotMain initialized");
            }
        }

        opmode.telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        opmode.telemetry.update();
        if (DASHBOARD) {
            RobotMain.DashTelemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
            RobotMain.DashTelemetry.update();
        }
    }

    private void SetupLimelightVision(OpMode opmode) {
        vision = Vision.tryCreate(opmode.hardwareMap);
        Log.i("IronLog", "Vision Loaded");
        if (vision != null) vision.start(0);
        Log.i("IronLog", "Vision Started");
        visionFusion = new VisionFusion(follower, vision);
        Log.i("IronLog", "VisionFusion Started");
        if (vision != null) {
            HardwareHealth.get().expect("LIMELIGHT");
        } else if (IronConstants.CompBot) {
            HardwareHealth.get().report("LIMELIGHT", "absent at init (not in RC config)");
        }
    }

    /** Seeds the follower pose from the Pinpoint's retained pose (falls back to autonomousEndPose). */
    private void SetupStartingPositionFromPinPointOrAuton() {
        try {
            com.pedropathing.localization.Localizer dr =
                    visionFusion != null ? visionFusion.getDeadReckoning() : null;
            if (dr != null) {
                Pose p = dr.pose();
                if (p != null && (p.x() != 0 || p.y() != 0)) {
                    follower.setPose(p);
                    Log.i("IronLog", "Follower pose seeded from Pinpoint: " + p);
                }
                return;
            }
        } catch (Exception e) {
            Log.w("IronLog", "Pinpoint pose seed failed; falling back to autonomousEndPose", e);
        }
        try {
            if (autonomousEndPose != null) follower.setPose(autonomousEndPose);
        } catch (Exception ignored) {
        }
    }

    /** Re-anchors the raw Pinpoint odometry device to the given pose. */
    public void writePoseToPinpoint(Pose pose) {
        if (pose == null) {
            return;
        }
        com.pedropathing.localization.Localizer dr =
                visionFusion != null ? visionFusion.getDeadReckoning() : null;
        if (dr == null) {
            Log.w("IronLog", "Pinpoint pose write skipped: no dead-reckoning localizer");
            return;
        }
        try {
            dr.setPose(pose);
            Log.i("IronLog", "Pinpoint re-anchored to: " + pose);
        } catch (Exception e) {
            Log.w("IronLog", "Pinpoint pose write failed", e);
        }
    }

    /** Saves the current fused pose into Pinpoint + autonomousEndPose if within field bounds (call in real OpMode stop()). */
    public void saveFusedPoseToPinpoint() {
        try {
            Pose fused = follower.pose();
            if (fused == null) {
                return;
            }
            if (fused.x() < -144.0 || fused.x() > 288.0
                    || fused.y() < -144.0 || fused.y() > 288.0) {
                Log.w("IronLog", "Fused pose outside field bounds; Pinpoint NOT re-anchored: " + fused);
                return;
            }
            writePoseToPinpoint(fused);
            autonomousEndPose = fused;
        } catch (Exception ignored) {
        }
    }

    /** Main per-loop update: clears bulk cache, updates odometry/vision, runs commands, updates readiness, and measures loop time. */
    public void RobotRunPeriodic()
    {
        for (int i = 0; i < allHubs.size(); i++) {
            allHubs.get(i).clearBulkCache();
        }
        runPeriodicSeen = true;
        follower.update();
        if (visionFusion != null) visionFusion.correct();
        if (vision != null) vision.healthPing();
        if (CommandF != null) CommandF.updateMotionAndTimers();
        CommandScheduler.getInstance().run();
        if (CommandF != null) CommandF.readinessPeriodic();
        looptime();
    }

    /** Init-loop update: refreshes odometry and Limelight vision without running commands or motors. */
    public void InitRunPeriodic()
    {
        for (int i = 0; i < allHubs.size(); i++) {
            allHubs.get(i).clearBulkCache();
        }
        follower.update();
        if (visionFusion != null) visionFusion.correct(true);
        if (vision != null) vision.healthPing();
    }

    /** Draws the fused pose, turret aim ray, and raw MT1/MT2 vision poses on the Panels Field widget. */
    public void drawField() {
        Pose fused = null;
        Pose mt1 = null;
        Pose mt2 = null;
        try {
            if (follower != null) fused = follower.pose();
        } catch (Exception ignored) {
        }
        try {
            if (visionFusion != null) {
                mt1 = visionFusion.lastMt1Pedro();
                mt2 = visionFusion.lastMt2Pedro();
            }
        } catch (Exception ignored) {
        }
        try {
            if (turret != null) {
                fieldRenderer.drawTurretRay(fused, turret.getPosition());
            }
        } catch (Exception ignored) {
        }
        try {
            if (CommandF != null && CommandF.aim != null) {
                Pose eff = CommandF.aim.activeEffectiveTarget != null
                        ? CommandF.aim.activeEffectiveTarget
                        : CommandF.aim.liveEffectiveTarget();
                fieldRenderer.drawAimTargetDot(eff);
            }
        } catch (Exception ignored) {
        }
        try {
            fieldRenderer.drawFusedAndVision(fused, mt1, mt2);
        } catch (Exception ignored) {
        }
    }

    /** Evaluates hardware health and flushes Panels telemetry once at the end of each loop tick. */
    public void flushTelemetry() {
        HardwareHealth.get().setLoopStats(lastLoopTimeMs);
        HardwareHealth.get().evaluate(runPeriodicSeen);

        if (!DASHBOARD) {
            if (telemetryLogger != null && TELEMETRY_IO) {
                telemetryLogger.log("Loop", "dashboard off");
            }
            return;
        }
        drawField();
        if (visionFusion != null) visionFusion.report();
        reportTurret();
        DashTelemetry.addData("HW health", HardwareHealth.get().summary());
        if (telemetryLogger != null && TELEMETRY_IO) {
            telemetryLogger.captureSnapshot(DashTelemetry);
        }
        DashTelemetry.update();
    }

    /** Publishes robot-relative and field-relative turret headings to Panels telemetry. */
    public void reportTurret() {
        if (turret == null) {
            return;
        }
        try {
            double robotRelDeg = turret.getPosition();
            DashTelemetry.addData("Turret heading (robot)",
                    String.format(Locale.US, "%+.1f deg", robotRelDeg));
            double fieldRelDeg = robotRelDeg;
            try {
                Pose p = follower.pose();
                if (p != null) {
                    fieldRelDeg = normDeg180(Math.toDegrees(p.heading()) + robotRelDeg);
                }
            } catch (Exception ignored) {
                // no follower pose yet — robot-relative value still reported
            }
            DashTelemetry.addData("Turret heading (field)",
                    String.format(Locale.US, "%+.1f deg", fieldRelDeg));
        } catch (Exception ignored) {
        }
    }

    /** Normalize degrees to [-180, 180). */
    private static double normDeg180(double deg) {
        double r = (deg + 180.0) % 360.0;
        if (r < 0) r += 360.0;
        return r - 180.0;
    }

    /**
     * Finalize the telemetry log file. Call once from {@code OpMode.stop()}.
     * Safe to call multiple times or when logging failed to init (null-safe).
     */
    public void shutdown() {
        if (telemetryLogger != null) {
            try {
                telemetryLogger.log("Event", "OpMode stopped");
            } catch (Exception ignored) {
            }
            try {
                telemetryLogger.close();
            } catch (Exception ignored) {
            }
            telemetryLogger = null;
        }
    }
    long lastTime = System.nanoTime();
    /** Measured loop period (ms) from the LAST looptime() — 0 until the first tick. */
    private double lastLoopTimeMs = 0.0;

    /** Measures loop period (ms) and queues Panels loop-time telemetry. */
    public void looptime() {
        long currentTime = System.nanoTime();
        double loopTimeMs = (currentTime - lastTime) / 1_000_000.0;
        lastTime = currentTime;
        lastLoopTimeMs = loopTimeMs;

        if (DASHBOARD) {
            RobotMain.DashTelemetry.addData("Loop Time (ms)", "%.2f ms", loopTimeMs);
            RobotMain.DashTelemetry.addData("Hz", "%.1f Hz", 1000.0 / loopTimeMs);
        }
    }

    /** Last measured loop period in ms (0.0 before the first tick). */
    public double getLoopTimeMs() {
        return lastLoopTimeMs;
    }
}
