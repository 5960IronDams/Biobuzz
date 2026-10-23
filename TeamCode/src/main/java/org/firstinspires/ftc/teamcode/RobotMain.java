package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;
import android.util.Log;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.CommandFactory;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.turret.TurretPivot;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.HiveCellMonitor;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ShootGateServo;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PanelsFieldRenderer;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionFusion;
import org.firstinspires.ftc.teamcode.WattageLib.PanelsTelemLogging.TelemetryFileLogger;

import com.bylazar.configurables.annotations.Configurable;

import java.lang.reflect.Field;
import java.util.Locale;

@Configurable
public class RobotMain {
    public static ALLIANCE_COLOR CurrentAlliance = ALLIANCE_COLOR.ALLIANCE_RED;

    // ---- Loop-time I/O switches (Panels -> RobotMain). All ON = full debug
    // richness (~30-50ms loops); turn OFF for matches / fast loops. ----
    /** Master switch for the telemetry FILE logger (disk write+flush per tick). */
    public static boolean TELEMETRY_IO = true;
    /**
     * Master switch for the Panels dashboard: field canvas draw, Fusion/*
     * report lines, captureSnapshot, and the Panels update() send. False =
     * lean loop (DS telemetry only). Panels-tunable but takes effect from the
     * NEXT flushTelemetry() call.
     */
    public static boolean DASHBOARD = true;

    /** Panels telemetry (Panels web UI). DS telemetry stays on opmode.telemetry. */
    public static Telemetry DashTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    public final PanelsFieldRenderer fieldRenderer = new PanelsFieldRenderer();
    //subsystems
    public Follower follower;// = SharedObjects.follower;
    public Intake intake;
    public Transfer transfer;
    public Flywheel flywheel;
    public ShootGateServo PosServ;
    /** Turret rotation (null when the turret hardware isn't in the RC config). */
    public TurretPivot turret;
    /** Null when no Limelight3A is in the RC config (Pinpoint-only mode). */
    @Nullable
    public Vision vision;
    /** Kalman corrector feeding Limelight solves into the Fusion localizer. */
    public VisionFusion visionFusion;
    /** Relative hive-cell monitor (cluster aiming only, never the global pose). */
    public HiveCellMonitor cells;
    // Telemetry file logger for debugging
    public TelemetryFileLogger telemetryLogger;
    //
    public CommandFactory CommandF;
    public static Pose autonomousEndPose = new Pose(0, 0, 0);
    public RobotMain(OpMode opmode)
    {
            CommandScheduler.getInstance().reset(); //clears all scheduled commands + subsystem registrations after opmode switch.

        // REV Hub bulk-read caching: batch all encoder/digital I2C reads on each
        // hub into ONE transaction per loop instead of one-per-read. AUTO mode
        // refreshes the cache automatically on first read each loop. Cuts
        // multi-device loop latency (same win Pedro's tuners already use).
        // Wrapped: skip silently on non-REV test rigs with no Lynx module.
        try {
            for (LynxModule hub : opmode.hardwareMap.getAll(LynxModule.class)) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        } catch (Exception ignored) {
        }

        //subsystems
        follower = Constants.create(opmode.hardwareMap);
        SetupStartingPositionFromPinPointOrAuton();
        SetupLimelightVision(opmode);
        // Relative cell monitor shares the same Limelight (no extra HW handle).
        cells = new HiveCellMonitor(vision);
        // First paint so the Field widget shows a pose even before init_loop
        // ticks (TeleOp never resets the pose). Steady-state drawing lives in
        // flushTelemetry() — exactly one batched draw per loop/init tick.
        drawField();
        intake = new Intake(opmode);
        transfer = new Transfer(opmode);
        flywheel = new Flywheel(opmode);
        PosServ = new ShootGateServo(opmode);
        // Optional subsystem: tryCreate logs + returns null instead of crashing
        // every OpMode when the turret isn't in the RC config (Vision pattern).
        turret = TurretPivot.tryCreate(opmode);


        //after all subsystems are started (i.e their variables point to an object). Build the command factory
        CommandF = new CommandFactory(follower,intake,transfer,flywheel,PosServ,turret, opmode.hardwareMap);

        // Initialize telemetry file logger (optional: comment out to disable logging).
        // Must pass appContext: the RC app UID can only write to its app-private
        // dirs — /data/local/tmp is shell-owned and gives Permission denied.
        // Tag with the OpMode name: every init opens a new timestamped file, so
        // the tag tells you which file belongs to which run.
        telemetryLogger = TelemetryFileLogger.create(
                opmode.hardwareMap.appContext, opmode.getClass().getSimpleName());
        if (telemetryLogger != null) {
            telemetryLogger.log("Event", "RobotMain initialized");
        }

        //declare what alliance we are on to BOTH telemetry outputs
        //this is for one good final double check for the driver and co pilot
        opmode.telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        opmode.telemetry.update();
        RobotMain.DashTelemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        RobotMain.DashTelemetry.update();
    }

    private void SetupLimelightVision(OpMode opmode) {
        // Limelight is optional: absent in tuning configs -> fused filter runs
        // Pinpoint-only (VisionFusion reports "no-limelight-configured").
        vision = Vision.tryCreate(opmode.hardwareMap);
        Log.i("IronLog","Vision Loaded");
        if (vision != null) vision.start(0);
        Log.i("IronLog","Vision Started");
        visionFusion = new VisionFusion(follower, vision);
        Log.i("IronLog","VisionFusion Started");
    }

    private void SetupStartingPositionFromPinPointOrAuton() {
        // Seed the filter from the Pinpoint's retained pose (the Pinpoint holds
        // the end-of-auto pose across OpModes). Without this, TeleOp starts at
        // (0,0,0) and the first update composes the full pose delta into the
        // Kalman history — that startup transient is the window vision fuses
        // get back-dated into, and re-propagating across it corrupts the
        // heading (the 90->255deg flips seen at 23:04/23:22). Also fixes the
        // init-screen (0,0) display.
        try {
            Field drField = FusionLocalizer.class.getDeclaredField("deadReckoning");
            drField.setAccessible(true);
            Object dr = drField.get(follower.localizer);
            if (dr instanceof Localizer) {
                Pose p = ((Localizer) dr).pose();
                if (p != null && (p.x() != 0 || p.y() != 0)) {
                    follower.setPose(p);
                    Log.i("IronLog", "Follower pose seeded from Pinpoint: " + p);
                }
            }
        } catch (Exception e) {
            Log.w("IronLog", "Pinpoint pose seed failed; falling back to autonomousEndPose", e);
            try {
                if (autonomousEndPose != null) follower.setPose(autonomousEndPose);
            } catch (Exception ignored) {
            }
        }
    }

    /**
     * Resolve the FusionLocalizer's raw dead-reckoning localizer (the Pinpoint)
     * by reflection — the same accessor {@link #SetupStartingPositionFromPinPointOrAuton()}
     * uses. Returns null when there is no follower/localizer, it is not a
     * FusionLocalizer, or the library's internal field changed; callers must
     * handle null.
     */
    private Localizer deadReckoning() {
        try {
            if (follower == null || !(follower.localizer instanceof FusionLocalizer)) {
                return null;
            }
            Field drField = FusionLocalizer.class.getDeclaredField("deadReckoning");
            drField.setAccessible(true);
            Object dr = drField.get(follower.localizer);
            return (dr instanceof Localizer) ? (Localizer) dr : null;
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Re-anchor the raw Pinpoint (dead-reckoning) to {@code pose} so the
     * device-side pose matches the fused belief. The Pinpoint keeps integrating
     * afterwards — it tracks pushes/coasting while disabled — and its retained
     * pose is exactly what {@link #SetupStartingPositionFromPinPointOrAuton()}
     * seeds the next OpMode from. Writing the fused pose here is what keeps a
     * TeleOp restart from warping back to the raw Pinpoint's drifted pose.
     *
     * <p>Safe mid-run (the Pinpoint's setPosition is designed for runtime
     * re-anchoring); a deliberate re-zero while moving may cause a one-tick
     * velocity transient.
     */
    public void writePoseToPinpoint(Pose pose) {
        if (pose == null) {
            return;
        }
        Localizer dr = deadReckoning();
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

    /**
     * Save the CURRENT FUSED pose into the Pinpoint, sanity-gated. Call from
     * every real-field OpMode's stop() (TeleOp + Auto) so the next OpMode seeds
     * from a vision-corrected pose instead of raw drifted odometry. Never call
     * from bench/tuning OpModes (FusionTune, tuners): their arbitrary bench
     * poses would re-anchor the device off-field and poison the next real seed.
     *
     * <p>Also refreshes {@link #autonomousEndPose} (the seed's fallback), but
     * only when the fused pose is sane — a diverged pose must never become the
     * next run's seed, from either source.
     */
    public void saveFusedPoseToPinpoint() {
        try {
            Pose fused = follower.pose();
            if (fused == null) {
                return;
            }
            // Same 2x-field sanity bound as VisionFusion.correct(): a diverged
            // fused pose must not become the next run's seed.
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

    public void RobotRunPeriodic()
    {
        follower.update();//updates this robots pedro Follower (predict: Pinpoint)
        if (visionFusion != null) visionFusion.correct(); // correct: Limelight -> Kalman
        // Relative cell state (cluster tags). Never touches the global pose.
        if (cells != null) {
            cells.update(CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE ? 'B' : 'R');
        }
        CommandScheduler.getInstance().run(); //runs subsystem periodic()s + scheduled commands (AimAtGoal adds data, no update)
        looptime(); //adds loop-time lines, no update — OpMode loop must end with flushTelemetry()
    }

    /**
     * Init-screen periodic: same vision/odometry ticking as
     * {@link #RobotRunPeriodic()}, minus the things that must not run before
     * START (subsystem actuation, the Ivy scheduler, loop-time lines).
     * Call from every OpMode's {@code init_loop()} followed by
     * {@link #flushTelemetry()} so the LL connects, solves, and reports
     * {@code Fusion/*} while the drivers are still setting up.
     *
     * <p>Safe for autos: the vision correction does NOT move the robot and
     * re-running it during init is exactly what the drivers see on the Field
     * widget. Autos still re-assert their start pose AFTER this runs
     * (see the base init_loop) so the correction cannot drift a stored pose.
     */
    public void InitRunPeriodic()
    {
        follower.update(); // predict step: keeps the pose fresh (robot is parked)
        if (visionFusion != null) visionFusion.correct(); // LL connect + solve + fuse
        if (cells != null) {
            cells.update(CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE ? 'B' : 'R');
        }
        // RunPeriodic()/Scheduler/looptime() intentionally omitted — see above.
    }

    /**
     * One batched Field-widget draw: fused robot (alliance colors) + raw
     * MT1 (yellow) + raw MT2 (green) in a single canvas flush. Reads the
     * latest raw-solve snapshot from {@link #visionFusion} (null-safe when
     * vision is absent/disabled — draws the robot alone).
     *
     * <p>Called once per tick from {@link #flushTelemetry()} — never call
     * directly from OpModes/keys. Centralizing here (rather than in
     * {@link #RobotRunPeriodic()}) guarantees the draw sees the FINAL pose of
     * the loop, including KeyBindings'/FusionTune's trailing drive update.
     */
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
        // Turret ray queues BEFORE drawFusedAndVision: that call queues the
        // robot + ghost shapes and then flushes the whole frame in one send
        // (the renderer's batching contract — never flush twice per loop).
        try {
            if (turret != null) {
                fieldRenderer.drawTurretRay(fused, turret.getPosition());
            }
        } catch (Exception ignored) {
        }
        try {
            fieldRenderer.drawFusedAndVision(fused, mt1, mt2);
        } catch (Exception ignored) {
        }
    }
    /**
     * Single Panels flush for the whole tick. Call ONCE, as the last line of
     * OpMode loop()/init()/init_loop(), after keys.update()/UpdateTelemetry()
     * have added their lines. Panels TelemetryManager sends AND clears its
     * buffer on every update(), so any second update in the same tick sends a
     * partial frame (flicker) and anything added after the last update waits
     * a full tick (lag).
     *
     * <p>Also owns the once-per-tick Field draw ({@link #drawField()}: fused
     * robot + raw MT1/MT2 ghosts, one canvas flush) and the once-per-tick
     * vision snapshot ({@code Fusion/*} + {@code Tip/*} via
     * {@code visionFusion.report()}). correct() may run twice per loop
     * (predict + trailing update) but never reports itself, so this is the
     * single place those lines are added — exactly one set per frame. Drawing
     * here (not in {@link #RobotRunPeriodic()}) means TeleOp's trailing drive
     * update is already applied, so the painted pose is never a loop stale.
     */
    public void flushTelemetry() {
        if (!DASHBOARD) {
            // Dashboard off: no field draw, no Fusion/* lines, no capture, no
            // Panels send. The only Panels interaction left is the file logger
            // (its own switch) — and it must NOT read pending lines before
            // DashTelemetry.update() would have consumed them, so snapshot
            // capture is also skipped: the file logger's log(key, value)
            // event API still works for explicit diagnostics.
            if (telemetryLogger != null && TELEMETRY_IO) {
                telemetryLogger.log("Loop", "dashboard off");
            }
            return;
        }
        drawField();
        if (visionFusion != null) visionFusion.report();
        reportTurret();
        // Capture telemetry snapshot BEFORE update
        if (telemetryLogger != null && TELEMETRY_IO) {
            telemetryLogger.captureSnapshot(DashTelemetry);
        }
        DashTelemetry.update();
    }

    /**
     * Turret heading readout for the dashboard (requested session-2 feature):
     * robot-relative comes straight from the turret encoder (0 = chassis
     * forward, +CCW); field-relative adds the chassis heading from the
     * follower (Pedro heading, CCW+ radians, 0 = +X) and normalizes to
     * +/-180. Bench-verify the sign ONCE: hold the turret still, rotate the
     * chassis CCW — the field-relative value should stay put (it moves with
     * the turret's robot-relative reading, so a sign error shows up there).
     */
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
    public void looptime() {
        long currentTime = System.nanoTime();

        // Calculate loop time in milliseconds
        double loopTimeMs = (currentTime - lastTime) / 1_000_000.0;
        lastTime = currentTime;

        // ... Your Robot Logic Here ...

        RobotMain.DashTelemetry.addData("Loop Time (ms)", "%.2f ms", loopTimeMs);
        RobotMain.DashTelemetry.addData("Hz", "%.1f Hz", 1000.0 / loopTimeMs);
        // No update() here — flushed once via flushTelemetry() at end of loop().
    }
}
