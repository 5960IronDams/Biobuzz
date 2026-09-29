package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.RunPeriodic;
import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.clearAll;

import androidx.annotation.Nullable;
import android.util.Log;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.killerwatts.Flywheel;
import org.firstinspires.ftc.teamcode.killerwatts.HiveCellMonitor;
import org.firstinspires.ftc.teamcode.killerwatts.Intake;
import org.firstinspires.ftc.teamcode.killerwatts.PositionalServo;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.killerwatts.lib.PanelsFieldRenderer;
import org.firstinspires.ftc.teamcode.killerwatts.Vision;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.VisionFusion;

public class RobotMain {
    public static ALLIANCE_COLOR CurrentAlliance = ALLIANCE_COLOR.ALLIANCE_RED;

    /** Panels telemetry (Panels web UI). DS telemetry stays on opmode.telemetry. */
    public static Telemetry DashTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    public final PanelsFieldRenderer fieldRenderer = new PanelsFieldRenderer();
    //subsystems
    public Follower follower;// = SharedObjects.follower;
    public Intake intake;
    public Flywheel flywheel;
    public PositionalServo PosServ;
    /** Null when no Limelight3A is in the RC config (Pinpoint-only mode). */
    @Nullable
    public Vision vision;
    /** Kalman corrector feeding Limelight solves into the Fusion localizer. */
    public VisionFusion visionFusion;
    /** Relative hive-cell monitor (cluster aiming only, never the global pose). */
    public HiveCellMonitor cells;
    //
    public CommandFactory CommandF;
    public static Pose autonomousEndPose = new Pose(0, 0, 0);
    public RobotMain(OpMode opmode)
    {
        clearAll(); // avoid double-registration of subsystem on re-run or Opmode switch
        Scheduler.reset(); //clears all scheduler commands in ivy after opmode switch.
        //subsystems
        follower = Constants.create(opmode.hardwareMap);
        // Limelight is optional: absent in tuning configs -> fused filter runs
        // Pinpoint-only (VisionFusion reports "no-limelight-configured").
        vision = Vision.tryCreate(opmode.hardwareMap);
        Log.i("IronLog","Vision Loaded");
        if (vision != null) vision.start(0);
        Log.i("IronLog","Vision Started");
        visionFusion = new VisionFusion(follower, vision);
        Log.i("IronLog","VisionFusion Started");
        // Relative cell monitor shares the same Limelight (no extra HW handle).
        cells = new HiveCellMonitor(vision);
        fieldRenderer.drawPedroPose(follower.pose());
        intake = new Intake(opmode);
        //flywheel = new Flywheel(opmode);
        PosServ = new PositionalServo(opmode);


        //after all subsystems are started (i.e their variables point to an object). Build the command factory
        CommandF = new CommandFactory(follower,intake,null,PosServ, opmode.hardwareMap);

        //declare what alliance we are on to BOTH telemetry outputs
        //this is for one good final double check for the driver and co pilot
        opmode.telemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        opmode.telemetry.update();
        RobotMain.DashTelemetry.addData("Current Alliance", RobotMain.CurrentAlliance.toString());
        RobotMain.DashTelemetry.update();
    }


    public void RobotRunPeriodic()
    {
        follower.update();//updates this robots pedro Follower (predict: Pinpoint)
        if (visionFusion != null) visionFusion.correct(); // correct: Limelight -> Kalman
        // Relative cell state (cluster tags). Never touches the global pose.
        if (cells != null) {
            cells.update(CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE ? 'B' : 'R');
        }
        RunPeriodic();//run all registered subsystems periodic (addData only, no update)
        Scheduler.execute(); //eun the Ivy scheduler periodic (AimAtGoal adds data, no update)
        looptime(); //adds loop-time lines, no update — OpMode loop must end with flushTelemetry()
    }
    /**
     * Single Panels flush for the whole loop. Call ONCE, as the last line of
     * OpMode loop(), after keys.update()/UpdateTelemetry() have added their lines.
     * Panels TelemetryManager sends AND clears its buffer on every update(), so
     * any second update in the same loop sends a partial frame (flicker) and
     * anything added after the last update waits a full loop (lag).
     *
     * <p>Also emits the once-per-loop vision snapshot ({@code Fusion/*} +
     * {@code Tip/*} via {@code visionFusion.report()}). correct() may run twice
     * per loop (predict + trailing update) but never reports itself, so this is
     * the single place those lines are added — exactly one set per frame.
     */
    public void flushTelemetry() {
        if (visionFusion != null) visionFusion.report();
        DashTelemetry.update();
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
