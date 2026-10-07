package org.firstinspires.ftc.teamcode.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.WattageLib.Ballistics.NoFireZones;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ShootGateServo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

/**
 * Multi-subsystem compositions only. Single-subsystem commands live on their
 * subsystem ({@code intake.runCmd()/holdCmd()/stopCmd()},
 * {@code flywheel.toRpm()/holdTunable()/stopCmd()/coastToIdleCmd()},
 * {@code PosServ.toggleCmd()/toPos()/nudgeCmd()}).
 */
@Configurable
public class CommandFactory {

    // ---- Shoot LUT (Panels-tunable): distance from the aimed hive -> flywheel RPM ----
    /**
     * PLACEHOLDER curve - tune on the robot (Panels -> CommandFactory).
     * Keys MUST be strictly increasing; distances outside the table clamp to
     * the nearest endpoint RPM. PLAIN LINEAR interpolation between points
     * (the SolversLib InterpLUT this replaces is a monotone cubic SPLINE, not
     * linear) and the arrays are read live, so Panels edits apply instantly -
     * no rebuild step.
     */
    public static double[] LUT_DISTANCES_IN = {6, 36, 48, 60, 72, 96};
    /** RPM at each distance in {@link #LUT_DISTANCES_IN} (same order). */
    public static double[] LUT_RPMS = {500, 1950, 1800, 1700, 1600, 1500};

    /**
     * Max slew of the COMMANDED flywheel setpoint, RPM/sec. The raw LUT output
     * can step for two reasons that are NOT the LUT's shape: the aim lock hops
     * to a different hive target (distance jumps by the target spacing) and
     * fused-pose noise moves the distance every loop. This rate-limits how
     * fast the commanded RPM chases the raw lookup, so the setpoint glides.
     * 0 or negative = instant (slew off).
     */
    public static double LUT_MAX_RPM_SLEW = 1200.0;

    /** Last slew-limited RPM the compound commanded (NaN until the first lock). */
    private double lastCmdRpm = Double.NaN;
    private long lastRpmNs = 0L;

    public final Follower follower;
    public final Intake intake;
    public final Flywheel flywheel;
    public final ShootGateServo PosServ;
    public final HardwareMap hardwareMap;

    /**
     * ONE shared follower requirement wrapper for every AimAtGoal instance
     * (bare aim commands and the one inside ShootAtGoal). Sharing the object
     * makes the scheduler see them as the same requirement, so two aims
     * conflict/interrupt each other instead of silently fighting over the
     * follower.
     */
    private final Subsystem aimFollowerReqt = new SubsystemBase() {};

    public CommandFactory(Follower _follower, Intake _intake, Flywheel _flywheel, ShootGateServo _posServ, HardwareMap _hardwareMap) {
        follower = _follower;
        intake = _intake;
        flywheel = _flywheel;
        PosServ = _posServ;
        hardwareMap = _hardwareMap;
    }

    /**
     * Hold-to-aim: faces the closest AimPoints.pp target for the current alliance
     * (Red* / Blue* filtered, re-picked every loop) while {@code forward}/{@code strafe}
     * keep the driver on translation. Runs until cancelled - hold the left trigger
     * to aim, compose with the flywheel via {@code .until(...)} / parallel / deadline.
     *
     * <p>Pass the same alliance-aware stick mapping the TeleOp uses, e.g. for Red:
     * {@code () -> -gamepad1.left_stick_y} and {@code () -> -gamepad1.left_stick_x}.
     */
    public Command AimAtClosestGoal(DoubleSupplier forward, DoubleSupplier strafe) {
        return new AimAtGoal(follower, hardwareMap, forward, strafe,
                () -> RobotMain.CurrentAlliance, aimFollowerReqt);
    }

    /**
     * Hold-to-shoot: aims at the closest hive target AND spins the flywheel at
     * an RPM looked up from the distance-to-hive LUT. The shoot gate opens only
     * once the flywheel is at that RPM (WaitUntil -> open). On release (or any
     * interrupt) the gate closes and the flywheel coasts to {@code IDLE_RPM}
     * (soft FLOAT slowdown; the PID re-engages to hold idle on arrival).
     *
     * <p>Per loop while held: aim re-picks the closest target, the distance to
     * it is fed through the linear LUT and SLEW-LIMITED (see
     * {@link #LUT_MAX_RPM_SLEW}) before reaching the flywheel setpoint - so
     * aim-lock hops between targets and pose noise glide instead of stepping.
     * The gate is then driven EVERY loop by (at speed) AND (outside all
     * no-fire zones): it opens when both hold, and closes the moment the
     * shooter enters a no-fire zone (ported from IndianaJoe -
     * {@code Ballistics.NoFireZones}), reopening once clear. Entering a zone
     * never stops the flywheel - it just closes the gate so no ball leaves
     * while unsafe.
     *
     * <p>Requirements: aim's follower wrapper + flywheel + gate servo - so a
     * manual servo toggle (X) or nudge (dpad) taken mid-shot interrupts the
     * whole compound (manual override).
     *
     * <p>Pass the same alliance-aware stick mapping the TeleOp uses, e.g. for Red:
     * {@code () -> -gamepad1.left_stick_y} and {@code () -> -gamepad1.left_stick_x}.
     */
    public Command ShootAtGoal(DoubleSupplier forward, DoubleSupplier strafe) {
        AimAtGoal aim = new AimAtGoal(follower, hardwareMap, forward, strafe,
                () -> RobotMain.CurrentAlliance, aimFollowerReqt);

        return new ParallelCommandGroup(
                // 1) Aim: owns the follower turn axis; sticks keep translation.
                aim,

                // 2) RPM manager: distance -> linear LUT -> slew-limited
                //    flywheel setpoint every loop. end() = gate closed +
                //    coast-to-idle (the release behavior), which also runs on
                //    ANY interrupt of the compound.
                new FunctionalCommand(
                        () -> { // initialize: fresh slew state per trigger pull
                            lastCmdRpm = Double.NaN;
                            lastRpmNs = 0L;
                        },
                        () -> {
                            double d = aim.getDistanceInches();
                            if (Double.isNaN(d) || d <= 0) {
                                return; // no lock yet: hold the last setpoint
                            }
                            double raw = lutRpm(d);
                            // Slew-limit the chase so aim-lock hops and pose
                            // noise glide instead of stepping.
                            long nowNs = System.nanoTime();
                            double dt = lastRpmNs == 0L ? 0.02
                                    : Math.min(Math.max((nowNs - lastRpmNs) / 1.0e9, 0.0), 0.25);
                            lastRpmNs = nowNs;
                            if (Double.isNaN(lastCmdRpm) || LUT_MAX_RPM_SLEW <= 0) {
                                lastCmdRpm = raw; // first lock (or slew off): take it directly
                            } else {
                                double maxDelta = LUT_MAX_RPM_SLEW * dt;
                                lastCmdRpm += Math.max(-maxDelta,
                                        Math.min(maxDelta, raw - lastCmdRpm));
                            }
                            flywheel.setTargetRpm(lastCmdRpm);
                            // addData only — flushed once per loop by RobotMain.
                            RobotMain.DashTelemetry.addData("Shoot LUT raw RPM", "%.0f", raw);
                            RobotMain.DashTelemetry.addData("Shoot LUT cmd RPM", "%.0f", lastCmdRpm);
                        },
                        (interrupted) -> {
                            PosServ.goA();          // gate closed (POS_A preset)
                            flywheel.coastToIdle(); // soft idle, NOT a PID slam to idle
                        },
                        () -> false,                // runs until the trigger is released
                        flywheel),

                // 3) Gate: driven every loop by (at speed) AND (outside all
                //    no-fire zones). Opens on both, closes on either failing -
                //    so driving into a no-fire zone mid-shot closes the gate
                //    and it reopens automatically once clear. No-fire zone
                //    math ported straight from the IndianaJoe ballistics.
                new RunCommand(() -> {
                    boolean clear = !NoFireZones.isInNoFireZone(follower.pose());
                    if (flywheel.isHoldingTarget() && flywheel.isAtTargetRpm() && clear) {
                        PosServ.goB(); // gate open (POS_B preset)
                    } else {
                        PosServ.goA(); // gate closed (POS_A preset)
                    }
                }, PosServ));
    }

    /**
     * Linear LUT lookup, edge-clamped to the endpoints. Reads the tunable
     * arrays live, so Panels edits apply on the very next lookup (no rebuild).
     */
    private static double lutRpm(double distIn) {
        double[] xs = LUT_DISTANCES_IN;
        double[] ys = LUT_RPMS;
        int n = Math.min(xs.length, ys.length);
        if (n < 2) {
            return ys.length > 0 ? ys[0] : 0.0;
        }
        if (distIn <= xs[0]) return ys[0];
        if (distIn >= xs[n - 1]) return ys[n - 1];
        for (int i = 0; i < n - 1; i++) {
            if (distIn <= xs[i + 1]) {
                double t = (distIn - xs[i]) / (xs[i + 1] - xs[i]);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return ys[n - 1];
    }
}
