package org.firstinspires.ftc.teamcode.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ShootGateServo;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.turret.TurretPivot;
import org.firstinspires.ftc.teamcode.WattageLib.Ballistics.NoFireZones;

/**
 * Multi-subsystem commands and shooting readiness checks.
 *
 * <ul>
 *   <li>{@link #TrackTarget()} — aims the turret, hood, and flywheel RPM using the distance LUTs.</li>
 *   <li>{@link #FireAtGoal()} — opens the shoot gate and runs transfer when {@link #isReadyToShoot()} is true.</li>
 *   <li>{@link #AutoShootAtGoal()} — runs {@link #TrackTarget()} and {@link #FireAtGoal()} together.</li>
 *   <li>{@link #ManualShootAtGoal()} — fixed-RPM shooting without moving the turret.</li>
 * </ul>
 */
@Configurable
public class CommandFactory {

    // ---- Shoot LUTs (Panels -> CommandFactory): distance (in) -> flywheel RPM / hood angle ----
    /**
     * Lookup table distances in inches (must be strictly increasing).
     * Distances outside this range clamp to the nearest endpoint.
     */
    public static double[] LUT_DISTANCES_IN = {6, 36, 48, 60, 72, 96};
    /** Target flywheel RPM at each distance in {@link #LUT_DISTANCES_IN}. */
    public static double[] LUT_RPMS = {1650, 1800, 1850, 1950, 2200, 2300};
    /** Target hood launch angle (deg, 0 = vertical up, 90 = horizontal) at each LUT distance. */
    public static double[] LUT_HOOD_DEGS = {19.831, 20.831, 24.831, 38.852, 45.852, 45.852};
    /** Estimated ball time-of-flight (s) at each LUT distance for shoot-on-the-move lead. */
    public static double[] LUT_TOF_S = {0.35, 0.45, 0.52, 0.60, 0.70, 0.85};

    // ---- Manual align & shoot mode (Panels -> CommandFactory) ----
    /** Fixed flywheel RPM used by {@link #ManualShootAtGoal()} (turret does not move). */
    public static double MANUAL_SHOOT_RPM = 2000.0;

    /** Maximum hood angle change rate (deg/s) when tracking LUT changes; <= 0 disables slew limit. */
    public static double LUT_MAX_HOOD_SLEW_DPS = 90.0;

    /**
     * Maximum flywheel setpoint change rate (RPM/s). Lower values make speed changes
     * smoother when switching targets; <= 0 jumps immediately.
     */
    public static double LUT_MAX_RPM_SLEW = 1200.0;

    /**
     * Distance deadband (in): ignore distance jitter smaller than this so the LUT
     * setpoint stays steady when the robot is stationary.
     */
    public static double LUT_DISTANCE_DEADBAND_IN = 0.5;

    /**
     * Low-pass filter time constant (seconds) on LUT outputs (`dt / (tau + dt)`).
     * Lower values react faster; 0 disables filtering.
     */
    public static double LUT_FILTER_TAU_S = 0.02;

    private double lastCmdRpm = Double.NaN;
    private long lastRpmNs = 0L;
    private double commandedRpm = Double.NaN;
    private double lastLutDist = Double.NaN;
    private double lutFilteredRpm = Double.NaN;

    private double lastCmdHoodDeg = Double.NaN;
    private long lastHoodNs = 0L;
    private double commandedHoodDeg = Double.NaN;
    private double lutFilteredHoodDeg = Double.NaN;

    public final Follower follower;
    public final Intake intake;
    public final Transfer transfer;
    public final Flywheel flywheel;
    public final ShootGateServo PosServ;
    public final TurretPivot turret;
    public final Hood hood;
    public final HardwareMap hardwareMap;

    /** Shared alliance-aware aim tracker (null when turret is not present). */
    public final AimAtGoal aim;

    // =====================================================================
    //  READINESS THRESHOLDS (Panels -> CommandFactory)
    // =====================================================================

    /** Max chassis translation speed (in/s) allowed while auto-firing. */
    public static double MAX_TRANSLATION_SPEED_IN_S = 40.0;

    /** Max chassis |yaw rate| (deg/s) allowed while auto-firing. */
    public static double MAX_ROTATION_SPEED_DPS = 80.0;

    // ---- Flywheel readiness hysteresis ----
    /** Flywheel smoothed error (RPM) within which the wheel enters the "at speed" state. */
    public static double FLYWHEEL_READY_ON_RPM = 85.0;
    /** Optional continuous dwell (s) within {@link #FLYWHEEL_READY_ON_RPM} before firing (0 = immediate). */
    public static double FLYWHEEL_SETTLE_DWELL_S = 0.0;
    /** Flywheel error (RPM) above which an already-ready wheel stops firing (wider to survive ball dips). */
    public static double FLYWHEEL_READY_OFF_RPM = 175.0;

    /** Extra margin (deg) added to live aim tolerance before an aligned turret drops out of ready state. */
    public static double AIM_HYST_EXIT_MARGIN_DEG = 2.0;

    /** Extra margin (deg) added to {@link Hood#TOLERANCE_DEG} before a settled hood drops out of ready state. */
    public static double HOOD_HYST_EXIT_MARGIN_DEG = 1.5;

    /** Whether chassis translation speed must be below {@link #MAX_TRANSLATION_SPEED_IN_S} to fire. */
    public static boolean REQUIRE_STABLE_TRANSLATION = true;
    /** Whether chassis yaw rate must be below {@link #MAX_ROTATION_SPEED_DPS} to fire. */
    public static boolean REQUIRE_STABLE_ROTATION = true;
    /** Grace period (s) after OpMode start during which chassis motion checks are skipped. */
    public static double MOTION_WARMUP_S = 0.5;

    // =====================================================================
    //  State
    // =====================================================================

    /** Measured chassis translation speed (in/s) and yaw rate (deg/s). */
    private double measuredSpeedInS = 0.0;
    private double measuredYawDps = 0.0;
    private double prevPoseX = Double.NaN;
    private double prevPoseY = Double.NaN;
    private double prevPoseHeading = Double.NaN;
    private long prevPoseNs = 0L;
    private long lastPeriodicNs = 0L;
    private double warmupRemainingS = MOTION_WARMUP_S;

    /** Hysteresis latches for readiness members that sit near a threshold during SOTM. */
    private final Hysteresis alignHyst = new Hysteresis();
    private final Hysteresis hoodHyst = new Hysteresis();
    private boolean flywheelBandLatched = false;
    /** Timestamp (ns) when actual RPM first entered FLYWHEEL_READY_ON_RPM continuously. */
    private long flywheelInToleranceSinceNs = 0L;

    private boolean lastManualFlywheelReady = false;
    private boolean lastManualGateOpen = false;
    private boolean lastManualReady = false;

    /** What the fire half is currently asking the transfer to do (for the dashboard). */
    private FeedState transferState = FeedState.OFF;

    /** Intended transfer state, so stop()/runForward() are only written on a change. */
    private enum FeedState { OFF, FEEDING, REVERSING, MANUAL }

    private void setTransferState(FeedState s) {
        transferState = s;
    }

    /** True while the fire half is feeding (updated by {@link #FireAtGoal()}). */
    private boolean firing = false;
    /** Set when firing stopped the driver's intake; restored once firing is over. */
    private boolean intakeStoppedByFire = false;
    /** Driver-interrupt memory: a manual transfer command must not be instantly overridden. */
    private boolean fireSuppressedByDriver = false;
    private double fireSuppressedForS = 0.0;

    /**
     * The ONE latched auto-shoot instance. Single instance matters:
     * {@code KeyBindings} toggles it with {@code toggleWhenActive} and cancels
     * it from the panic button, and SolversLib's {@code toggleWhenActive}
     * tracks pressed/released state on the trigger's bound Runnable — a new
     * instance per call would leave the button and the robot disagreeing about
     * whether the shot is on.
     */
    private final Command autoShootCmd;

    /**
     * The ONE latched manual-shoot instance: fixed RPM, no turret movement,
     * auto-feeds via transfer when at speed.
     */
    private final Command manualShootCmd;

    public CommandFactory(Follower _follower, Intake _intake, Transfer _transfer, Flywheel _flywheel, ShootGateServo _posServ, TurretPivot _turret, Hood _hood, HardwareMap _hardwareMap) {
        follower = _follower;
        intake = _intake;
        transfer = _transfer;
        flywheel = _flywheel;
        PosServ = _posServ;
        turret = _turret;
        hood = _hood;
        hardwareMap = _hardwareMap;

        // The shared aim model (alliance points + shot-aware tolerance).
        aim = turret == null ? null
                : new AimAtGoal(turret, follower, hardwareMap,
                        () -> RobotMain.CurrentAlliance, this::getCommandedRpm);

        // Build the single latched instance LAST — its halves need `aim`.
        autoShootCmd = aim == null ? null : buildAutoShoot();
        // Manual shoot requires flywheel; does not require turret or aim.
        manualShootCmd = flywheel == null ? null : buildManualShoot();
    }

    public CommandFactory(Follower _follower, Intake _intake, Transfer _transfer, Flywheel _flywheel, ShootGateServo _posServ, TurretPivot _turret, HardwareMap _hardwareMap) {
        this(_follower, _intake, _transfer, _flywheel, _posServ, _turret, null, _hardwareMap);
    }

    // =====================================================================
    //  BALL FLOW (unchanged)
    // =====================================================================

    /**
     * Ball-flow compound: intake AND transfer both run FORWARD, re-reading
     * their live RPMs every loop (Panels -&gt; Intake / Transfer retunes apply
     * while held). Runs until replaced/stopped - pair with {@link #feedStopCmd()}
     * (or bind releases to it). Requires both subsystems, so a manual intake or
     * transfer command interrupts it.
     */
    public Command feedCmd() {
        return new ParallelCommandGroup(
                new RunCommand(intake::runIntake, intake),
                new RunCommand(transfer::runForward, transfer));
    }

    /** PID-brake BOTH ball-flow motors to zero (intake + transfer). */
    public Command feedStopCmd() {
        return new ParallelCommandGroup(
                intake.stopCmd(),
                transfer.stopCmd());
    }

    // =====================================================================
    //  AIM
    // =====================================================================

    /**
     * Hold-to-aim: SPINS THE TURRET to face the closest AimPoints.pp target
     * for the current alliance (Red* / Blue* filtered, re-picked every loop)
     * while the driver keeps full chassis control (translation AND rotation).
     * Requires only the turret subsystem — the follower is read for the pose
     * but never driven. Runs until cancelled.
     */
    public Command AimAtClosestGoal() {
        if (aim == null) {
            return Commands.none();
        }
        // Shared, pre-parsed aim points (the asset is read ONCE for the whole
        // OpMode, not once per command construction) but a FRESH AimAtGoal:
        // a Command may join exactly one CommandGroup, so returning anything
        // cached here would break the second call. `aim` is passed rather than
        // re-loaded so repeated construction stays free.
        return new AimAtGoal(turret, follower, aim.aimPoints,
                () -> RobotMain.CurrentAlliance, this::getCommandedRpm, aim);
    }

    /**
     * The TRACK half of the shot — the left trigger's latched state: aim the
     * turret at the closest alliance target AND hold the flywheel at the
     * distance-to-hive LUT RPM. Never opens the gate and never feeds; that is
     * {@link #FireAtGoal()}'s job, which runs alongside it gated on
     * {@link #isReadyToShoot()}.
     *
     * <p>Requirements: the turret (aim) + the RPM manager (flywheel) — a manual
     * turret spin or flywheel command interrupts it.
     *
     * <p>Every call returns a NEW command tree.
     */
    public Command TrackTarget() {
        if (aim == null) {
            return Commands.none();
        }
        // Fresh AimAtGoal per TrackTarget, NOT the shared `aim` field: a Command
        // may join exactly one CommandGroup (CommandGroupBase enforces this in a
        // static set). Wrapping the field instance would make the SECOND
        // AutoShootAtGoal() throw "Commands cannot be added to more than one
        // CommandGroup". The `aim` field stays the long-lived instance for
        // TeleOp's toggleWhenActive(); its live state is exposed through the
        // is*OnTarget()/live*() accessors either way.
        if (hood != null) {
            return new ParallelCommandGroup(
                    // 1) Aim: spins the TURRET to the target; chassis stays with the driver.
                    AimAtClosestGoal(),
                    // 2) RPM manager: distance -> linear LUT -> slew-limited setpoint.
                    rpmManagerCommand(),
                    // 3) Hood manager: distance -> linear LUT -> slew-limited launch angle.
                    hoodManagerCommand()
            ).setName("TrackTarget");
        }
        return new ParallelCommandGroup(
                // 1) Aim: spins the TURRET to the target; chassis stays with the driver.
                AimAtClosestGoal(),
                // 2) RPM manager: distance -> linear LUT -> slew-limited setpoint.
                rpmManagerCommand()
        ).setName("TrackTarget");
    }

    /**
     * The FIRE half of the shot: opens the gate when shooting starts, keeps it
     * open throughout the shot session, and closes it when shooting stops.
     * Moment-to-moment readiness pauses stop/start the TRANSFER rather than
     * slamming the gate shut mid-motion.
     *
     * <p>Requirements: gate servo + transfer ONLY. The intake is deliberately
     * not required — see {@link #holdManualIntakeOutOfTheWay()}.
     */
    public Command FireAtGoal() {
        final Subsystem[] requirements = new Subsystem[]{PosServ, transfer};
        return new FunctionalCommand(
                () -> {
                    firing = true;
                    fireSuppressedByDriver = false;
                    fireSuppressedForS = 0.0;
                    alignHyst.reset(aimAlignedHyst());
                    hoodHyst.reset(hoodAtAngleHyst());
                    flywheelBandLatched = false;
                    flywheelInToleranceSinceNs = 0L;
                    transferState = FeedState.OFF;
                    openGate();
                },
                () -> {
                    // Evaluate once on this tick using up-to-date subsystem and TrackTarget states.
                    Readiness r = evaluateReadiness();
                    lastReadiness = r;
                    boolean manualFwd = manualTransferForwardHeld;
                    boolean manualRev = manualTransferReverseHeld;
                    boolean autoReady = !fireSuppressedByDriver && r.all;

                    if (manualRev) {
                        closeGate();
                        if (transfer != null) transfer.runReverse();
                        setTransferState(FeedState.REVERSING);
                    } else if (manualFwd) {
                        openGate();
                        holdManualIntakeOutOfTheWay();
                        if (transfer != null) transfer.runForward();
                        setTransferState(FeedState.MANUAL);
                    } else if (!fireSuppressedByDriver && r.shooterReady) {
                        openGate();
                        if (r.gate) {
                            holdManualIntakeOutOfTheWay();
                            if (transfer != null) transfer.runForward();
                            setTransferState(FeedState.FEEDING);
                        } else {
                            if (transfer != null && transferState != FeedState.OFF) transfer.stop();
                            setTransferState(FeedState.OFF);
                        }
                    } else {
                        // Leave the gate open during active auto-shooting; pause transfer only.
                        openGate();
                        if (transfer != null && transferState != FeedState.OFF) transfer.stop();
                        setTransferState(FeedState.OFF);
                    }
                    if (RobotMain.DASHBOARD) {
                        reportReadinessDetails(r, autoReady, manualFwd, manualRev);
                    }
                },
                (interrupted) -> {
                    firing = false;
                    closeGate();
                    if (transfer != null) transfer.stop();
                    setTransferState(FeedState.OFF);
                    flywheelBandLatched = false;
                    flywheelInToleranceSinceNs = 0L;
                    // A manual transfer command interrupting us is a driver
                    // override (jam-clear): do not slam straight back.
                    if (interrupted) {
                        fireSuppressedByDriver = true;
                        fireSuppressedForS = 0.0;
                    }
                },
                () -> false, // latched
                requirements
        ).setName("FireAtGoal");
    }

    /**
     * The FIRE half of the manual shoot mode: opens the gate on activation and
     * feeds via the transfer as soon as the flywheel is at {@link #MANUAL_SHOOT_RPM}
     * and the gate has finished opening. Closes the gate when manual shoot ends.
     */
    public Command FireManualAtGoal() {
        List<Subsystem> reqList = new ArrayList<>();
        if (PosServ != null) reqList.add(PosServ);
        if (transfer != null) reqList.add(transfer);
        final Subsystem[] requirements = reqList.toArray(new Subsystem[0]);

        return new FunctionalCommand(
                () -> {
                    firing = true;
                    fireSuppressedByDriver = false;
                    fireSuppressedForS = 0.0;
                    flywheelBandLatched = false;
                    flywheelInToleranceSinceNs = 0L;
                    transferState = FeedState.OFF;
                    openGate();
                },
                () -> {
                    boolean flywheelReady = flywheelReadyHyst();
                    boolean gateOpenNow = gateOk();
                    boolean ready = flywheelReady && gateOpenNow;
                    lastManualFlywheelReady = flywheelReady;
                    lastManualGateOpen = gateOpenNow;
                    lastManualReady = ready;
                    boolean manualFwd = manualTransferForwardHeld;
                    boolean manualRev = manualTransferReverseHeld;
                    boolean autoReady = !fireSuppressedByDriver && ready;

                    if (manualRev) {
                        closeGate();
                        if (transfer != null) transfer.runReverse();
                        setTransferState(FeedState.REVERSING);
                    } else if (manualFwd) {
                        openGate();
                        holdManualIntakeOutOfTheWay();
                        if (transfer != null) transfer.runForward();
                        setTransferState(FeedState.MANUAL);
                    } else if (!fireSuppressedByDriver && flywheelReady) {
                        openGate();
                        if (gateOpenNow) {
                            holdManualIntakeOutOfTheWay();
                            if (transfer != null) transfer.runForward();
                            setTransferState(FeedState.FEEDING);
                        } else {
                            if (transfer != null && transferState != FeedState.OFF) transfer.stop();
                            setTransferState(FeedState.OFF);
                        }
                    } else {
                        openGate();
                        if (transfer != null && transferState != FeedState.OFF) transfer.stop();
                        setTransferState(FeedState.OFF);
                    }
                    if (RobotMain.DASHBOARD) {
                        reportManualReadinessDetails(flywheelReady, gateOpenNow, ready, autoReady, manualFwd, manualRev);
                    }
                },
                (interrupted) -> {
                    firing = false;
                    closeGate();
                    if (transfer != null) transfer.stop();
                    setTransferState(FeedState.OFF);
                    flywheelBandLatched = false;
                    flywheelInToleranceSinceNs = 0L;
                    if (interrupted) {
                        fireSuppressedByDriver = true;
                        fireSuppressedForS = 0.0;
                    }
                },
                () -> false, // latched until cancelled
                requirements
        ).setName("FireManualAtGoal");
    }

    /**
     * Full auto-fire command: runs {@link #TrackTarget()} and {@link #FireAtGoal()} in parallel
     * so the transfer feeds automatically whenever readiness gates pass.
     *
     * <p>This is a FACTORY: every call returns a NEW, ungrouped command tree.
     * Do not "optimise" it back into {@code return autoShootCmd} — a Command
     * may join exactly one CommandGroup (CommandGroupBase enforces this in a
     * static set), so a cached instance throws "Commands cannot be added to
     * more than one CommandGroup" the second time it is decorated with
     * {@code .withTimeout()} or placed in a group. A TeleOp that needs one
     * STABLE instance for {@code toggleWhenActive()} must use
     * {@link #getAutoShootCommand()} instead.
     */
    public Command AutoShootAtGoal() {
        return aim == null ? Commands.none() : buildAutoShoot();
    }

    /** Builds the latched compound (called once, from the constructor). */
    private Command buildAutoShoot() {
        return new ParallelCommandGroup(
                //intake.runCmd(),//reserves the intake and this causes issues in tele op for some reason.
                TrackTarget(),
                FireAtGoal()
        ).setName("AutoShootAtGoal");
    }

    /**
     * Builds the compound manual shoot command: runs flywheel to MANUAL_SHOOT_RPM
     * and auto-fires via transfer when at speed, without requiring or moving the turret.
     */
    private Command buildManualShoot() {
        if (flywheel == null) {
            return Commands.none();
        }
        return new ParallelCommandGroup(
                manualRpmCommand(),
                FireManualAtGoal()
        ).setName("ManualShootAtGoal");
    }

    /**
     * Factory for a fresh manual align and shoot command tree (fixed RPM, no turret movement).
     */
    public Command ManualShootAtGoal() {
        return flywheel == null ? Commands.none() : buildManualShoot();
    }

    /**
     * Hold-to-shoot command: aims the turret, tracks the distance LUT for flywheel
     * and hood, and runs the gate/transfer whenever {@link #isReadyToShoot()} passes.
     */
    public Command ShootAtGoal() {
        return AutoShootAtGoal();
    }

    /**
     * The latched AT-FIELD bridge used by {@code KeyBindings}/{@code RobotMain}
     * when it needs one command object to schedule: the auto-firing shoot.
     * Null when the turret is absent from the RC config.
     */
    public Command getAutoShootCommand() {
        return autoShootCmd;
    }

    /**
     * The latched manual shoot instance used by KeyBindings on operator right trigger.
     */
    public Command getManualShootCommand() {
        return manualShootCmd;
    }

    /** True if the manual shoot command is currently scheduled. */
    public boolean isManualShootActive() {
        return manualShootCmd != null && manualShootCmd.isScheduled();
    }

    /**
     * The LEFT BUMPER "unlock / panic" command: turn the auto-shoot latch OFF
     * and put the shooter in its safe rest state.
     *
     * <p>It cancels {@link #autoShootCmd} and {@link #manualShootCmd} — NOT whatever happens
     * to be scheduled — because {@code KeyBindings} toggles those exact
     * instances with {@code toggleWhenActive}; cancelling them is what keeps the
     * trigger's pressed/released bookkeeping in lockstep with the robot (the
     * next trigger press schedules it cleanly again). The cancel runs the
     * compound's {@code end()} on both halves: gate closed, transfer stopped,
     * flywheel coasts to idle, turret released to NEUTRAL.
     *
     * <p>It takes NO subsystem requirements and finishes immediately, so it can
     * never fight the compound for the same subsystems — it only calls
     * fire-and-forget setters on them. Idempotent: pressing it with the latch
     * already off just re-asserts the safe state.
     */
    public Command getStopShootCommand() {
        if (autoShootCmd == null && manualShootCmd == null) {
            return Commands.none();
        }
        return new FunctionalCommand(
                () -> {
                    if (autoShootCmd != null && autoShootCmd.isScheduled()) {
                        autoShootCmd.cancel(); // runs both halves' end()
                    }
                    if (manualShootCmd != null && manualShootCmd.isScheduled()) {
                        manualShootCmd.cancel();
                    }
                    // Belt-and-suspenders: whatever end() did, make the safe
                    // resting state explicit.
                    closeGate(); // no-op until the gate exists
                    if (transfer != null) transfer.stop();
                    if (flywheel != null) flywheel.coastToIdle();
                    if (turret != null) turret.BRAKE();
                    if (hood != null) hood.stow();
                    firing = false;
                    intakeStoppedByFire = false;
                    fireSuppressedByDriver = false; // a deliberate unlock re-arms for next time
                    if (RobotMain.DASHBOARD) {
                        RobotMain.DashTelemetry.addData("Shoot latch (LT toggle)", "OFF (LB unlock)");
                        RobotMain.DashTelemetry.addData("Manual shoot latch (RT toggle)", "OFF (LB unlock)");
                    }
                },
                () -> {},
                (interrupted) -> {},
                () -> true
        ).setName("StopAutoShoot");
    }


    /** A fresh track-only command instance (aim + RPM, no feeding). */
    public Command getTrackCommand() {
        return TrackTarget();
    }

    /** A fresh gated fire command instance (gate + transfer). */
    public Command getFireCommand() {
        return FireAtGoal();
    }

    /** True while the fire half is feeding right now. */
    public boolean isFiring() {
        return firing;
    }

    // ---------------------------------------------------------------------
    //  RPM manager
    // ---------------------------------------------------------------------

    /**
     * Distance -> linear LUT -> slew-limited flywheel setpoint every loop.
     * The LUT input is the shared aim instance's locked distance while it is
     * aiming, and the live target distance otherwise — so the RPM is always
     * computed for the point the turret is being driven at.
     */
    private Command rpmManagerCommand() {
        return new FunctionalCommand(
                () -> { // initialize: fresh slew state per activation
                    lastCmdRpm = Double.NaN;
                    lastRpmNs = 0L;
                    commandedRpm = Double.NaN;
                    lastLutDist = Double.NaN;
                    lutFilteredRpm = Double.NaN;
                },
                () -> {
                    double d = currentShotDistanceInches();
                    if (Double.isNaN(d) || d <= 0) {
                        return; // no lock yet: hold the last setpoint
                    }
                    if (Double.isNaN(lastLutDist)
                            || Math.abs(d - lastLutDist) > LUT_DISTANCE_DEADBAND_IN) {
                        lastLutDist = d;
                    } else {
                        d = lastLutDist;
                    }
                    double raw = lutRpm(d);
                    long nowNs = System.nanoTime();
                    double dt = lastRpmNs == 0L ? 0.02
                            : Math.min(Math.max((nowNs - lastRpmNs) / 1.0e9, 0.0), 0.25);
                    lastRpmNs = nowNs;
                    // Time-constant low-pass filter on the LUT output.
                    if (Double.isNaN(lutFilteredRpm) || LUT_FILTER_TAU_S <= 0.0) {
                        lutFilteredRpm = raw;
                    } else {
                        double a = dt / (LUT_FILTER_TAU_S + dt);
                        lutFilteredRpm += a * (raw - lutFilteredRpm);
                    }
                    raw = lutFilteredRpm;
                    // Slew-limit the chase so aim-lock hops and pose noise
                    // glide instead of stepping.
                    if (Double.isNaN(lastCmdRpm) || LUT_MAX_RPM_SLEW <= 0) {
                        lastCmdRpm = raw; // first lock (or slew off): take it directly
                    } else {
                        double maxDelta = LUT_MAX_RPM_SLEW * dt;
                        lastCmdRpm += Math.max(-maxDelta,
                                Math.min(maxDelta, raw - lastCmdRpm));
                    }
                    commandedRpm = lastCmdRpm;
                    flywheel.setTargetRpm(lastCmdRpm);
                    if (RobotMain.DASHBOARD) {
                        RobotMain.DashTelemetry.addData("Shoot LUT dist (in)", "%.1f", d);
                        RobotMain.DashTelemetry.addData("Shoot LUT raw RPM", "%.0f", raw);
                        RobotMain.DashTelemetry.addData("Shoot LUT cmd RPM", "%.0f", lastCmdRpm);
                    }
                },
                (interrupted) -> {
                    closeGate();                       // no-op until the gate exists
                    flywheel.coastToIdle();            // soft idle, NOT a PID slam to idle
                    lastCmdRpm = Double.NaN;
                    commandedRpm = Double.NaN;
                    lastLutDist = Double.NaN;
                    lutFilteredRpm = Double.NaN;
                },
                () -> false, // runs until the command is cancelled
                flywheel);
    }

    /**
     * Distance -> linear LUT -> slew-limited Hood launch angle every loop.
     * The LUT input is the shared aim instance's locked distance while aiming,
     * or the live target distance otherwise. Reuses the same distance deadband
     * as the flywheel to prevent pose noise from moving the hood.
     */
    private Command hoodManagerCommand() {
        if (hood == null) {
            return Commands.none();
        }
        return new FunctionalCommand(
                () -> {
                    lastCmdHoodDeg = Double.NaN;
                    lastHoodNs = 0L;
                    commandedHoodDeg = Double.NaN;
                    lutFilteredHoodDeg = Double.NaN;
                },
                () -> {
                    double d = currentShotDistanceInches();
                    if (Double.isNaN(d) || d <= 0) {
                        return;
                    }
                    if (!Double.isNaN(lastLutDist)) {
                        d = lastLutDist;
                    }
                    double raw = lutHoodDeg(d);
                    long nowNs = System.nanoTime();
                    double dt = lastHoodNs == 0L ? 0.02
                            : Math.min(Math.max((nowNs - lastHoodNs) / 1.0e9, 0.0), 0.25);
                    lastHoodNs = nowNs;

                    if (Double.isNaN(lutFilteredHoodDeg) || LUT_FILTER_TAU_S <= 0.0) {
                        lutFilteredHoodDeg = raw;
                    } else {
                        double a = dt / (LUT_FILTER_TAU_S + dt);
                        lutFilteredHoodDeg += a * (raw - lutFilteredHoodDeg);
                    }
                    raw = lutFilteredHoodDeg;

                    if (Double.isNaN(lastCmdHoodDeg) || LUT_MAX_HOOD_SLEW_DPS <= 0) {
                        lastCmdHoodDeg = raw;
                    } else {
                        double maxDelta = LUT_MAX_HOOD_SLEW_DPS * dt;
                        lastCmdHoodDeg += Math.max(-maxDelta, Math.min(maxDelta, raw - lastCmdHoodDeg));
                    }
                    commandedHoodDeg = lastCmdHoodDeg;
                    hood.setTargetDegrees(lastCmdHoodDeg);

                    if (RobotMain.DASHBOARD) {
                        RobotMain.DashTelemetry.addData("Shoot LUT raw Hood deg", "%.2f", raw);
                        RobotMain.DashTelemetry.addData("Shoot LUT cmd Hood deg", "%.2f", lastCmdHoodDeg);
                    }
                },
                (interrupted) -> {
                    if (hood != null) hood.stow();
                    lastCmdHoodDeg = Double.NaN;
                    commandedHoodDeg = Double.NaN;
                    lutFilteredHoodDeg = Double.NaN;
                },
                () -> false,
                hood
        );
    }

    /**
     * Spools the flywheel directly to {@link #MANUAL_SHOOT_RPM} without pose or distance LUT.
     * On interrupt / end, coasts the flywheel to idle.
     */
    private Command manualRpmCommand() {
        if (flywheel == null) {
            return Commands.none();
        }
        return new FunctionalCommand(
                () -> {
                    commandedRpm = MANUAL_SHOOT_RPM;
                    flywheel.setTargetRpm(MANUAL_SHOOT_RPM);
                },
                () -> {
                    commandedRpm = MANUAL_SHOOT_RPM;
                    if (Flywheel.TARGET_RPM != MANUAL_SHOOT_RPM) {
                        flywheel.setTargetRpm(MANUAL_SHOOT_RPM);
                    }
                    if (RobotMain.DASHBOARD) {
                        RobotMain.DashTelemetry.addData("Manual Shoot cmd RPM", "%.0f", MANUAL_SHOOT_RPM);
                    }
                },
                (interrupted) -> {
                    closeGate();
                    flywheel.coastToIdle();
                    commandedRpm = Double.NaN;
                },
                () -> false, // runs until cancelled
                flywheel
        ).setName("ManualRpmManager");
    }

    /** The distance (in) from the shooter to the target the shot is being made at. */
    private double currentShotDistanceInches() {
        if (aim == null) {
            return Double.NaN;
        }
        return aim.activeDistanceIn;
    }

    // =====================================================================
    //  READINESS TRIGGERS — the Biobuzz port of SystemController's set
    // =====================================================================

    /** True when the FLYWHEEL is at the commanded speed. */
    public Trigger isFlywheelAtSpeed() {
        return new Trigger(this::flywheelAtSpeed);
    }


    /** True when the TURRET is ALIGNED on the live target within the shot-scaled tolerance. */
    public Trigger isTurretOnTarget() {
        return new Trigger(() -> aim != null
                && !Double.isNaN(aim.activeHeadingErrorDeg)
                && Math.abs(aim.activeHeadingErrorDeg) <= aim.activeToleranceDeg);
    }

    /** True when the TURRET has reached a commanded setpoint (FRC {@code isHoodAtAngle()} slot). */
    public Trigger isTurretAtSetpoint() {
        return new Trigger(this::turretAtSetpoint);
    }

    /** True when the HOOD has reached its commanded launch angle. */
    public Trigger isHoodAtAngle() {
        return new Trigger(this::hoodAtAngle);
    }

    private boolean hoodAtAngle() {
        if (hood == null) {
            return true;
        }
        return hood.isAtTargetDegrees(Hood.TOLERANCE_DEG);
    }

    /**
     * Hood readiness with hysteresis so small single-loop LUT steps during SOTM
     * do not momentarily drop readiness between Hood.periodic() and hoodManagerCommand().
     */
    private boolean hoodAtAngleHyst() {
        if (hood == null) {
            return true;
        }
        double err = Math.abs(hood.getCurrentDegrees() - hood.getTargetDegrees());
        double on = Hood.TOLERANCE_DEG;
        double off = on + Math.max(0.0, HOOD_HYST_EXIT_MARGIN_DEG);
        return hoodHyst.update(err, on, off);
    }

    /** True when the shoot gate servo is open and finished slewing. */
    public Trigger isGateSettled() {
        return new Trigger(this::gateOk);
    }

    /** True when the chassis TRANSLATION speed is under {@link #MAX_TRANSLATION_SPEED_IN_S}. */
    public Trigger isDrivetrainStable() {
        return new Trigger(() -> !translationMatters() || measuredSpeedInS <= MAX_TRANSLATION_SPEED_IN_S);
    }

    /** True when the chassis |yaw rate| is under {@link #MAX_ROTATION_SPEED_DPS}. */
    public Trigger isRotationStable() {
        return new Trigger(() -> !rotationMatters() || Math.abs(measuredYawDps) <= MAX_ROTATION_SPEED_DPS);
    }

    /** True when the SHOOTER is clear of every no-fire zone (and the shot path leaves the field). */
    public Trigger isShooterClear() {
        return new Trigger(this::shooterIsClear);
    }

    /** The raw AND of every member (drives fresh each call) — for diagnostics. */
    public Trigger isReadyNow() {
        return new Trigger(() -> evaluateReadiness().all);
    }

    /** True when all readiness checks (including gate open) pass in the current loop snapshot. */
    public Trigger isReadyToShoot() {
        return new Trigger(() -> getLatestReadiness().all);
    }

    /** One immutable snapshot of every readiness member for THIS loop. */
    public static final class Readiness {
        public final boolean flywheel;
        public final boolean turretPos;
        public final boolean aim;
        public final boolean gate;
        public final boolean translation;
        public final boolean rotation;
        public final boolean clear;
        public final boolean hood;
        public final boolean shooterReady;
        public final boolean all;

        Readiness(boolean flywheel, boolean turretPos, boolean aim, boolean gate,
                  boolean translation, boolean rotation, boolean clear, boolean hood) {
            this.flywheel = flywheel;
            this.turretPos = turretPos;
            this.aim = aim;
            this.gate = gate;
            this.translation = translation;
            this.rotation = rotation;
            this.clear = clear;
            this.hood = hood;
            this.shooterReady = flywheel && turretPos && aim && translation && rotation && clear && hood;
            this.all = this.shooterReady && gate;
        }
    }

    private Readiness lastReadiness = null;

    /** Returns the latest readiness snapshot (computed in readinessPeriodic()). */
    public Readiness getLatestReadiness() {
        return lastReadiness != null ? lastReadiness : evaluateReadiness();
    }

    /** Shoot readiness summary line for driver station / dashboard. */
    public String getShootReadinessSummary() {
        if (isManualShootActive()) {
            if (lastManualReady) {
                return "MANUAL READY (FIRE)";
            }
            return "MANUAL WAIT: " + (lastManualFlywheelReady ? "gate (opening)" : "flywheel " + flywheelText());
        }
        Readiness r = getLatestReadiness();
        if (r.all) {
            return "READY TO SHOOT (FIRE)";
        }
        return "BLOCKED: " + firstBlocker(r);
    }

    /** Live flywheel readiness details string. */
    public String getFlywheelStatusText() {
        return flywheelText();
    }

    /** Evaluate every member exactly once (hysteresis members advance once here). */
    private Readiness evaluateReadiness() {
        return new Readiness(
                flywheelReadyHyst(),
                turretAtSetpoint(),
                aimAlignedHyst(),
                gateOk(),
                translationOk(),
                rotationOk(),
                shooterIsClear(),
                hoodAtAngleHyst());
    }

    /** Gate readiness check: passes once the gate servo is open and finished slewing. */
    private boolean gateOk() {
        return PosServ == null || PosServ.isOpen();
    }

    // ---- raw predicates the triggers read ----

    private boolean flywheelAtSpeed() {
        return flywheel != null
                && flywheel.isHoldingTarget()   // a COAST/STOP would otherwise read "at 0"
                && flywheel.isAtTargetRpm();
    }

    /**
     * Hysteresis + dwell check on flywheel speed: enters ready after staying within
     * {@link #FLYWHEEL_READY_ON_RPM} for {@link #FLYWHEEL_SETTLE_DWELL_S}, and stays
     * ready until error exceeds {@link #FLYWHEEL_READY_OFF_RPM}.
     */
    private boolean flywheelReadyHyst() {
        if (flywheel == null || !flywheel.isHoldingTarget()) {
            flywheelBandLatched = false;
            flywheelInToleranceSinceNs = 0L;
            return false;
        }
        double target = commandedRpm;
        if (Double.isNaN(target) || target <= 0) {
            // Nothing commanded yet (e.g. before the first LUT lock).
            flywheelBandLatched = false;
            flywheelInToleranceSinceNs = 0L;
            return false;
        }
        double err = Math.abs(flywheel.getSmoothedRpm() - target);
        double bandOn = FLYWHEEL_READY_ON_RPM;
        double bandOff = Math.max(bandOn, FLYWHEEL_READY_OFF_RPM);

        if (flywheelBandLatched) {
            // Once latched, only unlatch if speed drops past bandOff (e.g. ball impact)
            if (err > bandOff) {
                flywheelBandLatched = false;
                flywheelInToleranceSinceNs = 0L;
            }
        } else if (err <= bandOn) {
            if (FLYWHEEL_SETTLE_DWELL_S <= 0.0) {
                flywheelBandLatched = true;
            } else {
                long nowNs = System.nanoTime();
                if (flywheelInToleranceSinceNs == 0L) {
                    flywheelInToleranceSinceNs = nowNs;
                }
                long dwellNs = (long) (FLYWHEEL_SETTLE_DWELL_S * 1.0e9);
                if (nowNs - flywheelInToleranceSinceNs >= dwellNs) {
                    flywheelBandLatched = true;
                }
            }
        } else {
            flywheelInToleranceSinceNs = 0L;
        }
        return flywheelBandLatched;
    }

    /**
     * Turret-aligned check with hysteresis on the angular error, for the same
     * reason: the aim error is live-tracked every loop and the turret sits on
     * its settle boundary, so a single-degree threshold can flip per sample.
     * Enter inside the live tolerance, leave only past the exit margin.
     */
    private boolean aimAlignedHyst() {
        double err = Math.abs(aim == null ? Double.NaN : aim.activeHeadingErrorDeg);
        if (Double.isNaN(err)) {
            alignHyst.latch(false);
            return false;
        }
        double on = aim.activeToleranceDeg;
        if (Double.isNaN(on)) {
            alignHyst.latch(false);
            return false;
        }
        // Release point is a fixed margin wider so a settled turret sticks.
        double off = on + AIM_HYST_EXIT_MARGIN_DEG;
        return alignHyst.update(err, on, off);
    }

    private boolean turretAtSetpoint() {
        if (turret == null) {
            return false;
        }
        // No setpoint is being held (BRAKED/IDLE) => the turret has nothing to be "at".
        if (turret.State == TurretPivot.SubSystemState.BRAKED
                || turret.State == TurretPivot.SubSystemState.IDLE) {
            return false;
        }
        return turret.atSetpoint();
    }

    private boolean translationMatters() {
        return REQUIRE_STABLE_TRANSLATION && warmupRemainingS <= 0.0;
    }

    private boolean rotationMatters() {
        return REQUIRE_STABLE_ROTATION && warmupRemainingS <= 0.0;
    }

    private boolean shooterIsClear() {
        Pose robot = follower != null ? follower.pose() : null;
        return robot != null && !NoFireZones.isInNoFireZone(robot);
    }

    /** True when the chassis translation is within limits (or not being checked). */
    private boolean translationOk() {
        return !translationMatters() || measuredSpeedInS <= MAX_TRANSLATION_SPEED_IN_S;
    }

    /** True when the chassis rotation is within limits (or not being checked). */
    private boolean rotationOk() {
        return !rotationMatters() || Math.abs(measuredYawDps) <= MAX_ROTATION_SPEED_DPS;
    }

    /**
     * The full readiness AND, using the HYSTERESIS forms of the two members
     * that were measured to chatter (wheel band, aim band). This is what the
     * fire policy and the dashboard both use, so what the driver reads on the
     * screen is exactly what drove the transfer.
     */
    private boolean readyNow() {
        return evaluateReadiness().all;
    }

    /** Emits readiness telemetry lines and names the first blocking check. */
    private void reportReadinessDetails(Readiness r, boolean ready) {
        reportReadinessDetails(r, ready, false, false);
    }

    private void reportReadinessDetails(Readiness r, boolean ready, boolean manualFwd, boolean manualRev) {
        String policy;
        if (manualRev) {
            policy = "REVERSE (manual)";
        } else if (manualFwd) {
            policy = "FIRE (manual)";
        } else if (ready) {
            policy = "FIRE (ready)";
        } else {
            policy = "WAIT (not ready)";
        }
        RobotMain.DashTelemetry.addData("Fire policy", policy);
        RobotMain.DashTelemetry.addData("Ready to shoot", r.all);
        RobotMain.DashTelemetry.addData("Fire block", (ready || manualFwd || manualRev) ? "none"
                : (fireSuppressedByDriver ? "driver override (manual transfer)" : firstBlocker(r)));
        // Per-member flags and the live numbers behind them.
        RobotMain.DashTelemetry.addData("  ok: flywheel", flag(r.flywheel) + flywheelText());
        RobotMain.DashTelemetry.addData("  ok: turret pos", flag(r.turretPos) + turretPosText());
        RobotMain.DashTelemetry.addData("  ok: aim", flag(r.aim) + aimText());
        RobotMain.DashTelemetry.addData("  ok: gate", flag(r.gate) + gateText());
        RobotMain.DashTelemetry.addData("  ok: translation", flag(r.translation)
                + String.format(java.util.Locale.US, " %.2f/%.2f in/s", measuredSpeedInS, MAX_TRANSLATION_SPEED_IN_S));
        RobotMain.DashTelemetry.addData("  ok: rotation", flag(r.rotation)
                + String.format(java.util.Locale.US, " %.0f/%.0f dps", Math.abs(measuredYawDps), MAX_ROTATION_SPEED_DPS));
        RobotMain.DashTelemetry.addData("  ok: clear", flag(r.clear));
        RobotMain.DashTelemetry.addData("  ok: hood", flag(r.hood) + hoodText());
        RobotMain.DashTelemetry.addData("Fire transfer", transferState);
    }

    private void reportManualReadinessDetails(boolean flywheelReady, boolean gateOpenNow, boolean ready, boolean autoReady, boolean manualFwd, boolean manualRev) {
        String policy;
        if (manualRev) {
            policy = "REVERSE (manual)";
        } else if (manualFwd) {
            policy = "FIRE (manual)";
        } else if (autoReady) {
            policy = "FIRE (manual ready)";
        } else {
            policy = "WAIT (spooling)";
        }
        RobotMain.DashTelemetry.addData("Fire policy", policy);
        RobotMain.DashTelemetry.addData("Ready to shoot", ready);
        RobotMain.DashTelemetry.addData("Fire block", (autoReady || manualFwd || manualRev) ? "none"
                : (fireSuppressedByDriver ? "driver override (manual transfer)" : (ready ? "none" : (flywheelReady ? "gate (opening)" : "flywheel"))));
        RobotMain.DashTelemetry.addData("  ok: flywheel", flag(flywheelReady) + flywheelText());
        RobotMain.DashTelemetry.addData("  ok: gate", flag(gateOpenNow) + gateText());
        RobotMain.DashTelemetry.addData("  turret: MANUAL (not moving)", "0");
        RobotMain.DashTelemetry.addData("Fire transfer", transferState);
    }

    private static String flag(boolean ok) {
        return ok ? "OK  " : "NO  ";
    }

    /** Explain the hood member: missing hardware is called out explicitly, not passed off as OK. */
    private String hoodText() {
        if (hood == null) {
            return " (no hood)";
        }
        return String.format(java.util.Locale.US, " %.1f/%.1f deg (err %.1f, tol %.1f)",
                hood.getCurrentDegrees(), hood.getTargetDegrees(),
                Math.abs(hood.getCurrentDegrees() - hood.getTargetDegrees()),
                Hood.TOLERANCE_DEG);
    }

    /** Explain the gate member status for telemetry. */
    private String gateText() {
        if (PosServ == null) {
            return " (no gate)";
        }
        if (PosServ.isOpen()) {
            return " OPEN";
        }
        return PosServ.isClosed() ? " closed" : " moving";
    }

    private String flywheelText() {
        double tgt = commandedRpm;
        if (Double.isNaN(tgt)) {
            return " (no RPM commanded)";
        }
        double act = flywheel == null ? 0.0 : flywheel.getSmoothedRpm();
        return String.format(java.util.Locale.US, " %.0f/%.0f RPM (err %.0f, on<=%.0f off>%.0f, dwell %.2fs)",
                act, tgt, Math.abs(act - tgt),
                FLYWHEEL_READY_ON_RPM,
                Math.max(FLYWHEEL_READY_OFF_RPM, FLYWHEEL_READY_ON_RPM),
                FLYWHEEL_SETTLE_DWELL_S);
    }

    private String turretPosText() {
        if (turret == null) {
            return " (no turret)";
        }
        return String.format(java.util.Locale.US, " (%s %.1f/%.1f)",
                turret.State, turret.getPosition(), turret.getSetpoint());
    }

    private String aimText() {
        if (aim == null) {
            return " (no aim model)";
        }
        double err = safeAimErr();
        double tol = aim.activeToleranceDeg;
        return String.format(java.util.Locale.US, " err %.1f / tol %.1f deg", err, tol);
    }

    /**
     * The first member (in evaluation order) that is stopping the shot — one
     * word the driver can read at a glance. Gate is checked last so upstream
     * shooter blockers show before "GATE (opening)".
     */
    private String firstBlocker(Readiness r) {
        if (!r.flywheel) {
            return "FLYWHEEL" + flywheelText();
        }
        if (!r.turretPos) {
            return "TURRET POS" + turretPosText();
        }
        if (!r.aim) {
            return "AIM" + aimText();
        }
        if (!r.translation) {
            return String.format(java.util.Locale.US, "DRIVE (%.2f in/s)", measuredSpeedInS);
        }
        if (!r.rotation) {
            return String.format(java.util.Locale.US, "ROTATE (%.0f dps)", measuredYawDps);
        }
        if (!r.clear) {
            return "NO-FIRE ZONE";
        }
        if (!r.hood) {
            return "HOOD" + hoodText();
        }
        if (!r.gate) {
            return "GATE (opening)";
        }
        return "none";
    }

    private double safeAimErr() {
        double e = aim == null ? Double.NaN : aim.activeHeadingErrorDeg;
        return Double.isNaN(e) ? 0.0 : e;
    }

    /** True when the gate's commanded target is POS_B (open). */
    private boolean gateOpen() {
        return PosServ != null && Math.abs(PosServ.getTarget() - ShootGateServo.POS_B)
                <= Math.abs(PosServ.getTarget() - ShootGateServo.POS_A);
    }

    /** Open the shoot gate servo. */
    private void openGate() {
        if (PosServ == null) return;
        if (!gateOpen()) PosServ.goB();
    }

    /** Close the shoot gate servo. */
    private void closeGate() {
        if (PosServ == null) return;
        PosServ.goA();
    }

    // =====================================================================
    //  Motion sensing + per-loop bookkeeping
    // =====================================================================

    /**
     * Updates chassis speed/yaw measurements, warmup countdown, idle aim snapshot,
     * and driver-override timeout. Called from {@link RobotMain#RobotRunPeriodic()}
     * right after {@code follower.update()} and before {@code CommandScheduler.run()}.
     */
    public void updateMotionAndTimers() {
        long nowNs = System.nanoTime();
        double dt = lastPeriodicNs == 0L ? 0.0 : (nowNs - lastPeriodicNs) / 1.0e9;
        dt = Math.min(Math.max(dt, 0.0), 0.25);
        lastPeriodicNs = nowNs;
        warmupRemainingS = Math.max(0.0, warmupRemainingS - dt);

        Velocity vel = null;
        try {
            vel = follower != null ? follower.velocity() : null;
        } catch (Exception ignored) {
        }
        Pose pose = follower != null ? follower.pose() : null;
        if (vel != null && Double.isFinite(vel.vx) && Double.isFinite(vel.vy) && Double.isFinite(vel.omega)) {
            measuredSpeedInS = Math.hypot(vel.vx, vel.vy);
            measuredYawDps = Math.toDegrees(vel.omega);
            if (pose != null) {
                prevPoseX = pose.x();
                prevPoseY = pose.y();
                prevPoseHeading = pose.heading();
                prevPoseNs = nowNs;
            }
        } else if (pose == null) {
            measuredSpeedInS = 0.0;
            measuredYawDps = 0.0;
            prevPoseX = Double.NaN;
            prevPoseNs = 0L;
        } else {
            boolean hadPrev = !Double.isNaN(prevPoseX) && prevPoseNs != 0L;
            double pdt = hadPrev ? (nowNs - prevPoseNs) / 1.0e9 : 0.0;
            if (hadPrev && pdt > 1e-4 && pdt < 0.5) {
                measuredSpeedInS =
                        Math.hypot(pose.x() - prevPoseX, pose.y() - prevPoseY) / pdt;
                double dHeading = pose.heading() - prevPoseHeading;
                while (dHeading > Math.PI) dHeading -= 2.0 * Math.PI;
                while (dHeading < -Math.PI) dHeading += 2.0 * Math.PI;
                measuredYawDps = Math.toDegrees(dHeading) / pdt;
            } else {
                measuredSpeedInS = 0.0;
                measuredYawDps = 0.0;
            }
            prevPoseX = pose.x();
            prevPoseY = pose.y();
            prevPoseHeading = pose.heading();
            prevPoseNs = nowNs;
        }

        if (aim != null && !aim.activeAiming) {
            aim.updateIdleSnapshot();
        }

        if (fireSuppressedByDriver) {
            fireSuppressedForS += dt;
            if (fireSuppressedForS > FIRE_OVERRIDE_HOLD_S) {
                fireSuppressedByDriver = false;
            }
        }
    }

    /**
     * Post-scheduler readiness update: restores manual intake if firing just ended,
     * refreshes {@code lastReadiness} when no fire command ran this tick, and publishes
     * dashboard summary telemetry from the exact same-tick snapshot.
     * Called from {@link RobotMain#RobotRunPeriodic()} right after {@code CommandScheduler.run()}.
     */
    public void readinessPeriodic() {
        restoreManualIntakeWhenFree();

        if (isManualShootActive()) {
            if (!firing) {
                lastManualFlywheelReady = flywheelReadyHyst();
                lastManualGateOpen = gateOk();
                lastManualReady = lastManualFlywheelReady && lastManualGateOpen;
            }
            if (RobotMain.DASHBOARD) {
                RobotMain.DashTelemetry.addData("Shoot Ready", lastManualReady ? "READY (MANUAL)" : "WAIT (MANUAL)");
                RobotMain.DashTelemetry.addData("Shoot Blocker", lastManualReady ? "none" : (lastManualFlywheelReady ? "gate (opening)" : "flywheel"));
                RobotMain.DashTelemetry.addData("Flywheel Ready", flag(lastManualFlywheelReady) + flywheelText());
            }
        } else {
            if (!firing) {
                lastReadiness = evaluateReadiness();
            }
            Readiness r = getLatestReadiness();
            if (RobotMain.DASHBOARD) {
                RobotMain.DashTelemetry.addData("Shoot Ready", r.all ? "READY" : "WAIT");
                RobotMain.DashTelemetry.addData("Shoot Blocker", r.all ? "none" : firstBlocker(r));
                RobotMain.DashTelemetry.addData("Flywheel Ready", flag(r.flywheel) + flywheelText());
            }
        }

        if (RobotMain.DASHBOARD) {
            if (firing) {
                RobotMain.DashTelemetry.addData("Chassis speed (in/s)", "%.2f", measuredSpeedInS);
                RobotMain.DashTelemetry.addData("Chassis yaw (dps)", "%.1f", measuredYawDps);
            }
            if (aim != null && !Double.isNaN(aim.activeDistanceIn)) {
                RobotMain.DashTelemetry.addData("Target dist (in)", "%.1f", aim.activeDistanceIn);
            }
        }
    }

    /** Live distance (in) to the closest target right now (NaN if no pose/target). */
    public double getLiveTargetDistanceInches() {
        return aim != null ? aim.activeDistanceIn : Double.NaN;
    }

    /** How long a driver's manual transfer overrides the auto-fire (s). */
    public static double FIRE_OVERRIDE_HOLD_S = 0.75;

    /** Measured chassis translation speed (in/s) — telemetry. */
    public double getMeasuredSpeedInS() {
        return measuredSpeedInS;
    }

    /** Measured chassis yaw rate (deg/s, +CCW) — telemetry. */
    public double getMeasuredYawDps() {
        return measuredYawDps;
    }

    /** The RPM the RPM manager is commanding right now (NaN when idle). */
    public double getCommandedRpm() {
        return commandedRpm;
    }

    /** True during the post-start grace window where motion checks are skipped. */
    public boolean inMotionWarmup() {
        return warmupRemainingS > 0.0;
    }

    // =====================================================================
    //  Intake hand-off
    // =====================================================================

    /**
     * Stop the driver's intake while the transfer feeds, so the shot is not
     * fighting a stream of new balls. Fire-and-forget ({@link Intake#stop}),
     * and the intake subsystem is NOT a requirement of the fire half — the
     * driver can restart it at any time, and {@link #restoreManualIntakeWhenFree()}
     * hands it back the moment firing stops.
     *
     * <p>Deliberately a plain stop (zero velocity) rather than a brake: the
     * intake's own next {@code holdCmd} re-asserts its RPM, so nothing is
     * latched either way.
     *
     * <p>Gated on {@code intakeStoppedByFire}: the intake is only "ours to
     * stop" once. If the driver re-presses the right trigger mid-feed the
     * restored RPM simply runs again — the fire policy does not fight a held
     * intake, it only clears the way once per firing episode.
     */
    private void holdManualIntakeOutOfTheWay() {
        if (intake == null || intakeStoppedByFire) {
            return;
        }
        // Nothing to get out of the way if it is already stopped.
        if (Math.abs(intake.getActualRpm()) > INTAKE_RUNNING_RPM) {
            intake.stop();
            intakeStoppedByFire = true;
        }
    }

    /**
     * Re-press the intake once firing stops — ONLY if the fire half was the
     * reason it stopped AND the driver still wants it running. Called from
     * {@link #readinessPeriodic()} (once per tick, outside the scheduler) so
     * the hand-back happens even on the loop where the fire half's own
     * {@code end()} just ran.
     */
    private void restoreManualIntakeWhenFree() {
        if (!intakeStoppedByFire || firing) {
            return; // still feeding (or nothing to restore)
        }
        if (intake != null && restoreIntakeRpmOnRelease && manualIntakeHeld) {
            intake.runIntake();
        }
        intakeStoppedByFire = false;
    }

    /**
     * The driver's right-body intake intent, reported by {@code KeyBindings}
     * every loop ({@link #setManualIntakeHeld(boolean)}). The intake itself
     * cannot reveal it: once the fire half stops the roller, "is it spinning"
     * reads exactly like "the driver let go of the trigger".
     */
    private boolean manualIntakeHeld = false;

    /** Tell the factory whether the driver is holding the intake trigger (right trigger). */
    public void setManualIntakeHeld(boolean held) {
        manualIntakeHeld = held;
    }

    /**
     * Driver's manual transfer intent from gamepad (RB = forward, B = reverse).
     * Set by KeyBindings.update() each loop so manual feeding/jam-clear can run
     * during auto-shoot without cancelling the auto-aim or flywheel setpoint.
     */
    private boolean manualTransferForwardHeld = false;
    private boolean manualTransferReverseHeld = false;

    public void setManualTransferForwardHeld(boolean held) {
        manualTransferForwardHeld = held;
    }

    public void setManualTransferReverseHeld(boolean held) {
        manualTransferReverseHeld = held;
    }

    public boolean isManualTransferForwardHeld() {
        return manualTransferForwardHeld;
    }

    public boolean isManualTransferReverseHeld() {
        return manualTransferReverseHeld;
    }

    /** An intake spinning above this (RPM) is considered "running". */
    public static double INTAKE_RUNNING_RPM = 50.0;

    /**
     * When true (default) the intake resumes at its configured RPM as soon as
     * the auto-fire stops. Set false if the driver would rather the intake
     * stay stopped until the right trigger is pressed again.
     */
    public static boolean restoreIntakeRpmOnRelease = true;

    /**
     * Two-threshold (Schmitt-trigger) latch: turns ON when the value drops to
     * or below {@code on}, turns OFF only once it rises above {@code off},
     * where {@code off &gt; on}. Used for the aim-alignment check so a signal
     * sitting right on its threshold cannot flip the shot every loop.
     */
    private static final class Hysteresis {
        private boolean latched = false;

        /** Force the latch state (used to reset per firing session). */
        void reset(boolean state) {
            latched = state;
        }

        /** Set the latch directly (no thresholds). */
        void latch(boolean state) {
            latched = state;
        }

        /** True while the value is inside the on-band, until it leaves the off-band. */
        boolean update(double value, double on, double off) {
            if (latched) {
                if (value > off) latched = false;
            } else if (value <= on) {
                latched = true;
            }
            return latched;
        }

        boolean isLatched() {
            return latched;
        }
    }

    /**
     * Linear LUT lookup, edge-clamped to the endpoints. Reads the tunable
     * arrays live, so Panels edits apply on the very next lookup (no rebuild).
     */
    private static double lutInterpolate(double distIn, double[] ys, double fallback) {
        double[] xs = LUT_DISTANCES_IN;
        int n = Math.min(xs.length, ys.length);
        if (n < 2) {
            return ys.length > 0 ? ys[0] : fallback;
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

    private static double lutRpm(double distIn) {
        return lutInterpolate(distIn, LUT_RPMS, 0.0);
    }

    /** The Hood angle the manager is commanding right now (NaN when idle). */
    public double getCommandedHoodDeg() {
        return commandedHoodDeg;
    }

    /**
     * Linear LUT lookup for hood angle, edge-clamped to endpoints.
     * Reads the tunable arrays live, so Panels edits apply on the very next lookup.
     */
    public static double lutHoodDeg(double distIn) {
        return lutInterpolate(distIn, LUT_HOOD_DEGS, Hood.HOOD_DOWN_DEG);
    }

    /**
     * Linear LUT lookup for ball time-of-flight (seconds), edge-clamped to endpoints.
     * Reads {@link #LUT_TOF_S} live for shoot-on-the-move lead.
     */
    public static double lutTofSec(double distIn) {
        return lutInterpolate(distIn, LUT_TOF_S, 0.5);
    }
}
