package org.firstinspires.ftc.teamcode.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.button.Trigger;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ShootGateServo;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.turret.TurretPivot;
import org.firstinspires.ftc.teamcode.WattageLib.Ballistics.NoFireZones;

/**
 * Multi-subsystem compositions only. Single-subsystem commands live on their
 * subsystem ({@code intake.runCmd()/holdCmd()/stopCmd()},
 * {@code transfer.runCmd()/holdCmd()/reverseCmd()/reverseHoldCmd()/stopCmd()},
 * {@code flywheel.toRpm()/holdTunable()/stopCmd()/coastToIdleCmd()},
 * {@code PosServ.toggleCmd()/toPos()/nudgeCmd()}).
 *
 * <h2>Readiness triggers + auto-transfer (ported from 2026-IndianaJoe)</h2>
 *
 * IndianaJoe's {@code SystemController} auto-fired: while the driver's "ready
 * to shoot" latch was on it ran {@code FeedAndKick()} whenever
 * {@code isReadyToShoot()} was true, and that trigger was an AND of checks:
 *
 * <pre>
 *   isReadyToShoot = isFlywheelAtSpeed()   // RPM at the LUT target (debounced)
 *                   &amp;&amp; isHoodAtAngle()     // pointing mechanism at position
 *                   &amp;&amp; isTurretOnTarget()  // turret aligned (distance-scaled tol)
 *                   &amp;&amp; isDrivetrainStable()// XY speed under a limit
 *                   &amp;&amp; isRotationStable()  // |yaw rate| under a limit
 *                   &amp;&amp; isShooterClear();   // shooter not in a no-fire zone
 * </pre>
 *
 * This class ports that set. Every member is a plain {@link Trigger} over LIVE
 * state (the shared {@link AimAtGoal}, {@link Flywheel}, the follower pose), so
 * any loop can sample them — they do not depend on an aim command running.
 *
 * <table summary="FRC readiness trigger -> FTC port">
 *   <tr><th>FRC (SystemController)</th><th>Biobuzz (this class)</th></tr>
 *   <tr><td>{@code isFlywheelAtSpeed()}</td><td>{@link #isFlywheelAtSpeed()} /
 *       {@link #isFlywheelStable()}</td></tr>
 *   <tr><td>{@code isHoodAtAngle()}</td><td>{@link #isTurretAtSetpoint()}
 *       (this robot has no hood; the LUT RPM is the "hood" variable)</td></tr>
 *   <tr><td>{@code isTurretOnTarget()}</td><td>{@link #isTurretOnTarget()}</td></tr>
 *   <tr><td>{@code isDrivetrainStable()}</td><td>{@link #isDrivetrainStable()}</td></tr>
 *   <tr><td>{@code isRotationStable()}</td><td>{@link #isRotationStable()}</td></tr>
 *   <tr><td>{@code isShooterClear()}</td><td>{@link #isShooterClear()}</td></tr>
 *   <tr><td>{@code isReadyToShoot()}</td><td>{@link #isReadyToShoot()}</td></tr>
 * </table>
 *
 * <p><b>The shot is now two halves</b> (IndianaJoe's split):
 * <ul>
 *   <li>{@link #TrackTarget()} — turret aim + the RPM LUT. Latched ON,
 *       runs ready or not.</li>
 *   <li>{@link #FireAtGoal()} — gate + transfer, feeding only while
 *       {@link #isReadyToShoot()} permits.</li>
 *   <li>{@link #AutoShootAtGoal()} — both, which is what the left trigger
 *       latch now runs ({@link #ShootAtGoal()} keeps its old name).</li>
 * </ul>
 * Readiness exists to protect the BALL, not the flywheel: the ball only leaves
 * through the transfer roller, so feeding while the wheel is off-RPM or the
 * turret is off target either jams the ball in the barrel or fires wherever it
 * happens to be pointed (and, inside a no-fire zone, at something we must not
 * hit).
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
    public static double[] LUT_RPMS = {500, 1950, 1800, 1700, 1600, 1500};//max observed RPM is 4100;

    /**
     * Max slew of the COMMANDED flywheel setpoint, RPM/sec. The raw LUT output
     * can step for two reasons that are NOT the LUT's shape: the aim lock hops
     * to a different hive target (distance jumps by the target spacing) and
     * fused-pose noise moves the distance every loop. This rate-limits how
     * fast the commanded RPM chases the raw lookup, so the setpoint glides.
     * 0 or negative = instant (slew off).
     */
    public static double LUT_MAX_RPM_SLEW = 1200.0;

    /**
     * Aimed-distance deadband (in): while the distance stays within this of the
     * last accepted value, the previous value is reused, so pose/LUT noise
     * cannot walk the flywheel setpoint around. The 20261025-005022 capture
     * shows the distance moving only 45.8..47.6 in while still, yet the LUT
     * target crawled 1827 -> 1805. 1.0 in ≈ 2 RPM on the current table.
     */
    public static double LUT_DISTANCE_DEADBAND_IN = 1.0;

    /**
     * Low-pass weight on the LUT RPM output per loop (0..1; 1 = no filtering).
     * Complements the slew limit: slew bounds the RATE, this bounds the STEP.
     */
    public static double LUT_RPM_FILTER_ALPHA = 0.25;

    /** Last slew-limited RPM the RPM manager commanded (NaN until the first lock). */
    private double lastCmdRpm = Double.NaN;
    private long lastRpmNs = 0L;
    /** The RPM the RPM manager is commanding right now (NaN when idle). */
    private double commandedRpm = Double.NaN;
    /** Last accepted LUT distance (deadband state) and the filtered LUT RPM. */
    private double lastLutDist = Double.NaN;
    private double lutFilteredRpm = Double.NaN;

    public final Follower follower;
    public final Intake intake;
    public final Transfer transfer;
    public final Flywheel flywheel;
    public final ShootGateServo PosServ;
    public final TurretPivot turret;
    public final HardwareMap hardwareMap;

    /**
     * The one alliance-aware aim/ballistics instance shared by every shoot
     * command AND every readiness trigger — so the turret tracking, the RPM
     * LUT and the "on target" check can never disagree about which point is
     * being shot at. Null when the turret is absent from the RC config.
     */
    public final AimAtGoal aim;

    // =====================================================================
    //  READINESS THRESHOLDS (Panels -> CommandFactory)
    // =====================================================================

    /**
     * Max chassis TRANSLATION speed (in/s) for {@link #isDrivetrainStable()}.
     * FRC analog: {@code ballisticsConstants.StableTranslationSpeedMaxMps}
     * (0.05 m/s ≈ 2 in/s); Pedro exposes a pose but no velocity vector, so the
     * speed is measured from the pose delta (see {@link #readinessPeriodic()}).
     */
    public static double MAX_TRANSLATION_SPEED_IN_S = 2.0;

    /**
     * Max chassis |yaw rate| (deg/s) for {@link #isRotationStable()}.
     * FRC analog: {@code ballisticsConstants.StablerotationSpeedMaxDegreespersecond}.
     * A shot taken while the chassis is still spinning keeps angular momentum
     * that spends the trajectory's spin early, so the wheel/turret can be
     * perfectly aimed and the shot still misses by the tail swing.
     */
    public static double MAX_ROTATION_SPEED_DPS = 60.0;

    /**
     * Falling-edge debounce for {@link #isReadyToShoot()} and
     * {@link #isFlywheelStable()} (s). FRC used 0.25 s
     * ({@code shootReadyDebounceS}) on the flywheel alone so a wheel dip could
     * not stop the feed instantly. Rising edges are immediate.
     */
    public static double READY_DEBOUNCE_S = 0.15;

    // ---- Flywheel readiness band (hysteresis) ----
    /**
     * Enter "at speed" when |actual − target| is within this (RPM).
     *
     * <p>Sized against the MEASURED quantisation, not a guess: the 5203's
     * built-in encoder gives 28 ticks/rev, so at a ~50 ms loop one tick is
     * ~43 RPM, and the 20261025-005022 capture shows readings stepping
     * 1671.4 / 1714.3 / 1757.1 / 1800.0. The previous single band (75 RPM,
     * {@link Flywheel#AT_SPEED_TOLERANCE_RPM}) was under two ticks wide, so
     * normal PID hunting flipped readiness. 120 RPM ≈ 2.8 ticks — enter only
     * once the wheel is solidly there.
     */
    public static double FLYWHEEL_READY_ON_RPM = 120.0;
    /**
     * Leave "at speed" only once |actual − target| exceeds this (RPM). Wider
     * than the enter band on purpose: a shot in progress is not abandoned for
     * a one-tick reading. 220 RPM ≈ 5 ticks.
     */
    public static double FLYWHEEL_READY_OFF_RPM = 220.0;

    /**
     * Aim-alignment hysteresis: once aligned, the turret may drift this many
     * degrees OUTSIDE the live tolerance before the shot is pulled. Keeps a
     * settled turret (and a shot in progress) from being cancelled by the
     * turret PID's own deadband.
     */
    public static double AIM_HYST_EXIT_MARGIN_DEG = 1.5;

    /** Include the chassis-translation check in {@link #isReadyToShoot()} (FRC always did). */
    public static boolean REQUIRE_STABLE_TRANSLATION = true;
    /** Include the chassis-rotation check in {@link #isReadyToShoot()} (FRC always did). */
    public static boolean REQUIRE_STABLE_ROTATION = true;
    /**
     * SECONDS after an OpMode starts during which the pose-derived motion
     * checks are skipped, so the first (delta-less) sample can never pin the
     * fire off. 0 disables the grace.
     */
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

    /**
     * Falling-edge debouncers (SolversLib 0.3.6's {@code Trigger} has no
     * {@code debounce()} and WPILib's {@code Debouncer} is not on the classpath,
     * so this class carries its own).
     */
    /** Falling-edge debounce on the whole readiness AND ({@link #READY_DEBOUNCE_S}). */
    private final FallingEdgeDebounce readyDebounce = new FallingEdgeDebounce(READY_DEBOUNCE_S);

    /** Hysteresis latches for the two readiness members that were measured to chatter. */
    private final Hysteresis alignHyst = new Hysteresis();
    private boolean flywheelBandLatched = false;

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

    public CommandFactory(Follower _follower, Intake _intake, Transfer _transfer, Flywheel _flywheel, ShootGateServo _posServ, TurretPivot _turret, HardwareMap _hardwareMap) {
        follower = _follower;
        intake = _intake;
        transfer = _transfer;
        flywheel = _flywheel;
        PosServ = _posServ;
        turret = _turret;
        hardwareMap = _hardwareMap;

        // The shared aim model (alliance points + shot-aware tolerance).
        aim = turret == null ? null
                : new AimAtGoal(turret, follower, hardwareMap,
                        () -> RobotMain.CurrentAlliance, this::getCommandedRpm);

        // Build the single latched instance LAST — its halves need `aim`.
        autoShootCmd = aim == null ? null : buildAutoShoot();
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
        return new AimAtGoal(turret, follower, hardwareMap,
                () -> RobotMain.CurrentAlliance, this::getCommandedRpm);
    }

    /**
     * The TRACK half of the shot — the left trigger's latched state: aim the
     * turret at the closest alliance target AND hold the flywheel at the
     * distance-to-hive LUT RPM. Never opens the gate and never feeds; that is
     * {@link #FireAtGoal()}'s job, which runs alongside it gated on
     * {@link #isReadyToShoot()}.
     *
     * <p>Requirements: the shared aim instance (turret) + the RPM manager
     * (flywheel) — a manual turret spin or flywheel command interrupts it.
     */
    public Command TrackTarget() {
        if (aim == null) {
            return Commands.none();
        }
        return new ParallelCommandGroup(
                // 1) Aim: spins the TURRET to the target; chassis stays with the driver.
                aim,
                // 2) RPM manager: distance -> linear LUT -> slew-limited setpoint.
                rpmManagerCommand()
        ).setName("TrackTarget");
    }

    /**
     * The FIRE half of the shot: opens the gate and runs the TRANSFER whenever
     * {@link #isReadyToShoot()} is true, and closes the gate and stops the
     * transfer the moment it is not. Runs until cancelled (latched).
     *
     * <p>The gate is therefore the hardware interlock and this policy is what
     * drives it: a ball may only reach the flywheel while the whole shot —
     * RPM, turret position, turret alignment, gate, drivetrain, no-fire zones —
     * is good. Holding the transfer is what makes this robot conveyor-driven:
     * the ball goes from hopper to flywheel only because the transfer runs.
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
                    // Start the alignment latch from the current alignment
                    // state, so a first ready tick can fire immediately
                    // without having to re-prove alignment.
                    alignHyst.reset(aimAlignedHyst());
                    transferState = FeedState.OFF;
                },
                () -> {
                    // ONE evaluation pass per loop: the hysteresis members are
                    // stateful, so evaluating them twice (once for the policy,
                    // once for the display) would consume two Schmitt steps and
                    // make the telemetry not match what was decided.
                    Readiness r = evaluateReadiness();
                    boolean ready = !fireSuppressedByDriver && r.all;
                    if (ready) {
                        openGate();
                        holdManualIntakeOutOfTheWay();
                        // Re-assert runForward every loop: it re-reads
                        // TRANSFER_RPM, so live Panels retunes apply mid-shot.
                        if (transfer != null) transfer.runForward();
                        setTransferState(FeedState.FEEDING);
                    } else {
                        closeGate();
                        // Stop ONCE on the falling edge, not every loop.
                        if (transfer != null && transferState != FeedState.OFF) transfer.stop();
                        setTransferState(FeedState.OFF);
                    }
                    // Explain the verdict: WHICH member is holding the shot.
                    reportReadinessDetails(r, ready);
                },
                (interrupted) -> {
                    firing = false;
                    closeGate(); // no-op until the gate exists
                    if (transfer != null) transfer.stop();
                    setTransferState(FeedState.OFF);
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
     * The whole auto-firing shot the driver latches ON: {@link #TrackTarget()}
     * plus {@link #FireAtGoal()}. This is "auto aim auto-runs the transfer when
     * ready", i.e. IndianaJoe's
     * {@code isReadyToShoot().whileTrue(FeedAndKick())} with the aim command
     * already running — the transfer fires itself, no second button.
     */
    public Command AutoShootAtGoal() {
        return autoShootCmd == null ? Commands.none() : autoShootCmd;
    }

    /** Builds the latched compound (called once, from the constructor). */
    private Command buildAutoShoot() {
        return new ParallelCommandGroup(
                TrackTarget(),
                FireAtGoal()
        ).setName("AutoShootAtGoal");
    }

    /**
     * Hold-to-shoot: SPINS THE TURRET to aim at the closest hive target AND
     * spins the flywheel at an RPM looked up from the distance-to-hive LUT.
     * The transfer feeds (and the gate opens) only while
     * {@link #isReadyToShoot()} — the whole shot — is satisfied, and stops the
     * moment it is not. On release (or any interrupt) the gate closes, the
     * transfer stops, the flywheel coasts down to {@code IDLE_RPM} (soft FLOAT
     * slowdown; the PID re-engages to hold idle on arrival, and holds there
     * until the next shoot), and the aim member returns the turret motor to
     * NEUTRAL.
     *
     * <p>Per loop while held: aim re-picks the closest target, the distance to
     * it is fed through the linear LUT and SLEW-LIMITED (see
     * {@link #LUT_MAX_RPM_SLEW}) before reaching the flywheel setpoint - so
     * aim-lock hops between targets and pose noise glide instead of stepping.
     * The gate is then driven EVERY loop by the full readiness AND — flywheel
     * at speed, turret at its setpoint and on target, gate able, chassis
     * translation/rotation stable, and outside all no-fire zones (ported from
     * IndianaJoe — {@code Ballistics.NoFireZones}). Entering a no-fire zone
     * never stops the flywheel - it just closes the gate so no ball leaves
     * while unsafe.
     *
     * <p>KEPT as the historical name; it now returns
     * {@link #AutoShootAtGoal()}, i.e. the fully readiness-gated auto-fire
     * (aim + RPM + gate + TRANSFER, feeding only when ready). The gate-only
     * half alone is {@link #TrackTarget()} plus {@link #FireAtGoal()}.
     *
     * <p>Requirements are the union of the halves: turret (aim), flywheel
     * (RPM manager), gate servo and transfer. A manual turret spin (dpad),
     * servo toggle (X), nudge or transfer command (bumpers) taken mid-shot
     * interrupts the whole compound (manual override).
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
     * The LEFT BUMPER "unlock / panic" command: turn the auto-shoot latch OFF
     * and put the shooter in its safe rest state.
     *
     * <p>It cancels {@link #autoShootCmd} specifically — NOT whatever happens
     * to be scheduled — because {@code KeyBindings} toggles that exact
     * instance with {@code toggleWhenActive}; cancelling it is what keeps the
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
        if (autoShootCmd == null) {
            return Commands.none();
        }
        return new FunctionalCommand(
                () -> {
                    if (autoShootCmd.isScheduled()) {
                        autoShootCmd.cancel(); // runs both halves' end()
                    }
                    // Belt-and-suspenders: whatever end() did, make the safe
                    // resting state explicit.
                    closeGate(); // no-op until the gate exists
                    if (transfer != null) transfer.stop();
                    if (flywheel != null) flywheel.coastToIdle();
                    if (turret != null) turret.BRAKE();
                    firing = false;
                    intakeStoppedByFire = false;
                    fireSuppressedByDriver = false; // a deliberate unlock re-arms for next time
                    RobotMain.DashTelemetry.addData("Shoot latch (LT toggle)", "OFF (LB unlock)");
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
                },
                () -> {
                    double d = currentShotDistanceInches();
                    if (Double.isNaN(d) || d <= 0) {
                        return; // no lock yet: hold the last setpoint
                    }
                    double raw = lutRpm(d);
                    // Distance deadband: a sub-inch wobble in the aimed distance
                    // must not move the setpoint at all. The capture shows the
                    // LUT target crawling 1827 -> 1805 purely from distance
                    // noise (45.8..47.6 in) while the robot was still, and every
                    // step re-opened the wheel's at-speed question.
                    long nowNs = System.nanoTime();
                    double dt = lastRpmNs == 0L ? 0.02
                            : Math.min(Math.max((nowNs - lastRpmNs) / 1.0e9, 0.0), 0.25);
                    lastRpmNs = nowNs;
                    if (Double.isNaN(lastLutDist)
                            || Math.abs(d - lastLutDist) > LUT_DISTANCE_DEADBAND_IN) {
                        lastLutDist = d;
                    } else {
                        d = lastLutDist; // hold the previous distance inside the deadband
                    }
                    // Low-pass the LUT output too, so a genuine distance change
                    // still arrives smoothly rather than as LUT interpolation steps.
                    if (Double.isNaN(lutFilteredRpm)) {
                        lutFilteredRpm = raw;
                    } else {
                        double a = Math.max(0.0, Math.min(1.0, LUT_RPM_FILTER_ALPHA));
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
                    // addData only — flushed once per loop by RobotMain.
                    RobotMain.DashTelemetry.addData("Shoot LUT raw RPM", "%.0f", raw);
                    RobotMain.DashTelemetry.addData("Shoot LUT cmd RPM", "%.0f", lastCmdRpm);
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

    /** The distance (in) from the shooter to the target the shot is being made at. */
    private double currentShotDistanceInches() {
        if (aim == null) {
            return Double.NaN;
        }
        double locked = aim.getLastDistanceInches();
        return Double.isNaN(locked) ? aim.liveDistanceInches() : locked;
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
        return new Trigger(() -> aim != null && aim.isLiveOnTarget());
    }

    /** True when the TURRET has reached a commanded setpoint (FRC {@code isHoodAtAngle()} slot). */
    public Trigger isTurretAtSetpoint() {
        return new Trigger(this::turretAtSetpoint);
    }

    /**
     * True when the gate is closed (or the gate is not implemented / absent).
     * NOT "settled" — see {@link #gateOk()} for why a settled test oscillates.
     */
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

    /**
     * True when ALL readiness members are simultaneously satisfied — the
     * FTC port of {@code SystemController.isReadyToShoot()}. Debounced on the
     * falling edge by {@link #READY_DEBOUNCE_S}.
     *
     * <p>NOTE: this re-evaluates the stateful hysteresis members. It is safe to
     * bind and poll (one evaluation per poll), but the FIRE POLICY does not use
     * it — {@link #FireAtGoal()} evaluates once and passes that snapshot to the
     * dashboard, so the two always agree.
     */
    public Trigger isReadyToShoot() {
        return new Trigger(() -> readyDebounce.update(evaluateReadiness().all));
    }

    /**
     * One immutable snapshot of every readiness member for THIS loop.
     *
     * <p>Introduced because the hint members are STATEFUL (the flywheel band and
     * the aim latch are Schmitt triggers). Evaluating them once and sharing the
     * result is what makes the dashboard truthful — the old code called them
     * separately for the policy and for the display, which both consumed extra
     * hysteresis steps and could print a state different from the one acted on.
     */
    private static final class Readiness {
        final boolean flywheel;
        final boolean turretPos;
        final boolean aim;
        final boolean gate;
        final boolean translation;
        final boolean rotation;
        final boolean clear;
        final boolean all;

        Readiness(boolean flywheel, boolean turretPos, boolean aim, boolean gate,
                  boolean translation, boolean rotation, boolean clear) {
            this.flywheel = flywheel;
            this.turretPos = turretPos;
            this.aim = aim;
            this.gate = gate;
            this.translation = translation;
            this.rotation = rotation;
            this.clear = clear;
            this.all = flywheel && turretPos && aim && gate && translation && rotation && clear;
        }
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
                shooterIsClear());
    }

    /**
     * The gate member of the readiness set.
     *
     * <p><b>Not implemented -&gt; always OK.</b> Until the gate is physically
     * built ({@link ShootGateServo#IMPLEMENTED}) there is nothing to wait for,
     * and demanding a settled gate would wedge the shot on a mechanism the
     * robot does not have.
     *
     * <p><b>Implemented -&gt; the gate must be CLOSED, not "settled".</b> This
     * is the fix for the oscillation seen in capture 20261025-010811: the old
     * test asked {@link ShootGateServo#isAtPosition()} ("has the slew ramp
     * finished?"), but opening the gate is exactly what the shot does, so
     * readiness necessarily failed while the gate travelled, which closed it,
     * which made readiness true again — roughly 5 Hz. "Closed" is stable
     * across the whole firing sequence because the fire half only opens the
     * gate AFTER readiness has passed.
     */
    private boolean gateOk() {
        if (PosServ == null || !ShootGateServo.IMPLEMENTED) {
            return true;
        }
        return PosServ.isClosed();
    }

    // ---- raw predicates the triggers read ----

    private boolean flywheelAtSpeed() {
        return flywheel != null
                && flywheel.isHoldingTarget()   // a COAST/STOP would otherwise read "at 0"
                && flywheel.isAtTargetRpm();
    }

    /**
     * Flywheel "at speed" with HYSTERESIS, as a band on the actual-vs-target
     * RPM error. This is the fix for the measured on/off chatter.
     *
     * <p><b>Why a single band was not enough.</b> The flywheel's built-in
     * encoder is 28 ticks per revolution, so at a ~50 ms loop ONE encoder tick
     * is ~43 RPM — and the captured samples show exactly that quantisation:
     * 1671.4 / 1714.3 / 1757.1 / 1800.0 (43 RPM apart). Meanwhile
     * {@link Flywheel#AT_SPEED_TOLERANCE_RPM} is 75 RPM, i.e. barely two
     * quantisation steps. The wheel hunting inside its PID deadband therefore
     * stepped in and out of a single threshold every few loops with almost no
     * true speed change, and every exit stopped the transfer.
     *
     * <p>The band is now split: enter at {@link #FLYWHEEL_READY_ON_RPM} of
     * error, leave only past {@link #FLYWHEEL_READY_OFF_RPM}. Requiring the
     * wheel to be genuinely further out to abandon a shot is the same
     * hysteresis the FRC {@code shootReadyDebounceS} was standing in for.
     * Below the reference RPM the band is skipped entirely (a slow wheel on the
     * way up must still be caught by the plain test).
     */
    private boolean flywheelReadyHyst() {
        if (flywheel == null || !flywheel.isHoldingTarget()) {
            flywheelBandLatched = false;
            return false;
        }
        double target = commandedRpm;
        if (Double.isNaN(target) || target <= 0) {
            // Nothing commanded yet (e.g. before the first LUT lock).
            flywheelBandLatched = false;
            return false;
        }
        double err = Math.abs(flywheel.getActualRpm() - target);
        double bandOn = Math.max(FLYWHEEL_READY_ON_RPM, Flywheel.AT_SPEED_TOLERANCE_RPM);
        double bandOff = Math.max(bandOn, FLYWHEEL_READY_OFF_RPM);
        if (flywheelBandLatched) {
            if (err > bandOff) flywheelBandLatched = false;
        } else if (err <= bandOn) {
            flywheelBandLatched = true;
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
        double err = Math.abs(aim == null ? Double.NaN : aim.liveHeadingErrorDeg());
        if (Double.isNaN(err)) {
            alignHyst.latch(false);
            return false;
        }
        double on = aim == null ? Double.NaN : aim.aimToleranceDeg(aim.liveDistanceInches());
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

    /**
     * Emit the readiness lines EVERY loop the fire half runs, and name the
     * blocking member.
     *
     * <p>This is the "why is it not firing" answer that was missing: the old
     * {@code Fire policy} line only said ready/not-ready, and the
     * {@code Ready to shoot} line was computed a tick earlier from
     * {@code readinessPeriodic()} — so the 20261025-005022 capture shows
     * {@code FIRE (ready)} next to {@code Ready to shoot: false}. Both now come
     * from ONE {@link Readiness} snapshot per loop, so they cannot disagree.
     */
    private void reportReadinessDetails(Readiness r, boolean ready) {
        RobotMain.DashTelemetry.addData("Fire policy", ready ? "FIRE (ready)" : "WAIT (not ready)");
        RobotMain.DashTelemetry.addData("Ready to shoot", r.all);
        RobotMain.DashTelemetry.addData("Fire block", ready ? "none"
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
        RobotMain.DashTelemetry.addData("Fire transfer", transferState);
    }

    private static String flag(boolean ok) {
        return ok ? "OK  " : "NO  ";
    }

    /** Explain the gate member: missing hardware is called out explicitly, not passed off as OK. */
    private String gateText() {
        if (PosServ == null) {
            return " (no gate)";
        }
        if (!ShootGateServo.IMPLEMENTED) {
            return " (NOT IMPLEMENTED - ignored)";
        }
        return String.format(java.util.Locale.US, " %s", PosServ.isClosed() ? "closed" : "OPEN");
    }

    private String flywheelText() {
        double tgt = commandedRpm;
        if (Double.isNaN(tgt)) {
            return " (no RPM commanded)";
        }
        double act = flywheel == null ? 0.0 : flywheel.getActualRpm();
        return String.format(java.util.Locale.US, " %.0f/%.0f RPM (err %.0f, on<=%.0f off>%.0f)",
                act, tgt, Math.abs(act - tgt),
                Math.max(FLYWHEEL_READY_ON_RPM, Flywheel.AT_SPEED_TOLERANCE_RPM),
                Math.max(FLYWHEEL_READY_OFF_RPM, Math.max(FLYWHEEL_READY_ON_RPM, Flywheel.AT_SPEED_TOLERANCE_RPM)));
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
        double tol = aim.aimToleranceDeg(aim.liveDistanceInches());
        return String.format(java.util.Locale.US, " err %.1f / tol %.1f deg", err, tol);
    }

    /**
     * The first member (in evaluation order) that is stopping the shot — one
     * word the driver can read at a glance. Same order as {@link #readyNow()},
     * read from the snapshot so it names the member that actually blocked.
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
        if (!r.gate) {
            return "GATE (moving)";
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
        return "none";
    }

    private double safeAimErr() {
        double e = aim == null ? Double.NaN : aim.liveHeadingErrorDeg();
        return Double.isNaN(e) ? 0.0 : e;
    }

    /** True when the gate's commanded target is POS_B (open). */
    private boolean gateOpen() {
        return PosServ != null && Math.abs(PosServ.getTarget() - ShootGateServo.POS_B)
                <= Math.abs(PosServ.getTarget() - ShootGateServo.POS_A);
    }

    /**
     * Open the gate — ONLY when the mechanism exists. While
     * {@link ShootGateServo#IMPLEMENTED} is false this is a no-op, so the
     * unimplemented servo is never written and never sits mid-slew.
     */
    private void openGate() {
        if (PosServ == null || !ShootGateServo.IMPLEMENTED) return;
        if (!gateOpen()) PosServ.goB();
    }

    /** Close the gate — no-op while the gate is unimplemented (see {@link #openGate()}). */
    private void closeGate() {
        if (PosServ == null || !ShootGateServo.IMPLEMENTED) return;
        PosServ.goA();
    }

    // =====================================================================
    //  Motion sensing + per-loop bookkeeping
    // =====================================================================

    /**
     * Measure chassis translation speed and yaw rate from the follower pose
     * (Pedro exposes a pose but no {@code ChassisSpeeds}, so the deltas between
     * successive poses ARE the velocity), tick the warmup + debounce timers,
     * and re-arm a driver-suppressed fire after a short pause.
     *
     * <p>Call ONCE per loop from the OpMode's teleop tick, BEFORE the scheduler
     * polls the triggers — {@code KeyBindings.update()} calls it first.
     */
    public void readinessPeriodic() {
        long nowNs = System.nanoTime();
        double dt = lastPeriodicNs == 0L ? 0.0 : (nowNs - lastPeriodicNs) / 1.0e9;
        dt = Math.min(Math.max(dt, 0.0), 0.25);
        lastPeriodicNs = nowNs;
        warmupRemainingS = Math.max(0.0, warmupRemainingS - dt);

        Pose pose = follower != null ? follower.pose() : null;
        if (pose == null) {
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

        readyDebounce.advance(dt);

        // A driver override (manual transfer command) holds the fire off for a
        // moment; without this the next "ready" tick would slam it back on
        // while the driver is still unjamming by hand.
        if (fireSuppressedByDriver) {
            fireSuppressedForS += dt;
            if (fireSuppressedForS > FIRE_OVERRIDE_HOLD_S) {
                fireSuppressedByDriver = false;
            }
        }

        restoreManualIntakeWhenFree();
        // NOTE: no "Ready to shoot" line here. It used to be emitted from this
        // method, which runs ONE TICK LATER than the scheduler's fire policy —
        // so the capture showed FIRE (ready) next to Ready-to-shoot=false and
        // looked like a contradiction. The readiness lines now come exclusively
        // from reportReadinessDetails(), in the same pass as the policy.
        if (firing) {
            RobotMain.DashTelemetry.addData("Chassis speed (in/s)", "%.2f", measuredSpeedInS);
            RobotMain.DashTelemetry.addData("Chassis yaw (dps)", "%.1f", measuredYawDps);
        }
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

    /** An intake spinning above this (RPM) is considered "running". */
    public static double INTAKE_RUNNING_RPM = 50.0;

    /**
     * When true (default) the intake resumes at its configured RPM as soon as
     * the auto-fire stops. Set false if the driver would rather the intake
     * stay stopped until the right trigger is pressed again.
     */
    public static boolean restoreIntakeRpmOnRelease = true;

    /**
     * Linear falling-edge debounce: false is reported only after the input has
     * been false continuously for the full window; true passes through
     * instantly. Measured in scheduler time (advanced by
     * {@link #readinessPeriodic()}), not wall-clock, so a paused robot cannot
     * "catch up" into a spurious edge.
     */
    private static final class FallingEdgeDebounce {
        private final double windowS;
        private double timeS;
        private double falseForS = 0.0;
        private boolean out = false;

        FallingEdgeDebounce(double windowS) {
            this.windowS = Math.max(0.0, windowS);
        }

        /** Advance scheduler time. Call once per loop. */
        void advance(double dt) {
            timeS += Math.max(0.0, dt);
        }

        boolean update(boolean in) {
            if (in) {
                falseForS = 0.0;
                out = true;
                return true;
            }
            if (!out) {
                return false;
            }
            if (falseForS == 0.0) {
                falseForS = timeS;
            }
            if (timeS - falseForS >= windowS) {
                out = false;
            }
            return out;
        }
    }

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
