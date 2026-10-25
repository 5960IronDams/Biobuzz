package org.firstinspires.ftc.teamcode.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Subsystems.turret.TurretPivot;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PPAssets;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PPFile;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.WattageLib.Ballistics.NoFireZones;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Hold-to-aim command: SPINS THE TURRET to face the closest shooting target
 * for the current alliance while the driver keeps full chassis control
 * (translation AND rotation stay on the sticks — the robot no longer aims
 * itself, the turret does).
 *
 * <p>Targets come from {@code assets/pathfiles/AimPoints.pp}, parsed with
 * {@link PPFile}. Points are filtered by alliance name prefix
 * ({@code "Red*"} / {@code "Blue*"}), and the closest one (by XY distance to
 * the follower pose) is picked fresh every loop, so the lock follows you
 * around the field.
 *
 * <p>SESSION 5 (turret-aim port, per the FRC IndianaJoe pattern): the aim
 * member no longer touches the follower. Each loop it computes the target's
 * robot-relative bearing (field bearing minus chassis heading), hands it to
 * {@link TurretPivot#GotoSafeAngle(double, double)} together with a
 * chassis-yaw-rate feedforward so the turret tracks the field aim while the
 * chassis turns, and lets the turret's own soft limits + PID do the motion.
 * On release the turret motor goes NEUTRAL ({@link TurretPivot#BRAKE()}: PID
 * released, 0 power, FLOAT) — it no longer stays PID-locked wherever it was
 * aiming. The aim command only requires the turret, so the chassis is
 * untouched and driving continues without a blip.
 *
 * <p>Requirements: the turret subsystem ONLY. The follower is read (pose) but
 * never required, so driving is never interrupted.
 *
 * <p><b>SESSION 6 — always-live aiming math.</b> The class used to be the only
 * place a target could be chosen, and it did that from {@code execute()} as a
 * COMMAND — so nothing outside the command could answer "which target?" or
 * "how far?" without an instance of it being scheduled. Readiness checks need
 * both every loop, whether or not anybody is aiming (IndianaJoe's
 * {@code isReadyToShoot} is a set of plain {@code Trigger}s reading live
 * targets). So the math now lives in its own executable methods:
 * <ul>
 *   <li>{@link #alliancePoints()} — the current alliance's aim points (what
 *       {@link CommandFactory#TrackTarget()} and its fire group scan).</li>
 *   <li>{@link #liveTarget()} — the closest point for the current alliance,
 *       re-picked every call. The locked {@link #getTargetPose() target} is
 *       preferred (that instance has hysteresis and is the one the turret
 *       tracks); the follower alone answers when it is not aiming.</li>
 *   <li>{@link #distanceTo(Pose)} — shooter-to-point distance, the flywheel
 *       LUT input. A command running this instance uses
 *       {@link #getDistanceInches()} instead, so the LUT and the readiness
 *       check always agree on the same point.</li>
 * </ul>
 * {@code execute()} is now just "resolve the aim point, then
 * {@link #commandTurretTo(Pose)}" — behavior on the robot is unchanged.
 */
@Configurable
public class AimAtGoal extends CommandBase {

    // ---- Panels-tunable aim constants (Panels -> AimAtGoal) ----
    /** Trigger value above which "aim is held" (TeleOp wiring helper). */
    public static double TRIGGER_THRESHOLD = 0.3;
    /**
     * Chassis-yaw-rate feedforward to the turret. SESSION 5g UNITS FIX: the
     * lead added to the setpoint is ONE LOOP of predicted chassis rotation,
     * in DEGREES: {@code ffDeg = -yawRateDps * loopDtSec * YAW_FF_SCALE},
     * so YAW_FF_SCALE is now "loops of lead" (1.0 = predict the heading one
     * loop ahead). The old code added {@code -yawRateDps * 1.0} — a VELOCITY
     * injected as a POSITION, 1/dt ≈ 25–50x too strong: every 0.5–0.9 deg of
     * per-loop Pinpoint heading noise/rock became a ±25–45 deg setpoint
     * jump, the turret slammed chasing it, and its own reaction shake
     * re-excited the heading noise. Proven in capture 20261023-025055
     * i=470–525: bearing + yaw was constant at −2.8 deg (true aim 2.8 deg
     * away) while the setpoint thrashed ±30–60 deg — every degree of it was
     * this term. 0 disables.
     */
    public static double YAW_FF_SCALE = 1.0;
    /**
     * EMA smoothing (new-sample weight) on the chassis yaw-rate estimate.
     * Pinpoint heading jitters ~±0.5–0.9 deg per loop under turret-slam
     * vibration, so the raw dHeading/dt swings ±25–45 deg/s loop-to-loop;
     * 0.3 keeps real chassis spins responsive while averaging the
     * alternating noise down. 1.0 = no smoothing.
     */
    public static double YAW_RATE_EMA_ALPHA = 0.3;
    /**
     * |chassis yaw rate| below this (deg/s) is treated as zero for the
     * feedforward — damps Pedro pose micro-jitter into the turret setpoint.
     * Raised 2 → 5 in session 5g: with the units fix the residual lead is
     * tiny either way, and a stationary-but-shaking robot should produce
     * ZERO ff.
     */
    public static double YAW_FF_DEADBAND_DPS = 5.0;
    /**
     * Aim-point hysteresis: keep the locked target unless another alliance
     * point is closer than the current one by more than this margin (in).
     * Selection was PROVEN stable while stationary (capture 20261023-025055
     * logged RedFront on every aim loop) — this is hardening for driving
     * between hives so range noise at a near-tie can't flip the aim.
     */
    public static double AIM_SWITCH_MARGIN_IN = 6.0;
    /**
     * Aim-error floor (deg) for the "on target" cone: the tolerance never goes
     * below this, no matter how close or slow the shot is.
     */
    public static double TOLERANCE_DEG = 2.0;
    /**
     * SESSION 6: how long the turret may be out of the live aim tolerance
     * before "on target" drops (s). A falling-edge debounce, the FTC sibling
     * of the FRC readiness set's {@code shootReadyDebounceS} on
     * {@code isFlywheelAtSpeed()}: one noisy bearing sample must not tear the
     * transfer down mid-shot. 0 = no debounce.
     */
    public static double ON_TARGET_DEBOUNCE_S = 0.15;

    // ---- SESSION 6: target-aware tolerance scaling (IndianaJoe parity) ----
    /**
     * Aim error (deg) at or below which the shot counts as aligned regardless
     * of distance — the "close enough" floor. Distant targets get a looser
     * cone because the same angular error is a much smaller lateral miss far
     * away (FRC scaled its tolerances by distance exactly this way).
     */
    public static double AIM_TOL_MIN_DEG = 2.0;
    /** Distance (in) beyond which the aim tolerance starts to loosen (below it: MIN_DEG). */
    public static double AIM_TOL_NEAR_DEG = 24.0;
    /** Extra degrees of aim tolerance per inch of distance past {@link #AIM_TOL_NEAR_DEG}. */
    public static double AIM_TOL_SLOPE_DEG_PER_IN = 0.05;
    /** Hard ceiling on the distance-scaled aim tolerance (deg). */
    public static double AIM_TOL_MAX_DEG = 8.0;
    /**
     * Shot speed (RPM) below which a shot earns the MINIMUM aim band, expressed
     * as a FRACTION of {@link #TOL_RPM_REF}. Between this and the reference RPM
     * the band ramps linearly from {@link #AIM_TOL_MIN_DEG} to the full
     * distance band. 0 disables the RPM term entirely (distance only).
     */
    public static double TOL_RPM_FLOOR_FRAC = 0.25;
    /**
     * Shot speed (RPM) that earns the FULL distance band. Slower shots have the
     * band squeezed toward {@link #AIM_TOL_MIN_DEG} (see {@link #aimToleranceDeg}).
     * 0 = ignore RPM, scale by distance only.
     */
    public static double TOL_RPM_REF = 2000.0;

    /** Asset path of the aim-point file. Already contains BOTH alliances' points. */
    public static final String ASSET_PATH = "pathfiles/AimPoints.pp";

    private final TurretPivot turret;
    private final Follower follower;
    private final PPFile aimPoints;
    private final Supplier<ALLIANCE_COLOR> alliance;
    /**
     * SESSION 6: the RPM the flywheel manager is driving to right now — the
     * one piece of shoot state that lives outside this class (CommandFactory
     * owns the distance -> RPM LUT and its slew). Used to scale the aim
     * tolerance: a slow shot is more forgiving of bearing error than a fast
     * one. null (or NaN) = scale by distance only.
     */
    private final DoubleSupplier targetRpm;

    // Per-run state (also surfaced for telemetry / flywheel compositions).
    private int targetIndex = -1;
    private Pose targetPose = null;
    private String targetName = "";
    private boolean usingFallback = false;
    private double headingErrorDeg = 0.0;
    private double turretSetpointDeg = 0.0;
    private double yawRateDps = 0.0;
    private double prevHeadingRad = Double.NaN;
    /** SESSION 5g: last execute() timestamp for the real loop period (ns). */
    private long lastExecNs = 0L;
    /** SESSION 5g: measured command loop period (s) — no hard-coded 0.02. */
    private double loopDtSec = 0.02;
    /** Distance to the locked point as of the last execute() (in). */
    private double lastDistanceIn = Double.NaN;
    /** Distance-scaled aim tolerance for the current target (deg). */
    private double toleranceDeg = TOLERANCE_DEG;

    // SESSION 6: cached alliance point set (alliance changes at most once per
    // OpMode; the scan is O(points) but fire-group and readiness both ask).
    private ALLIANCE_COLOR cachedAlliance = null;
    private int[] cachedIndices = new int[0];
    private boolean cachedFallback = false;

    // SESSION 6: "on target" with a falling-edge debounce.
    private boolean rawOnTarget = false;
    private boolean onTargetDebounced = false;
    private long onTargetFalseSinceNs = 0L;
    private long onTargetDebounceNs = -1L;
    /** Set while this command is scheduled; {@link #liveTarget()} prefers the locked target then. */
    private boolean aiming = false;

    /** Load aim points from assets and aim the TURRET with the driver in full control. */
    public AimAtGoal(TurretPivot turret, Follower follower, HardwareMap hardwareMap,
                     Supplier<ALLIANCE_COLOR> alliance) {
        this(turret, follower, loadAimPoints(hardwareMap), alliance, null);
    }

    /** Load aim points and scale the aim tolerance with the live flywheel RPM target. */
    public AimAtGoal(TurretPivot turret, Follower follower, HardwareMap hardwareMap,
                     Supplier<ALLIANCE_COLOR> alliance, DoubleSupplier targetRpm) {
        this(turret, follower, loadAimPoints(hardwareMap), alliance, targetRpm);
    }

    /** Inject an already-loaded PPFile (tests, or a shared copy). */
    public AimAtGoal(TurretPivot turret, Follower follower, PPFile aimPoints,
                     Supplier<ALLIANCE_COLOR> alliance) {
        this(turret, follower, aimPoints, alliance, null);
    }

    /** Full constructor: injected aim file + a live RPM-source for tolerance scaling. */
    public AimAtGoal(TurretPivot turret, Follower follower, PPFile aimPoints,
                     Supplier<ALLIANCE_COLOR> alliance, DoubleSupplier targetRpm) {
        if (turret == null) {
            throw new IllegalArgumentException("AimAtGoal needs a non-null TurretPivot");
        }
        if (aimPoints == null) {
            throw new IllegalArgumentException("AimAtGoal: " + ASSET_PATH + " has no points");
        }
        // Own the turret while running — manual right-stick spin must yield.
        addRequirements(turret);
        this.turret = turret;
        this.follower = follower; // read-only: pose source for target bearing
        this.aimPoints = aimPoints;
        this.alliance = alliance != null ? alliance : () -> ALLIANCE_COLOR.ALLIANCE_RED;
        this.targetRpm = targetRpm;
    }


    /**
     * Load the aim file with a plain degrees factory (NO Blue mirroring).
     * The file already stores both alliances' points in absolute field
     * coordinates, so mirroring would fling the Blue points to the wrong side.
     * Alliance is handled by filtering point names ({@code "Red*"}/{@code "Blue*"}) instead.
     */
    public static PPFile loadAimPoints(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException(
                    "AimAtGoal needs a non-null HardwareMap to load " + ASSET_PATH);
        }
        try {
            return PPAssets.fromAsset(hardwareMap, ASSET_PATH, PoseFactory.degrees());
        } catch (Exception e) {
            throw new RuntimeException("AimAtGoal: failed to load asset " + ASSET_PATH, e);
        }
    }

    /** Name prefix used to filter points per alliance (matches AimPoints.pp). */
    public static String prefixFor(ALLIANCE_COLOR alliance) {
        return alliance == ALLIANCE_COLOR.ALLIANCE_BLUE ? "Blue" : "Red";
    }

    /** TeleOp helper: is the trigger held past {@link #TRIGGER_THRESHOLD}? */
    public static boolean held(double triggerValue) {
        return triggerValue > TRIGGER_THRESHOLD;
    }

    // ---- Command lifecycle (runs until cancelled : hold trigger to aim) ----

    @Override
    public void initialize() {
        prevHeadingRad = Double.NaN;
        lastExecNs = 0L;
        aiming = true;
        pickClosestTarget();
        RobotMain.DashTelemetry.addData("Aim cmd", "START ("
                + (targetName == null || targetName.isEmpty() ? "no target yet" : targetName) + ")");
    }

    @Override
    public void execute() {
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null) {
            return;
        }
        pickClosestTarget();
        if (targetPose == null) {
            return;
        }
        commandTurretTo(robot);
        reportTelemetry();
    }

    @Override
    public boolean isFinished() {
        return false; // hold-to-aim: cancel on trigger release or compose with .until(...)
    }

    @Override
    public void end(boolean interrupted) {
        aiming = false;
        // Drop the per-run shot state: the install is no longer tracking that
        // target, so anything asking afterwards must fall back to the live
        // closest-point answer rather than a stale distance/tolerance.
        lastDistanceIn = Double.NaN;
        rawOnTarget = false;
        onTargetDebounced = false;
        onTargetFalseSinceNs = 0L;
        // Release the turret back to NEUTRAL when the button is let go: this
        // used to be a no-op, so GotoSafeAngle's setpoint stayed latched and
        // PID kept the turret locked wherever it was last aimed (capture
        // 20261024-232657: it sat at its aim angle with the button released).
        // BRAKE() drops the setpoint and puts the 5203 in FLOAT at 0 power —
        // the next aim/jog re-engages the PID. Chassis is untouched (never
        // required) so driving continues without a blip.
        turret.BRAKE();
        RobotMain.DashTelemetry.addData("Aim cmd", "END (" + (interrupted ? "interrupted" : "finished") + ")");
    }

    /**
     * One aim step against an already-resolved {@code robot} pose: bearing,
     * yaw-rate feedforward, turret command, and the aim/readiness state the
     * dashboard and the readiness triggers consume. Extracted from
     * {@code execute()} so the same math is used every loop regardless of who
     * calls it.
     */
    private void commandTurretTo(Pose robot) {
        // Shots originate from the SHOOTER pivot, not the robot center — rotate
        // the robot pose through its heading to apply the pivot offset, then
        // aim from that point.
        Pose shooter = getShooterPose(robot);
        lastDistanceIn = Math.hypot(targetPose.x() - shooter.x(), targetPose.y() - shooter.y());

        // Field bearing to the target from the SHOOTER, converted to a TURRET
        // angle: desired robot-relative bearing (deg, +CCW, 0 = chassis forward)
        double desiredBearingDeg =
                Math.toDegrees(Angle.normalizeSigned(
                        Math.atan2(targetPose.y() - shooter.y(), targetPose.x() - shooter.x())
                                - robot.heading()));
        headingErrorDeg = normDeg180(desiredBearingDeg - turret.getPosition());
        toleranceDeg = aimToleranceDeg(lastDistanceIn);
        rawOnTarget = Math.abs(headingErrorDeg) <= toleranceDeg;

        // SESSION 5g: measure the REAL command loop period. The old
        // hard-coded 0.02 s fed both the yaw-rate estimate and (through the
        // ff units bug) the setpoint noise amplitude.
        long nowNs = System.nanoTime();
        if (lastExecNs != 0L) {
            double dt = (nowNs - lastExecNs) * 1e-9;
            if (dt > 0.002 && dt < 0.2) {
                loopDtSec = dt;
            }
        }
        lastExecNs = nowNs;
        updateOnTargetDebounce(nowNs);

        // Chassis yaw-rate feedforward (deg/s, +CCW) — EMA-smoothed, then
        // sign-flipped into a ONE-LOOP-OF-LEAD angle (see YAW_FF_SCALE) so
        // the turret holds the field aim while the chassis turns (same idea
        // as FRC GotoSafeAngle(angle, velocityDps)).
        double headingRad = robot.heading();
        if (Double.isNaN(prevHeadingRad)) {
            prevHeadingRad = headingRad;
            yawRateDps = 0.0;
        } else {
            double dHeading = Angle.normalizeSigned(headingRad - prevHeadingRad);
            prevHeadingRad = headingRad;
            double rawYawDps = Math.toDegrees(dHeading / loopDtSec);
            yawRateDps = YAW_RATE_EMA_ALPHA * rawYawDps
                    + (1.0 - YAW_RATE_EMA_ALPHA) * yawRateDps;
        }
        double ffDeg = Math.abs(yawRateDps) < YAW_FF_DEADBAND_DPS
                ? 0.0
                : -yawRateDps * loopDtSec * YAW_FF_SCALE;

        turretSetpointDeg = desiredBearingDeg + ffDeg;
        // GotoSafeAngle clamps to soft limits + picks the nearest ±360 twin,
        // exactly like FRC's chooseTurretSetpointDeg. The velocity argument
        // is a true deg/s feedforward rate (used by the turret's kV path
        // when position-mode kV is enabled).
        turret.GotoSafeAngle(turretSetpointDeg, -yawRateDps);
    }

    /** Dashboard lines for the aim loop. */
    private void reportTelemetry() {
        RobotMain.DashTelemetry.addData("Aim target", "%s (%.1f, %.1f)%s",
                targetName, targetPose.x(), targetPose.y(),
                usingFallback ? " [fallback: no alliance match]" : "");
        RobotMain.DashTelemetry.addData("Aim err (deg)", "%.2f", headingErrorDeg);
        RobotMain.DashTelemetry.addData("Aim turret set", "%.1f deg", turretSetpointDeg);
        RobotMain.DashTelemetry.addData("Aim chassis yaw", "%+.1f dps", yawRateDps);
        RobotMain.DashTelemetry.addData("Aim dt (ms)", "%.1f", loopDtSec * 1000.0);
        RobotMain.DashTelemetry.addData("Aim dist (in)", "%.1f", lastDistanceIn);
        RobotMain.DashTelemetry.addData("Aim tolerance (deg)", "%.2f", toleranceDeg);
        RobotMain.DashTelemetry.addData("Aim on target", onTargetDebounced);
    }

    /**
     * Falling-edge debounce on {@link #rawOnTarget}: a value that goes false
     * stays "on target" for {@link #ON_TARGET_DEBOUNCE_S} before it counts.
     * Rising edges are immediate (the first-time entry into tolerance must
     * never be delayed).
     */
    private void updateOnTargetDebounce(long nowNs) {
        if (onTargetDebounceNs < 0L) {
            onTargetDebounceNs = Math.max(0L, (long) (ON_TARGET_DEBOUNCE_S * 1.0e9));
        }
        if (rawOnTarget) {
            onTargetDebounced = true;
            onTargetFalseSinceNs = 0L;
            return;
        }
        if (!onTargetDebounced) {
            onTargetFalseSinceNs = 0L;
            return;
        }
        if (onTargetFalseSinceNs == 0L) {
            onTargetFalseSinceNs = nowNs;
        }
        if (nowNs - onTargetFalseSinceNs >= onTargetDebounceNs) {
            onTargetDebounced = false;
        }
    }

    // ---- SESSION 6: always-live aiming math (works with or without the command) ----

    /**
     * Every aim point on the field belonging to the CURRENT alliance, in the
     * order they appear in {@code AimPoints.pp} ({@code "Red*"} / {@code "Blue*"}
     * name prefix, case-insensitive). Falls back to all points except index 0
     * (the "Robot" start point) when the file has none with the prefix.
     * Cached per alliance — a mid-match alliance change re-scans.
     */
    public Pose[] alliancePoints() {
        ALLIANCE_COLOR want = alliance.get();
        if (want != cachedAlliance) {
            String prefix = prefixFor(want);
            int[] found = new int[aimPoints.pointCount()];
            int n = 0;
            for (int i = 0; i < aimPoints.pointCount(); i++) {
                String name = aimPoints.pointName(i);
                if (name != null && name.regionMatches(true, 0, prefix, 0, prefix.length())) {
                    found[n++] = i;
                }
            }
            boolean fallback = n == 0;
            if (fallback) {
                n = 0;
                for (int i = 1; i < aimPoints.pointCount(); i++) {
                    found[n++] = i;
                }
            }
            int[] trimmed = new int[n];
            System.arraycopy(found, 0, trimmed, 0, n);
            cachedIndices = trimmed;
            cachedFallback = fallback;
            cachedAlliance = want;
        }
        Pose[] out = new Pose[cachedIndices.length];
        for (int k = 0; k < cachedIndices.length; k++) {
            out[k] = aimPoints.point(cachedIndices[k]);
        }
        return out;
    }

    /** True when {@link #alliancePoints()} had to fall back to the non-alliance point set. */
    public boolean isAllianceFallback() {
        alliancePoints(); // ensures the cache is warm
        return cachedFallback;
    }

    /**
     * The point the turret should be shooting at RIGHT NOW: the closest point
     * of the current alliance to the shooter. When THIS instance is running
     * as a command its hysteresis-locked {@link #getTargetPose() target} is
     * returned (so the fire group, the RPM LUT and the readiness checks all
     * agree on one point); otherwise the choice is made fresh from the
     * follower pose — which is what lets a readiness check or an auto-firing
     * compound track the shooter with no {@code AimAtGoal} command scheduled.
     *
     * @return the target pose, or null when there is no pose or no points
     */
    public Pose liveTarget() {
        if (aiming && targetPose != null) {
            return targetPose;
        }
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null) {
            return null;
        }
        Pose shooter = getShooterPose(robot);
        Pose best = null;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (Pose p : alliancePoints()) {
            if (p == null) {
                continue;
            }
            double d2 = dist2(shooter, p);
            if (d2 < bestD2) {
                bestD2 = d2;
                best = p;
            }
        }
        return best;
    }

    /**
     * Name of the point {@link #liveTarget()} would pick right now ("" if none).
     *
     * <p>Resolved by INDEX, not by pose equality: Pedro's {@link Pose} does not
     * override {@code equals()}, so matching the returned pose back to a name
     * would mean comparing raw doubles — fragile and unnecessary, since
     * {@link #alliancePoints()} and {@link #liveTarget()} both iterate the same
     * cached index list.
     */
    public String liveTargetName() {
        if (aiming && targetPose != null) {
            return targetName;
        }
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null) {
            return "";
        }
        alliancePoints(); // ensures the index cache is warm
        Pose shooter = getShooterPose(robot);
        int bestIndex = -1;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (int k = 0; k < cachedIndices.length; k++) {
            Pose p = aimPoints.point(cachedIndices[k]);
            if (p == null) {
                continue;
            }
            double d2 = dist2(shooter, p);
            if (d2 < bestD2) {
                bestD2 = d2;
                bestIndex = cachedIndices[k];
            }
        }
        return bestIndex < 0 ? "" : aimPoints.pointName(bestIndex);
    }

    /**
     * Distance (inches) from the shooter (robot pose + pivot offset) to
     * {@code point} — the flywheel RPM LUT input. NaN when there is no pose.
     */
    public double distanceTo(Pose point) {
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null || point == null) {
            return Double.NaN;
        }
        Pose shooter = getShooterPose(robot);
        return Math.hypot(point.x() - shooter.x(), point.y() - shooter.y());
    }

    /** Distance (in) to the {@link #liveTarget()} right now — NaN when there is no target. */
    public double liveDistanceInches() {
        return distanceTo(liveTarget());
    }

    /**
     * "The shot, as it would happen right now": turret within the live aim
     * tolerance of the {@link #liveTarget()}. Computed on demand, so it is
     * valid whether or not a command is driving the turret.
     *
     * @return true when the turret is aligned on the live target
     */
    public boolean isLiveOnTarget() {
        Pose target = liveTarget();
        if (target == null) {
            return false;
        }
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null) {
            return false;
        }
        Pose shooter = getShooterPose(robot);
        double desiredBearingDeg = Math.toDegrees(Angle.normalizeSigned(
                Math.atan2(target.y() - shooter.y(), target.x() - shooter.x()) - robot.heading()));
        double err = normDeg180(desiredBearingDeg - turret.getPosition());
        return Math.abs(err) <= aimToleranceDeg(distanceTo(target));
    }

    /**
     * Signed aim error (deg) between the turret and the {@link #liveTarget()}
     * — NaN when there is nothing to aim at. Command-free sibling of
     * {@link #getHeadingErrorDeg()}.
     */
    public double liveHeadingErrorDeg() {
        Pose target = liveTarget();
        Pose robot = follower != null ? follower.pose() : null;
        if (target == null || robot == null) {
            return Double.NaN;
        }
        Pose shooter = getShooterPose(robot);
        double desiredBearingDeg = Math.toDegrees(Angle.normalizeSigned(
                Math.atan2(target.y() - shooter.y(), target.x() - shooter.x()) - robot.heading()));
        return normDeg180(desiredBearingDeg - turret.getPosition());
    }

    /**
     * Distance-scaled aim tolerance (deg) — the FTC port of IndianaJoe's
     * "distance-scaled tolerances" (FRC overwrote its fixed
     * {@code currentTurretTolerance} every cycle from {@code TargetSelector}).
     * Two things loosen the cone:
     * <ol>
     *   <li><b>Distance.</b> The same angular error is a much smaller lateral
     *       miss on a far target, so
     *       {@code tol = MIN + (d - NEAR) * SLOPE}, clamped to MAX.</li>
     *   <li><b>Shot speed.</b> A slower wheel shoots a slower ball, whose arc
     *       is more forgiving of bearing error — but a slow shot is also less
     *       accurate to aim, so the band is blended: the full distance band at
     *       {@link #TOL_RPM_REF} or above, ramping linearly down to
     *       {@link #AIM_TOL_MIN_DEG} at
     *       {@code TOL_RPM_REF * TOL_RPM_FLOOR_FRAC}. With no RPM source —
     *       or before the RPM manager has commanded anything — the RPM term is
     *       skipped and the pure distance band applies (the generous default;
     *       readiness already requires the flywheel to BE at that RPM).</li>
     * </ol>
     *
     * @param distIn distance from the shooter to the target (in); NaN/0 = MIN
     * @return the tolerance band half-width in degrees, in [MIN, MAX]
     */
    public double aimToleranceDeg(double distIn) {
        if (Double.isNaN(distIn) || distIn <= 0) {
            return AIM_TOL_MIN_DEG;
        }
        double tol = AIM_TOL_MIN_DEG
                + Math.max(0.0, distIn - AIM_TOL_NEAR_DEG) * AIM_TOL_SLOPE_DEG_PER_IN;
        tol = Math.min(tol, AIM_TOL_MAX_DEG);
        double rpm = targetRpm != null ? targetRpm.getAsDouble() : Double.NaN;
        if (!Double.isNaN(rpm) && rpm > 0 && TOL_RPM_REF > 0) {
            double floorRpm = TOL_RPM_REF * clamp01(TOL_RPM_FLOOR_FRAC);
            double span = TOL_RPM_REF - floorRpm;
            // 0 at the floor RPM, 1 at/above the reference RPM.
            double speedFrac = span <= 0 ? 1.0 : clamp01((rpm - floorRpm) / span);
            tol = AIM_TOL_MIN_DEG + (tol - AIM_TOL_MIN_DEG) * speedFrac;
        }
        return Math.max(AIM_TOL_MIN_DEG, Math.min(AIM_TOL_MAX_DEG, tol));
    }


    // ---- Target selection ----

    /**
     * Re-pick the closest alliance point to the current pose.
     */
    private void pickClosestTarget() {
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null) {
            return;
        }
        // Closest target as measured from the SHOOTER (pivot offset applied),
        // not the robot center.
        Pose shooter = getShooterPose(robot);
        String prefix = prefixFor(alliance.get());

        boolean fallbackNow = false;
        int best = -1;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (int i = 0; i < aimPoints.pointCount(); i++) {
            String name = aimPoints.pointName(i);
            if (name == null || !name.regionMatches(true, 0, prefix, 0, prefix.length())) {
                continue;
            }
            double d2 = dist2(shooter, aimPoints.point(i));
            if (d2 < bestD2) {
                bestD2 = d2;
                best = i;
            }
        }
        if (best < 0) {
            // No alliance-prefixed points (file renamed?) - fall back to all
            // target points (skipping index 0, the "Robot" start point).
            for (int i = 1; i < aimPoints.pointCount(); i++) {
                double d2 = dist2(shooter, aimPoints.point(i));
                if (d2 < bestD2) {
                    bestD2 = d2;
                    best = i;
                }
            }
            fallbackNow = true;
        }
        if (best < 0) {
            return; // no points at all; execute() guards on targetPose == null
        }
        // SESSION 5g hysteresis: keep the locked target unless the challenger
        // is meaningfully closer (AIM_SWITCH_MARGIN_IN). Only applies within
        // the same candidate set (alliance vs fallback) so a set change still
        // switches immediately. Stationary selection was already proven
        // stable — this guards driving between hives against near-tie flips.
        if (best != targetIndex && targetIndex >= 0
                && targetIndex < aimPoints.pointCount()
                && fallbackNow == usingFallback) {
            double curDist = Math.sqrt(dist2(shooter, aimPoints.point(targetIndex)));
            double bestDist = Math.sqrt(bestD2);
            if (bestDist >= curDist - AIM_SWITCH_MARGIN_IN) {
                best = targetIndex;
            }
        }
        usingFallback = fallbackNow;
        if (best != targetIndex || targetPose == null) {
            targetIndex = best;
            targetPose = aimPoints.point(best);
            targetName = aimPoints.pointName(best);
        }
    }

    private static double dist2(Pose a, Pose b) {
        double dx = a.x() - b.x();
        double dy = a.y() - b.y();
        return dx * dx + dy * dy;
    }

    /**
     * Compute the SHOOTER's actual field-frame position: the robot's pose
     * rotated through its own heading to apply the physical pivot offset
     * ({@link NoFireZones#SHOOTER_OFFSET_X_IN}/{@code _Y_IN}). Shots originate
     * from here — the bearing and distance to targets must both be measured
     * from this point, not the robot center.
     */
    private static Pose getShooterPose(Pose robot) {
        double heading = robot.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double shooterX = robot.x() + NoFireZones.SHOOTER_OFFSET_X_IN * cos - NoFireZones.SHOOTER_OFFSET_Y_IN * sin;
        double shooterY = robot.y() + NoFireZones.SHOOTER_OFFSET_X_IN * sin + NoFireZones.SHOOTER_OFFSET_Y_IN * cos;
        return new Pose(shooterX, shooterY, heading);
    }

    // ---- Telemetry / composition accessors ----

    /** Name of the currently locked point (e.g. {@code "RedBack"}). */
    public String getTargetName() {
        return targetName;
    }

    /** Pose of the currently locked point. */
    public Pose getTargetPose() {
        return targetPose;
    }

    /** Signed turret aim error in degrees (+ = need CCW turret motion). */
    public double getHeadingErrorDeg() {
        return headingErrorDeg;
    }

    /** Last turret setpoint (deg, robot-relative, before soft-limit clamping). */
    public double getTurretSetpointDeg() {
        return turretSetpointDeg;
    }

    /** Last measured chassis yaw rate (deg/s, +CCW) used for the feedforward. */
    public double getYawRateDps() {
        return yawRateDps;
    }

    /**
     * True when the COMMAND's last measured aim error is within the
     * distance/speed-scaled tolerance ({@link #getAimToleranceDeg()}).
     * Legacy semantics kept (FRC {@code TOLERANCE_DEG} is now the tolerance
     * FLOOR); for a readiness check that must work with no command running use
     * {@link #isLiveOnTarget()}.
     */
    public boolean isOnTarget() {
        return Math.abs(headingErrorDeg) <= toleranceDeg;
    }

    /** Distance/speed-scaled aim tolerance (deg) that currently applies. */
    public double getAimToleranceDeg() {
        return toleranceDeg;
    }

    /** True when the command is currently driving the turret. */
    public boolean isAiming() {
        return aiming;
    }

    /**
     * Distance (in) to the locked target as of the last aim step; NaN if unknown
     * (including when the command is not running — see {@link #liveDistanceInches()}).
     */
    public double getLastDistanceInches() {
        return aiming ? lastDistanceIn : Double.NaN;
    }

    /**
     * Distance (inches) from the SHOOTER (robot pose + pivot offset) to the
     * locked point — the correct input to the flywheel RPM LUT; NaN if unknown.
     */
    public double getDistanceInches() {
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null || targetPose == null) {
            return Double.NaN;
        }
        Pose shooter = getShooterPose(robot);
        return Math.hypot(targetPose.x() - shooter.x(), targetPose.y() - shooter.y());
    }

    /** Number of points in the loaded aim file (expect 5: Robot + 4 targets). */
    public int getPointCount() {
        return aimPoints.pointCount();
    }

    /** Normalize degrees to [-180, 180). */
    private static double normDeg180(double deg) {
        double r = (deg + 180.0) % 360.0;
        if (r < 0) r += 360.0;
        return r - 180.0;
    }

    /** Clamp to [0, 1]. */
    private static double clamp01(double v) {
        return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
    }
}
