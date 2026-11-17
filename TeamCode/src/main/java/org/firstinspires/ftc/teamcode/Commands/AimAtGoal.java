package org.firstinspires.ftc.teamcode.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.Subsystems.turret.TurretPivot;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PPAssets;
import org.firstinspires.ftc.teamcode.WattageLib.lib.PPFile;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Aims the turret at the closest alliance target in {@code AimPoints.pp.json}
 * while the driver keeps full chassis control.
 *
 * <p>Requires only the turret subsystem; the follower pose is read-only.
 * Also provides live target/distance/tolerance helpers for {@link CommandFactory}
 * readiness checks even when the command is not running.
 */
@Configurable
public class AimAtGoal extends CommandBase {

    // ---- Panels-tunable aim constants (Panels -> AimAtGoal) ----
    /**
     * Robot-relative shooter offset (inches) from robot center (+X forward, +Y left).
     * Negative X moves the effective shot origin toward the back of the robot.
     */
    public static double SHOOTER_OFFSET_X_IN = -3.4;//-5.5;
    public static double SHOOTER_OFFSET_Y_IN = 0.0;

    /** Analog trigger threshold above which aim is considered held. */
    public static double TRIGGER_THRESHOLD = 0.3;
    /**
     * Chassis motion feedforward multiplier in loops of lead (1.0 = predict
     * one loop ahead). Increase to reduce lag while spinning/moving; 0 disables.
     */
    public static double YAW_FF_SCALE = 1.0;
    /**
     * Translational bearing-rate feedforward scale (1.0 = full compensation when
     * driving/strafing past the target; 0 = rotation feedforward only).
     */
    public static double TRANS_FF_SCALE = 1.0;
    /**
     * Low-pass smoothing factor (0..1) for chassis yaw rate. Higher values react
     * faster to turns; lower values smooth out heading jitter. 1.0 = no smoothing.
     */
    public static double YAW_RATE_EMA_ALPHA = 0.85;
    /**
     * Minimum |target angular rate| (deg/s) before feedforward is applied. Higher
     * values prevent small velocity vibrations from moving the turret setpoint.
     */
    public static double YAW_FF_DEADBAND_DPS = 5.0;
    /**
     * Shoot-on-the-move lead multiplier (0 = stationary aim only, 1.0 = full velocity lead).
     * Shifts the virtual target opposite to shooter velocity * time-of-flight.
     */
    public static double MOVING_LEAD_SCALAR = 1.0;
    /**
     * Maximum virtual-target lead shift (inches) applied by shoot-on-the-move.
     * Clamps velocity spikes so the aim point cannot jump unrealistically far.
     */
    public static double SOTM_MAX_LEAD_IN = 36.0;
    /**
     * Minimum shooter field speed (in/s) before SOTM lead is applied.
     * Ignores small stationary velocity noise.
     */
    public static double SOTM_MIN_SPEED_IN_S = 1.5;
    /**
     * Hysteresis margin (inches): keep the current target unless another point
     * is closer by more than this distance. Prevents target flipping near ties.
     */
    public static double AIM_SWITCH_MARGIN_IN = 6.0;
    /**
     * Minimum aim error tolerance (deg). Keep >= TurretPivot.POSITION_TOLERANCE_DEG
     * so the turret does not settle just outside the firing cone.
     */
    public static double TOLERANCE_DEG = 4.0;
    /**
     * Time (seconds) the turret may leave tolerance before "on target" turns false.
     * Prevents brief noise spikes from stopping a shot; 0 disables debounce.
     */
    public static double ON_TARGET_DEBOUNCE_S = 0.15;

    /** Closest-range aim tolerance floor (deg). Must stay >= TurretPivot.POSITION_TOLERANCE_DEG. */
    public static double AIM_TOL_MIN_DEG = 4.0;
    /** Distance (in) where aim tolerance begins loosening above {@link #AIM_TOL_MIN_DEG}. */
    public static double AIM_TOL_NEAR_DEG = 24.0;
    /** Additional aim tolerance (deg) added per inch past {@link #AIM_TOL_NEAR_DEG}. */
    public static double AIM_TOL_SLOPE_DEG_PER_IN = 0.05;
    /** Maximum allowed aim tolerance (deg) at long distance. */
    public static double AIM_TOL_MAX_DEG = 8.0;
    /**
     * Fraction of {@link #TOL_RPM_REF} at or below which tolerance narrows to
     * {@link #AIM_TOL_MIN_DEG}. 0 disables RPM-based tolerance scaling.
     */
    public static double TOL_RPM_FLOOR_FRAC = 0.25;
    /**
     * Flywheel RPM at or above which full distance-scaled tolerance applies.
     * Slower shots narrow toward {@link #AIM_TOL_MIN_DEG}; 0 scales by distance only.
     */
    public static double TOL_RPM_REF = 2000.0;

    /** Asset path of the aim-point file containing both alliances' targets. */
    public static final String ASSET_PATH = "pathfiles/AimPoints.pp.json";

    private final TurretPivot turret;
    private final Follower follower;
    /** Shared read-only aim points so CommandFactory avoids reloading the asset. */
    final PPFile aimPoints;
    private final Supplier<ALLIANCE_COLOR> alliance;
    /** Live flywheel RPM supplier used to scale aim tolerance (null = distance only). */
    private final DoubleSupplier targetRpm;

    // ---- Shared live aim state (written directly by the active AimAtGoal or idle snapshot) ----
    /** True while an AimAtGoal command is actively driving the turret. */
    public boolean activeAiming = false;
    /** Raw alliance target pose (goal coordinates). */
    public Pose activeTargetPose = null;
    /** Name of the currently selected alliance target (e.g. "RedBack"). */
    public String activeTargetName = "";
    /** SOTM-shifted virtual target pose (matches activeTargetPose when stationary). */
    public Pose activeEffectiveTarget = null;
    /** Effective SOTM-compensated distance (in) from shooter to activeEffectiveTarget. */
    public double activeDistanceIn = Double.NaN;
    /** Raw (non-SOTM) distance (in) from shooter to activeTargetPose. */
    public double activeRawDistanceIn = Double.NaN;
    /** Signed turret heading error (deg, + = need CCW turret motion) to activeEffectiveTarget. */
    public double activeHeadingErrorDeg = Double.NaN;
    /** Current distance/speed-scaled aim tolerance (deg). */
    public double activeToleranceDeg = TOLERANCE_DEG;

    private final AimAtGoal sharedState;

    private int targetIndex = -1;
    private Pose targetPose = null;
    private String targetName = "";
    private boolean usingFallback = false;
    private double headingErrorDeg = 0.0;
    private double turretSetpointDeg = 0.0;
    private double yawRateDps = 0.0;
    private double prevHeadingRad = Double.NaN;
    private long lastExecNs = 0L;
    private double loopDtSec = 0.02;
    private double lastDistanceIn = Double.NaN;
    private double lastRawDistanceIn = Double.NaN;
    private double lastLeadXIn = 0.0;
    private double lastLeadYIn = 0.0;
    private double toleranceDeg = TOLERANCE_DEG;

    private ALLIANCE_COLOR cachedAlliance = null;
    private final List<Pose> cachedAlliancePoints = new ArrayList<>();
    private final List<Integer> cachedIndices = new ArrayList<>();
    private boolean cachedFallback = false;

    private boolean rawOnTarget = false;
    private boolean onTargetDebounced = false;
    private long onTargetFalseSinceNs = 0L;
    private long onTargetDebounceNs = -1L;
    /** True while this command instance is actively scheduled. */
    private boolean aiming = false;

    /** Load aim points from assets and aim the TURRET with the driver in full control. */
    public AimAtGoal(TurretPivot turret, Follower follower, HardwareMap hardwareMap,
                     Supplier<ALLIANCE_COLOR> alliance) {
        this(turret, follower, loadAimPoints(hardwareMap), alliance, null, null);
    }

    /** Load aim points and scale the aim tolerance with the live flywheel RPM target. */
    public AimAtGoal(TurretPivot turret, Follower follower, HardwareMap hardwareMap,
                     Supplier<ALLIANCE_COLOR> alliance, DoubleSupplier targetRpm) {
        this(turret, follower, loadAimPoints(hardwareMap), alliance, targetRpm, null);
    }

    /** Inject an already-loaded PPFile (tests, or a shared copy). */
    public AimAtGoal(TurretPivot turret, Follower follower, PPFile aimPoints,
                     Supplier<ALLIANCE_COLOR> alliance) {
        this(turret, follower, aimPoints, alliance, null, null);
    }

    /** Inject an already-loaded PPFile + live RPM source for tolerance scaling. */
    public AimAtGoal(TurretPivot turret, Follower follower, PPFile aimPoints,
                     Supplier<ALLIANCE_COLOR> alliance, DoubleSupplier targetRpm) {
        this(turret, follower, aimPoints, alliance, targetRpm, null);
    }

    /** Full constructor: optionally publishes live state to a shared {@link AimAtGoal} holder. */
    public AimAtGoal(TurretPivot turret, Follower follower, PPFile aimPoints,
                     Supplier<ALLIANCE_COLOR> alliance, DoubleSupplier targetRpm,
                     AimAtGoal sharedState) {
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
        this.sharedState = sharedState != null ? sharedState : this;
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
        sharedState.activeAiming = true;
        sharedState.activeTargetPose = targetPose;
        sharedState.activeTargetName = targetName;
        if (RobotMain.DASHBOARD) {
            RobotMain.DashTelemetry.addData("Aim cmd", "START ("
                    + (targetName == null || targetName.isEmpty() ? "no target yet" : targetName) + ")");
        }
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
        lastDistanceIn = Double.NaN;
        lastRawDistanceIn = Double.NaN;
        lastLeadXIn = 0.0;
        lastLeadYIn = 0.0;
        rawOnTarget = false;
        onTargetDebounced = false;
        onTargetFalseSinceNs = 0L;
        sharedState.activeAiming = false;
        // Release turret to neutral/float when aim ends; chassis stays untouched.
        turret.BRAKE();
        sharedState.updateIdleSnapshot();
        if (RobotMain.DASHBOARD) {
            RobotMain.DashTelemetry.addData("Aim cmd", "END (" + (interrupted ? "interrupted" : "finished") + ")");
        }
    }

    /** Computes bearing + virtual-target SOTM lead + motion feedforward and commands the turret. */
    private void commandTurretTo(Pose robot) {
        Pose shooter = getShooterPose(robot);

        long nowNs = System.nanoTime();
        if (lastExecNs != 0L) {
            double dt = (nowNs - lastExecNs) * 1e-9;
            if (dt > 0.002 && dt < 0.2) {
                loopDtSec = dt;
            }
        }
        lastExecNs = nowNs;

        // Prefer Pedro's live Velocity (field-centric vx/vy in/s, omega rad/s); fall back to heading diff.
        Velocity vel = null;
        try {
            vel = follower != null ? follower.velocity() : null;
        } catch (Exception ignored) {
        }
        double rawYawDps;
        double headingRad = robot.heading();
        if (vel != null && Double.isFinite(vel.omega)) {
            rawYawDps = Math.toDegrees(vel.omega);
            prevHeadingRad = headingRad;
        } else if (Double.isNaN(prevHeadingRad)) {
            prevHeadingRad = headingRad;
            rawYawDps = 0.0;
        } else {
            double dHeading = Angle.normalizeSigned(headingRad - prevHeadingRad);
            prevHeadingRad = headingRad;
            rawYawDps = Math.toDegrees(dHeading / loopDtSec);
        }
        double alpha = Math.max(0.05, Math.min(1.0, YAW_RATE_EMA_ALPHA));
        yawRateDps = alpha * rawYawDps + (1.0 - alpha) * yawRateDps;

        // Shooter field-frame velocity (chassis translation + tangential offset from rotation).
        double shooterVx = 0.0;
        double shooterVy = 0.0;
        if (vel != null && Double.isFinite(vel.vx) && Double.isFinite(vel.vy)) {
            double omegaRad = Math.toRadians(yawRateDps);
            double offsetXs = shooter.x() - robot.x();
            double offsetYs = shooter.y() - robot.y();
            shooterVx = vel.vx - omegaRad * offsetYs;
            shooterVy = vel.vy + omegaRad * offsetXs;
        }

        // Virtual target lead: shift target opposite to shooter velocity * time-of-flight.
        double rawDx = targetPose.x() - shooter.x();
        double rawDy = targetPose.y() - shooter.y();
        lastRawDistanceIn = Math.hypot(rawDx, rawDy);

        Pose effectiveTarget = applySotmLead(targetPose, shooter, shooterVx, shooterVy);
        lastLeadXIn = effectiveTarget.x() - targetPose.x();
        lastLeadYIn = effectiveTarget.y() - targetPose.y();

        double dx = effectiveTarget.x() - shooter.x();
        double dy = effectiveTarget.y() - shooter.y();
        double distSq = dx * dx + dy * dy;
        lastDistanceIn = Math.sqrt(distSq);

        double desiredBearingDeg =
                Math.toDegrees(Angle.normalizeSigned(
                        Math.atan2(dy, dx) - robot.heading()));
        headingErrorDeg = normDeg180(desiredBearingDeg - turret.getPosition());
        toleranceDeg = aimToleranceDeg(lastDistanceIn);
        rawOnTarget = Math.abs(headingErrorDeg) <= toleranceDeg;
        updateOnTargetDebounce(nowNs);

        // Translational target angular rate: d/dt[atan2(dy, dx)] = (dy*vx_s - dx*vy_s) / dist^2
        double transRateDps = 0.0;
        if (TRANS_FF_SCALE != 0.0 && (shooterVx != 0.0 || shooterVy != 0.0)) {
            double safeDistSq = Math.max(distSq, 36.0); // clamp at 6 in radius to avoid close-range spikes
            double targetPhiRateRad = (dy * shooterVx - dx * shooterVy) / safeDistSq;
            transRateDps = Math.toDegrees(targetPhiRateRad) * TRANS_FF_SCALE;
        }

        double desiredTurretVelDps = transRateDps - yawRateDps;
        boolean ffActive = Math.abs(desiredTurretVelDps) >= YAW_FF_DEADBAND_DPS;
        double ffDeg = ffActive ? desiredTurretVelDps * loopDtSec * YAW_FF_SCALE : 0.0;
        double ffVelDps = ffActive ? desiredTurretVelDps : 0.0;

        turretSetpointDeg = desiredBearingDeg + ffDeg;
        turret.GotoSafeAngle(turretSetpointDeg, ffVelDps);

        sharedState.activeAiming = true;
        sharedState.activeTargetPose = targetPose;
        sharedState.activeTargetName = targetName;
        sharedState.activeEffectiveTarget = effectiveTarget;
        sharedState.activeDistanceIn = lastDistanceIn;
        sharedState.activeRawDistanceIn = lastRawDistanceIn;
        sharedState.activeHeadingErrorDeg = headingErrorDeg;
        sharedState.activeToleranceDeg = toleranceDeg;
    }

    /** Dashboard lines for the aim loop. */
    private void reportTelemetry() {
        if (!RobotMain.DASHBOARD) {
            return;
        }
        RobotMain.DashTelemetry.addData("Aim target", "%s (%.1f, %.1f)%s",
                targetName, targetPose.x(), targetPose.y(),
                usingFallback ? " [fallback: no alliance match]" : "");
        RobotMain.DashTelemetry.addData("Aim err (deg)", "%.2f", headingErrorDeg);
        RobotMain.DashTelemetry.addData("Aim turret set", "%.1f deg", turretSetpointDeg);
        RobotMain.DashTelemetry.addData("Aim chassis yaw", "%+.1f dps", yawRateDps);
        RobotMain.DashTelemetry.addData("Aim dt (ms)", "%.1f", loopDtSec * 1000.0);
        RobotMain.DashTelemetry.addData("Aim dist (in)", "%.1f (raw %.1f)", lastDistanceIn, lastRawDistanceIn);
        RobotMain.DashTelemetry.addData("Aim SOTM lead (in)", "%+.1f, %+.1f", lastLeadXIn, lastLeadYIn);
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

    // ---- Always-live aiming helpers (work with or without command running) ----

    /** Returns every aim point for the current alliance (cached per alliance, zero per-loop allocation). */
    public List<Pose> alliancePoints() {
        ALLIANCE_COLOR want = alliance.get();
        if (want != cachedAlliance) {
            cachedAlliancePoints.clear();
            cachedIndices.clear();
            String prefix = prefixFor(want);
            for (int i = 0; i < aimPoints.pointCount(); i++) {
                String name = aimPoints.pointName(i);
                if (name != null && name.regionMatches(true, 0, prefix, 0, prefix.length())) {
                    cachedIndices.add(i);
                    cachedAlliancePoints.add(aimPoints.point(i));
                }
            }
            boolean fallback = cachedIndices.isEmpty();
            if (fallback) {
                for (int i = 1; i < aimPoints.pointCount(); i++) {
                    cachedIndices.add(i);
                    cachedAlliancePoints.add(aimPoints.point(i));
                }
            }
            cachedFallback = fallback;
            cachedAlliance = want;
        }
        return cachedAlliancePoints;
    }

    /** True when {@link #alliancePoints()} had to fall back to the non-alliance point set. */
    public boolean isAllianceFallback() {
        alliancePoints(); // ensures the cache is warm
        return cachedFallback;
    }

    /** Refreshes the shared {@code active*} fields from live pose/velocity when no aim command is running. */
    public void updateIdleSnapshot() {
        if (activeAiming) {
            return;
        }
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null) {
            activeTargetPose = null;
            activeTargetName = "";
            activeEffectiveTarget = null;
            activeDistanceIn = Double.NaN;
            activeRawDistanceIn = Double.NaN;
            activeHeadingErrorDeg = Double.NaN;
            activeToleranceDeg = TOLERANCE_DEG;
            return;
        }
        Pose shooter = getShooterPose(robot);
        List<Pose> points = alliancePoints();
        int bestIdx = -1;
        Pose bestPose = null;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (int k = 0; k < points.size(); k++) {
            Pose p = points.get(k);
            if (p == null) {
                continue;
            }
            double d2 = dist2(shooter, p);
            if (d2 < bestD2) {
                bestD2 = d2;
                bestPose = p;
                bestIdx = cachedIndices.get(k);
            }
        }
        if (bestPose == null) {
            activeTargetPose = null;
            activeTargetName = "";
            activeEffectiveTarget = null;
            activeDistanceIn = Double.NaN;
            activeRawDistanceIn = Double.NaN;
            activeHeadingErrorDeg = Double.NaN;
            activeToleranceDeg = TOLERANCE_DEG;
            return;
        }
        Pose effective = correctedTargetFor(robot, bestPose);
        double rawDist = Math.sqrt(bestD2);
        double effDx = effective.x() - shooter.x();
        double effDy = effective.y() - shooter.y();
        double effDist = Math.hypot(effDx, effDy);
        double desiredBearingDeg = Math.toDegrees(Angle.normalizeSigned(
                Math.atan2(effDy, effDx) - robot.heading()));
        double errDeg = normDeg180(desiredBearingDeg - turret.getPosition());

        activeTargetPose = bestPose;
        activeTargetName = bestIdx >= 0 ? aimPoints.pointName(bestIdx) : "";
        activeEffectiveTarget = effective;
        activeRawDistanceIn = rawDist;
        activeDistanceIn = effDist;
        activeHeadingErrorDeg = errDeg;
        activeToleranceDeg = aimToleranceDeg(effDist);
    }

    private void ensureSnapshotReady() {
        if (activeTargetPose == null && !activeAiming && follower != null && follower.pose() != null) {
            updateIdleSnapshot();
        }
    }

    /** Current raw alliance target pose (updated by active aim command or idle snapshot). */
    public Pose liveTarget() {
        ensureSnapshotReady();
        return activeTargetPose;
    }

    /** Name of {@link #liveTarget()} ("" if none). */
    public String liveTargetName() {
        ensureSnapshotReady();
        return activeTargetName;
    }

    /** Distance (inches) from the shooter to {@code point}. NaN when there is no pose. */
    public double distanceTo(Pose point) {
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null || point == null) {
            return Double.NaN;
        }
        Pose shooter = getShooterPose(robot);
        return Math.hypot(point.x() - shooter.x(), point.y() - shooter.y());
    }

    /** Effective SOTM-compensated aim target in Pedro field coordinates. */
    public Pose liveEffectiveTarget() {
        ensureSnapshotReady();
        return activeEffectiveTarget;
    }

    /** Effective SOTM-adjusted distance (in) to {@link #liveTarget()} (NaN if none). */
    public double liveDistanceInches() {
        ensureSnapshotReady();
        return activeDistanceIn;
    }

    /** True when the turret is aligned on {@link #liveEffectiveTarget()} within tolerance. */
    public boolean isLiveOnTarget() {
        ensureSnapshotReady();
        return !Double.isNaN(activeHeadingErrorDeg)
                && Math.abs(activeHeadingErrorDeg) <= activeToleranceDeg;
    }

    /** Signed aim error (deg) between the turret and {@link #liveEffectiveTarget()} (NaN if none). */
    public double liveHeadingErrorDeg() {
        ensureSnapshotReady();
        return activeHeadingErrorDeg;
    }

    /** Shifts {@code rawTarget} by SOTM lead using live shooter velocity when enabled. */
    private Pose correctedTargetFor(Pose robot, Pose rawTarget) {
        if (MOVING_LEAD_SCALAR == 0.0 || follower == null) {
            return rawTarget;
        }
        Velocity vel = null;
        try {
            vel = follower.velocity();
        } catch (Exception ignored) {
        }
        if (vel == null || !Double.isFinite(vel.vx) || !Double.isFinite(vel.vy)) {
            return rawTarget;
        }
        Pose shooter = getShooterPose(robot);
        double omegaRad = Double.isFinite(vel.omega) ? vel.omega : 0.0;
        double offsetXs = shooter.x() - robot.x();
        double offsetYs = shooter.y() - robot.y();
        double shooterVx = vel.vx - omegaRad * offsetYs;
        double shooterVy = vel.vy + omegaRad * offsetXs;
        return applySotmLead(rawTarget, shooter, shooterVx, shooterVy);
    }

    /**
     * Applies virtual-target velocity lead: shifts {@code rawTarget} opposite to
     * shooter field velocity multiplied by time-of-flight (two-pass refined).
     */
    private static Pose applySotmLead(Pose rawTarget, Pose shooter, double shooterVx, double shooterVy) {
        if (MOVING_LEAD_SCALAR == 0.0 || Math.hypot(shooterVx, shooterVy) < SOTM_MIN_SPEED_IN_S) {
            return rawTarget;
        }
        double rawDist = Math.hypot(rawTarget.x() - shooter.x(), rawTarget.y() - shooter.y());
        double tof = CommandFactory.lutTofSec(rawDist);
        double leadX = -shooterVx * tof * MOVING_LEAD_SCALAR;
        double leadY = -shooterVy * tof * MOVING_LEAD_SCALAR;
        double leadMag = Math.hypot(leadX, leadY);
        if (SOTM_MAX_LEAD_IN > 0.0 && leadMag > SOTM_MAX_LEAD_IN) {
            double s = SOTM_MAX_LEAD_IN / leadMag;
            leadX *= s;
            leadY *= s;
        }

        // Second pass: refine TOF at the shifted virtual-target distance.
        double corrDist = Math.hypot((rawTarget.x() + leadX) - shooter.x(),
                (rawTarget.y() + leadY) - shooter.y());
        tof = CommandFactory.lutTofSec(corrDist);
        leadX = -shooterVx * tof * MOVING_LEAD_SCALAR;
        leadY = -shooterVy * tof * MOVING_LEAD_SCALAR;
        leadMag = Math.hypot(leadX, leadY);
        if (SOTM_MAX_LEAD_IN > 0.0 && leadMag > SOTM_MAX_LEAD_IN) {
            double s = SOTM_MAX_LEAD_IN / leadMag;
            leadX *= s;
            leadY *= s;
        }
        return new Pose(rawTarget.x() + leadX, rawTarget.y() + leadY, rawTarget.heading());
    }

    /**
     * Returns the distance- and speed-scaled aim tolerance half-width (deg)
     * clamped to [{@link #AIM_TOL_MIN_DEG}, {@link #AIM_TOL_MAX_DEG}].
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

    /** Picks the closest alliance point, keeping the current lock within {@link #AIM_SWITCH_MARGIN_IN}. */
    private void pickClosestTarget() {
        Pose robot = follower != null ? follower.pose() : null;
        if (robot == null) {
            return;
        }
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
            return;
        }
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
     * ({@link #SHOOTER_OFFSET_X_IN}/{@link #SHOOTER_OFFSET_Y_IN}). Shots originate
     * from here — the bearing and distance to targets must both be measured
     * from this point, not the robot center.
     */
    public static Pose getShooterPose(Pose robot) {
        if (robot == null) {
            return null;
        }
        double heading = robot.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double shooterX = robot.x() + SHOOTER_OFFSET_X_IN * cos - SHOOTER_OFFSET_Y_IN * sin;
        double shooterY = robot.y() + SHOOTER_OFFSET_X_IN * sin + SHOOTER_OFFSET_Y_IN * cos;
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
     * True when the last measured aim error is within {@link #getAimToleranceDeg()}.
     * For readiness checks when the command is not running, use {@link #isLiveOnTarget()}.
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
