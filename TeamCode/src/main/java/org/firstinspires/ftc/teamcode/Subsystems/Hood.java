package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.WattageLib.lib.killaUtils.clamp;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareHealth;
import org.firstinspires.ftc.teamcode.IronConstants;
import org.firstinspires.ftc.teamcode.RobotMain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Hood subsystem: Swyft Speed v1.1 positional servo driving a hood sector gear.
 *
 * <h2>Kinematics &amp; Hardware Geometry</h2>
 * <ul>
 *   <li>Pinion gear: 25 teeth on servo shaft</li>
 *   <li>Hood sector gear: 17 teeth travel (out of 172 teeth for a full 360&deg; circle)</li>
 *   <li>Servo: Swyft Speed v1.1 with default 270&deg; travel range (0.0 .. 1.0)</li>
 *   <li>Angular stroke: 17/25 revs = 244.8&deg; servo rotation = 90.67% of 270&deg; range</li>
 *   <li>Measured launch angles: 0&deg; = vertical up, 90&deg; = horizontal forward</li>
 *   <li>Hood Down = 16.831&deg; (steeper trajectory)</li>
 *   <li>Hood Up = 46.852&deg; (flatter trajectory)</li>
 * </ul>
 *
 * <p>Panels-tunable (Panels -&gt; Hood): zero position, direction, slew rate,
 * angle limits, and tolerance apply live.
 */
@Configurable
public class Hood extends SubsystemBase {
    Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    /** DS failure-board name for this subsystem (HardwareHealth). */
    public static final String HEALTH_NAME = "HOOD";

    /**
     * IS THE HOOD PHYSICALLY MOUNTED AND WIRED?
     * True enables active servo control and readiness gating during matches.
     */
    public static boolean IMPLEMENTED = true;

    // ---- Kinematic constants ----
    public static final int PINION_TEETH = 25;
    public static final int TRAVEL_TEETH = 17;
    public static final int FULL_GEAR_TEETH = 172;
    public static final double SERVO_RANGE_DEG = 270.0;

    /** Measured launch angle at physical DOWN stop (0 deg = vertical, 90 deg = horizontal). */
    public static double HOOD_DOWN_DEG = 16.831;
    /** Measured launch angle at physical UP stop. */
    public static double HOOD_UP_DEG = 46.852;

    /**
     * Calibrated servo position at HOOD_DOWN_DEG (16.831 deg).
     * Measured on the robot at lowest hood setting: ~0.94.
     */
    public static double POS_DOWN = 0.94;

    /**
     * Calibrated servo position at HOOD_UP_DEG (46.852 deg).
     * Meshed calibration position: 0.0.
     */
    public static double POS_UP = 0.0;

    /**
     * Servo zero position used when meshing gears at the physical stop.
     */
    public static double ZERO_SERVO_POS = 0.0;

    /** Invert servo command direction if hardware linkage requires it. */
    public static boolean REVERSED = false;

    /** Slew rate limit in degrees/sec to protect gear teeth against sudden snaps. */
    public static double SLEW_DEG_PER_SEC = 90.0;

    /** Tolerance in degrees for {@link #isAtTargetDegrees(double)}. */
    public static double TOLERANCE_DEG = 1.0;

    /** Stowed / rest angle when idle or not shooting. */
    public static double STOWED_DEG = 16.831;

    /** Jog speed for manual calibration commands (deg/sec). */
    public static double NUDGE_DEG_PER_SEC = 20.0;

    private final Servo servo;
    private double targetDeg;
    private double currentDeg;
    private long lastTimeNs = 0L;
    private long lastJogNs = -1L;
    private boolean slewOverride = false;

    public static Hood tryCreate(OpMode opMode) {
        if (opMode == null || opMode.hardwareMap == null) return null;
        try {
            return new Hood(opMode);
        } catch (Exception e) {
            Log.w("IronLog", "Hood servo '" + IronConstants.HoodServoName + "' absent from RC config: " + e.getMessage());
            return null;
        }
    }

    public Hood(OpMode opMode) {
        this(opMode.hardwareMap.get(Servo.class, IronConstants.HoodServoName));
    }

    public Hood(Servo servo) {
        this.servo = servo;
        this.targetDeg = STOWED_DEG;
        this.currentDeg = STOWED_DEG;
        this.lastTimeNs = System.nanoTime();
        if (servo != null && IMPLEMENTED) {
            servo.setPosition(degreesToServoPosition(currentDeg));
        }
    }

    /**
     * Convert hood launch angle (degrees) to normalized FTC servo position (0.0 .. 1.0)
     * based on calibrated POS_DOWN and POS_UP endpoints.
     */
    public static double degreesToServoPosition(double degrees) {
        double span = HOOD_UP_DEG - HOOD_DOWN_DEG;
        double frac = span <= 0.0 ? 0.0 : (degrees - HOOD_DOWN_DEG) / span;
        frac = clamp(frac, 0.0, 1.0);

        double pos = POS_DOWN + frac * (POS_UP - POS_DOWN);
        if (REVERSED) {
            pos = 1.0 - pos;
        }
        return clamp(pos, 0.0, 1.0);
    }

    /**
     * Set target launch angle in degrees, clamped to [HOOD_DOWN_DEG, HOOD_UP_DEG].
     */
    public void setTargetDegrees(double degrees) {
        targetDeg = clamp(degrees, Math.min(HOOD_DOWN_DEG, HOOD_UP_DEG), Math.max(HOOD_DOWN_DEG, HOOD_UP_DEG));
    }

    public double getTargetDegrees() {
        return targetDeg;
    }

    public double getCurrentDegrees() {
        return currentDeg;
    }

    /**
     * True if the current slewed angle is within tolerance of the target,
     * or if the hood is marked as NOT IMPLEMENTED.
     */
    public boolean isAtTargetDegrees(double tolDeg) {
        if (!IMPLEMENTED) return true;
        return Math.abs(currentDeg - targetDeg) <= tolDeg;
    }

    public boolean isAtTargetDegrees() {
        return isAtTargetDegrees(TOLERANCE_DEG);
    }

    public void stow() {
        setTargetDegrees(STOWED_DEG);
    }

    @Override
    public void periodic() {
        long nowNs = System.nanoTime();
        double dt = lastTimeNs == 0L ? 0.02
                : Math.min(Math.max((nowNs - lastTimeNs) / 1.0e9, 0.0), 0.25);
        lastTimeNs = nowNs;

        if (slewOverride) {
            currentDeg = targetDeg;
        } else if (SLEW_DEG_PER_SEC > 0.0) {
            double maxDelta = SLEW_DEG_PER_SEC * dt;
            double diff = targetDeg - currentDeg;
            currentDeg += clamp(diff, -maxDelta, maxDelta);
        } else {
            currentDeg = targetDeg;
        }

        double pos = degreesToServoPosition(currentDeg);
        if (servo != null && IMPLEMENTED) {
            servo.setPosition(pos);
        }

        if (RobotMain.DashTelemetry != null) {
            RobotMain.DashTelemetry.addData("Hood implemented", IMPLEMENTED);
            RobotMain.DashTelemetry.addData("Hood target deg", "%.2f", targetDeg);
            RobotMain.DashTelemetry.addData("Hood cur deg", "%.2f", currentDeg);
            RobotMain.DashTelemetry.addData("Hood servo pos", "%.4f", pos);
            RobotMain.DashTelemetry.addData("Hood at target", isAtTargetDegrees(TOLERANCE_DEG));
        }
    }

    // =====================================================================
    //  COMMANDS
    // =====================================================================

    /** Command to set hood angle and finish once settled at setpoint. */
    public Command toAngleCmd(double deg) {
        return new FunctionalCommand(
                () -> setTargetDegrees(deg),
                () -> {},
                (interrupted) -> {},
                () -> isAtTargetDegrees(TOLERANCE_DEG),
                this
        ).setName("HoodToAngle");
    }

    /** Snap servo to zero position limit for calibration and gear meshing. */
    public Command zeroCmd() {
        return new FunctionalCommand(
                () -> {
                    slewOverride = true;
                    targetDeg = (Math.abs(ZERO_SERVO_POS - POS_DOWN) < Math.abs(ZERO_SERVO_POS - POS_UP))
                            ? HOOD_DOWN_DEG : HOOD_UP_DEG;
                    currentDeg = targetDeg;
                    if (servo != null) {
                        servo.setPosition(ZERO_SERVO_POS);
                    }
                },
                () -> {
                    if (servo != null) {
                        servo.setPosition(ZERO_SERVO_POS);
                    }
                },
                (interrupted) -> slewOverride = false,
                () -> true,
                this
        ).setName("HoodZero");
    }

    /** Jog hood angle while held (increase or decrease). */
    public Command jogCmd(boolean increase) {
        return new FunctionalCommand(
                () -> lastJogNs = System.nanoTime(),
                () -> {
                    long nowNs = System.nanoTime();
                    double dt = lastJogNs <= 0L ? 0.02
                            : Math.min(Math.max((nowNs - lastJogNs) / 1.0e9, 0.0), 0.25);
                    lastJogNs = nowNs;
                    double step = (increase ? 1.0 : -1.0) * NUDGE_DEG_PER_SEC * dt;
                    setTargetDegrees(targetDeg + step);
                },
                (interrupted) -> lastJogNs = -1L,
                () -> false,
                this
        ).setName("HoodJog");
    }
}
