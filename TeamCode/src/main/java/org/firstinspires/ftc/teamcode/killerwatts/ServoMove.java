package org.firstinspires.ftc.teamcode.killerwatts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Comprehensive FTC servo subsystem: positional ({@link Servo}) + continuous ({@link CRServo}).
 *
 * <p>Two ways to use it:
 *
 * <pre>
 * // 1. Zero-code Dashboard test: just construct it. It auto-registers POS_NAME
 * //    as positional and CR_NAME as continuous (missing hardware is tolerated).
 * //    Change POS_NAME / CR_NAME + restart, then tune positions live.
 * servoMove = new ServoMove(this);
 *
 * // 2. Programmatic (auto or extra mechanisms):
 * servoMove.addPositional("claw", 0.2, 0.0, 1.0, false, 1.5);
 * servoMove.addContinuous("spinner", false, 1.0);
 * servoMove.setPosition("claw", 0.8);
 * servoMove.setContinuous("spinner", 1.0);
 * </pre>
 *
 * <p>Dashboard-tunable (FTC Dashboard -&gt; ServoMove): presets, limits, slew rate,
 * continuous powers, and the default gamepad bindings. Set
 * {@link #ENABLE_GAMEPAD} false in auto / when driving servos from code.
 *
 * <p>Default gamepad1 bindings (teleop test):
 * <ul>
 *   <li>X: toggle primary positional between {@link #POS_A} and {@link #POS_B}</li>
 *   <li>Left bumper / right bumper: nudge primary positional closed/open</li>
 *   <li>Dpad up / dpad down (hold): primary continuous forward / reverse</li>
 * </ul>
 */
@Config
public class ServoMove extends SubsystemBase {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    // ---- Primary-servo Dashboard tuning (auto-registered when hardware exists) ----
    /** Set to "" to skip auto-registering a positional servo. Name change needs restart. */
    public static String POS_NAME = "claw";
    public static double POS_A = 0.15;
    public static double POS_B = 0.75;
    /** Software clamp inside scaleRange. Keeps linkages off the hard stops. */
    public static double POS_MIN = 0.0;
    public static double POS_MAX = 1.0;
    public static boolean POS_REVERSED = false;
    /**
     * Slew limit, position units/sec. 1.5 = full sweep in ~0.67s (gentle on arms).
     * 0 or negative = snap instantly.
     */
    public static double POS_SLEW = 1.5;
    /** Hold-to-nudge speed while a bumper is held, position units/sec. */
    public static double POS_NUDGE_RATE = 0.6;
    /** Within this of target counts as "there" for {@link #isAtPosition}. */
    public static double POS_TOLERANCE = 0.01;

    /** Set to "" to skip auto-registering a continuous servo. Name change needs restart. */
    public static String CR_NAME = "spinner";
    public static boolean CR_REVERSED = false;
    /** Output cap so full stick isn't full speed. */
    public static double CR_MAX_POWER = 1.0;
    public static double CR_FWD_POWER = 1.0;
    public static double CR_REV_POWER = -1.0;
    /** Joystick/stick values under this park the servo instead of creeping. */
    public static double CR_DEADBAND = 0.08;

    /** Master switch for the default gamepad1 bindings below. */
    public static boolean ENABLE_GAMEPAD = true;

    private HardwareMap hardwareMap;
    private Gamepad gamepad1;

    private final Map<String, PosEntry> posServos = new LinkedHashMap<>();
    private final Map<String, CrEntry> crServos = new LinkedHashMap<>();

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = Double.NaN;

    private boolean prevToggle = false;
    public int loop = 0;

    private static final class PosEntry {
        Servo servo; // null when hardware name not configured
        double target;
        double current;
        double min;
        double max;
        double slew;
        boolean atInit = false;
    }

    private static final class CrEntry {
        CRServo servo; // null when hardware name not configured
        double power; // last applied, post-cap
        double requested; // programmatic hold value when no gamepad input
        double maxPower = 1.0; // per-servo output cap, 0..1
    }

    /** Legacy no-hardware constructor (kept so existing OpModes still compile). Does nothing. */
    public ServoMove() {
        timer.reset();
    }

    public ServoMove(LinearOpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad1 = opMode.gamepad1;
        timer.reset();
        // Best-effort defaults: missing names are kept as "missing" entries so
        // telemetry tells you the name is wrong instead of crashing the OpMode.
        if (POS_NAME != null && !POS_NAME.isEmpty()) {
            addPositional(POS_NAME, POS_A, POS_MIN, POS_MAX, POS_REVERSED, POS_SLEW);
        }
        if (CR_NAME != null && !CR_NAME.isEmpty()) {
            addContinuous(CR_NAME, CR_REVERSED, CR_MAX_POWER);
        }
    }

    // ---------------- Registration ----------------

    /**
     * Register (or reconfigure) a positional servo.
     *
     * @param hwName   hardware-map name; missing hardware is tolerated (entry shows "missing")
     * @param initial  starting target position, 0..1 (clamped to min/max)
     * @param min      software lower limit, 0..1
     * @param max      software upper limit, 0..1
     * @param reversed flip output direction
     * @param slewRate position units/sec; {@code <= 0} = instant
     */
    public void addPositional(String hwName, double initial, double min, double max,
                              boolean reversed, double slewRate) {
        PosEntry entry = posServos.computeIfAbsent(hwName, k -> new PosEntry());
        entry.min = clamp01(Math.min(min, max));
        entry.max = clamp01(Math.max(min, max));
        entry.slew = slewRate;
        entry.target = clamp(initial, entry.min, entry.max);
        if (!entry.atInit) {
            entry.current = entry.target;
            entry.atInit = true;
        }
        entry.servo = tryGetPositional(hwName);
        if (entry.servo != null) {
            // NOTE: no scaleRange() here on purpose. Positions are absolute 0..1
            // and software-clamped to [min, max] below, so scaleRange would
            // double-map (e.g. min=0.2 + setPosition(0.8) -> 0.68, not 0.8).
            entry.servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            entry.servo.setPosition(entry.current);
        }
    }

    /**
     * Register (or reconfigure) a continuous-rotation servo.
     *
     * @param hwName   hardware-map name; missing hardware is tolerated
     * @param reversed flip output direction
     * @param maxPower output cap, 0..1
     */
    public void addContinuous(String hwName, boolean reversed, double maxPower) {
        CrEntry entry = crServos.computeIfAbsent(hwName, k -> new CrEntry());
        entry.maxPower = clamp01(Math.abs(maxPower));
        entry.servo = tryGetContinuous(hwName);
        if (entry.servo != null) {
            entry.servo.setDirection(reversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
            entry.servo.setPower(0);
        }
        entry.requested = 0;
        entry.power = 0;
    }

    private Servo tryGetPositional(String hwName) {
        if (hardwareMap == null) return null;
        try {
            return hardwareMap.get(Servo.class, hwName);
        } catch (Exception ignored) {
            return null;
        }
    }

    private CRServo tryGetContinuous(String hwName) {
        if (hardwareMap == null) return null;
        try {
            return hardwareMap.get(CRServo.class, hwName);
        } catch (Exception ignored) {
            return null;
        }
    }

    // ---------------- Positional API ----------------

    /** Command a position (0..1, clamped to that servo's min/max). */
    public void setPosition(String hwName, double position) {
        PosEntry e = posServos.get(hwName);
        if (e == null) return;
        e.target = clamp(position, e.min, e.max);
    }

    /** Snap target AND output immediately (bypasses slew once). Useful for init. */
    public void setPositionImmediate(String hwName, double position) {
        PosEntry e = posServos.get(hwName);
        if (e == null) return;
        e.target = clamp(position, e.min, e.max);
        e.current = e.target;
        if (e.servo != null) e.servo.setPosition(e.current);
    }

    /** Primary-servo shortcut for {@code setPosition(POS_NAME, ...)}. */
    public void setPrimaryPosition(double position) {
        setPosition(POS_NAME, position);
    }

    public void goPresetA(String hwName) {
        setPosition(hwName, POS_A);
    }

    public void goPresetB(String hwName) {
        setPosition(hwName, POS_B);
    }

    /** Toggle between {@link #POS_A} and {@link #POS_B} (whichever is farther from target). */
    public void toggleAB(String hwName) {
        PosEntry e = posServos.get(hwName);
        if (e == null) return;
        double dA = Math.abs(e.target - POS_A);
        double dB = Math.abs(e.target - POS_B);
        e.target = (dA < dB) ? clamp(POS_B, e.min, e.max) : clamp(POS_A, e.min, e.max);
    }

    /** Relative jog, clamped. Positive = toward MAX. */
    public void nudge(String hwName, double delta) {
        PosEntry e = posServos.get(hwName);
        if (e == null) return;
        e.target = clamp(e.target + delta, e.min, e.max);
    }

    public double getTarget(String hwName) {
        PosEntry e = posServos.get(hwName);
        return e == null ? Double.NaN : e.target;
    }

    /** Last commanded (slew-limited) position actually sent to hardware. */
    public double getCommanded(String hwName) {
        PosEntry e = posServos.get(hwName);
        return e == null ? Double.NaN : e.current;
    }

    /** True when the slew-limited output is within {@link #POS_TOLERANCE} of target. */
    public boolean isAtPosition(String hwName) {
        PosEntry e = posServos.get(hwName);
        return e != null && Math.abs(e.target - e.current) <= POS_TOLERANCE;
    }

    public boolean hasPositional(String hwName) {
        PosEntry e = posServos.get(hwName);
        return e != null && e.servo != null;
    }

    // ---------------- Continuous API ----------------

    /**
     * Hold a power (-1..1, deadbanded + capped by {@link #CR_MAX_POWER}).
     * Gamepad dpad input overrides this while held; release returns to this value.
     */
    public void setContinuous(String hwName, double power) {
        CrEntry e = crServos.get(hwName);
        if (e == null) return;
        e.requested = capContinuous(power, e.maxPower);
    }

    public void stopContinuous(String hwName) {
        setContinuous(hwName, 0);
    }

    public void stopAllContinuous() {
        for (String name : crServos.keySet()) stopContinuous(name);
    }

    public double getContinuousPower(String hwName) {
        CrEntry e = crServos.get(hwName);
        return e == null ? Double.NaN : e.power;
    }

    public boolean hasContinuous(String hwName) {
        CrEntry e = crServos.get(hwName);
        return e != null && e.servo != null;
    }

    private double capContinuous(double power, double capIn) {
        if (Math.abs(power) < CR_DEADBAND) return 0;
        double cap = clamp01(Math.abs(capIn));
        return clamp(power, -cap, cap);
    }

    // ---------------- Loop ----------------

    @Override
    public void Periodic() {
        double now = timer.seconds();
        double rawDt = Double.isNaN(lastTime) ? 0.02 : Math.max(0, now - lastTime);
        // First loop / Dashboard pause shouldn't slingshot servos.
        double dtSec = rawDt > 0.25 ? 0.25 : rawDt;
        lastTime = now;

        // Live-tune support: Dashboard edits to limits/slew/reversed apply without
        // restart. A NEW hardware name is auto-registered on the fly, so you can
        // type the real config name into POS_NAME/CR_NAME and it just works.
        if (POS_NAME != null && !POS_NAME.isEmpty() && !posServos.containsKey(POS_NAME)) {
            addPositional(POS_NAME, POS_A, POS_MIN, POS_MAX, POS_REVERSED, POS_SLEW);
        }
        if (CR_NAME != null && !CR_NAME.isEmpty() && !crServos.containsKey(CR_NAME)) {
            addContinuous(CR_NAME, CR_REVERSED, CR_MAX_POWER);
        }
        PosEntry primary = posServos.get(POS_NAME);
        if (primary != null) {
            primary.min = clamp01(Math.min(POS_MIN, POS_MAX));
            primary.max = clamp01(Math.max(POS_MIN, POS_MAX));
            primary.slew = POS_SLEW;
            primary.target = clamp(primary.target, primary.min, primary.max);
            if (primary.servo != null) {
                primary.servo.setDirection(
                        POS_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            }
        }
        CrEntry primaryCr = crServos.get(CR_NAME);
        if (primaryCr != null) {
            primaryCr.maxPower = clamp01(Math.abs(CR_MAX_POWER));
            if (primaryCr.servo != null) {
                primaryCr.servo.setDirection(
                        CR_REVERSED ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
            }
        }

        if (ENABLE_GAMEPAD && gamepad1 != null) {
            handleGamepad(dtSec);
        }

        // Slew positional servos toward target, then write hardware once per loop.
        for (PosEntry e : posServos.values()) {
            if (e.slew > 0) {
                double maxMove = e.slew * dtSec;
                double err = e.target - e.current;
                if (Math.abs(err) <= maxMove) {
                    e.current = e.target;
                } else {
                    e.current += Math.signum(err) * maxMove;
                }
            } else {
                e.current = e.target;
            }
            if (e.servo != null) e.servo.setPosition(clamp(e.current, e.min, e.max));
        }

        // Continuous: gamepad dpad wins while held, else fall back to requested hold value.
        double stickOverride = gamepadDpadPower();
        for (Map.Entry<String, CrEntry> kv : crServos.entrySet()) {
            CrEntry e = kv.getValue();
            double want;
            if (ENABLE_GAMEPAD && gamepad1 != null && stickOverride != 0
                    && kv.getKey().equals(CR_NAME)) {
                want = capContinuous(stickOverride > 0 ? CR_FWD_POWER : CR_REV_POWER, e.maxPower);
            } else {
                want = capContinuous(e.requested, e.maxPower);
            }
            e.power = want;
            if (e.servo != null) e.servo.setPower(want);
        }

        // Telemetry last (one update per subsystem, matching Intake/Drivetrain pattern).
        telemetry.addData("ServoMove loop", loop);
        for (Map.Entry<String, PosEntry> kv : posServos.entrySet()) {
            PosEntry e = kv.getValue();
            telemetry.addData("Pos " + kv.getKey(),
                    e.servo == null ? "MISSING (check name)"
                            : String.format(Locale.US, "tgt %.3f cmd %.3f %s",
                            e.target, e.current, isAtPosition(kv.getKey()) ? "AT" : "moving"));
        }
        for (Map.Entry<String, CrEntry> kv : crServos.entrySet()) {
            CrEntry e = kv.getValue();
            telemetry.addData("CR " + kv.getKey(),
                    e.servo == null ? "MISSING (check name)"
                            : String.format(Locale.US, "pwr %.2f req %.2f", e.power, e.requested));
        }
        loop++;
        telemetry.update();
    }

    private double gamepadDpadPower() {
        if (gamepad1.dpad_up) return 1;
        if (gamepad1.dpad_down) return -1;
        return 0;
    }

    private void handleGamepad(double dt) {
        // X toggles primary positional between presets (edge-triggered).
        boolean toggle = gamepad1.x;
        if (toggle && !prevToggle && posServos.containsKey(POS_NAME)) {
            toggleAB(POS_NAME);
        }
        prevToggle = toggle;

        // Bumpers nudge primary positional while held.
        if (posServos.containsKey(POS_NAME)) {
            if (gamepad1.left_bumper) nudge(POS_NAME, -POS_NUDGE_RATE * dt);
            if (gamepad1.right_bumper) nudge(POS_NAME, POS_NUDGE_RATE * dt);
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.min(hi, Math.max(lo, v));
    }

    private static double clamp01(double v) {
        return clamp(v, 0.0, 1.0);
    }
}
