package org.firstinspires.ftc.teamcode.irondams.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


/**
 * Represents a standard four-wheel robot drivetrain configuration.
 * This class handles hardware initialization for all four drive motors,
 * setting their default directions and zero-power behaviors.
 *
 * <p>All wheel power output should go through {@link #setWheelPowers} so the
 * stiction (kS) boost, per-wheel trims, and future velocity-PID mode stay in
 * one place. Callers do mecanum mixing + normalization, then hand the
 * -1..1 shaped powers here.
 */
public final class FourWheelDriveTrain {
    private final DcMotorEx leftFrontDrive;
    private final DcMotorEx leftBackDrive;
    private final DcMotorEx rightFrontDrive;
    private final DcMotorEx rightBackDrive;

    /** Per-motor PIDF overrides for future individual tuning. Null = use shared DriveConstants gains. */
    private PIDFCoefficients pidfFL;
    private PIDFCoefficients pidfFR;
    private PIDFCoefficients pidfBL;
    private PIDFCoefficients pidfBR;
    private PIDFCoefficients lastPushedFL;
    private PIDFCoefficients lastPushedFR;
    private PIDFCoefficients lastPushedBL;
    private PIDFCoefficients lastPushedBR;
    private boolean lastVelocityMode = false;
    private boolean gainsInitialized = false;

    /**
     * @return The hardware instance for the front-left motor.
     */
    public DcMotorEx getLeftFrontDrive() {
        return leftFrontDrive;
    }

    /**
     * @return The hardware instance for the back-left motor.
     */
    public DcMotorEx getLeftBackDrive() {
        return leftBackDrive;
    }

    /**
     * @return The hardware instance for the front-right motor.
     */
    public DcMotorEx getRightFrontDrive() {
        return rightFrontDrive;
    }

    /**
     * @return The hardware instance for the back-right motor.
     */
    public DcMotorEx getRightBackDrive() {
        return rightBackDrive;
    }


    /**
     * Initializes the drivetrain by fetching motors from the hardware map.
     * Configures the right-side motors to REVERSE direction and all motors to BRAKE mode.
     *
     * @param hardwareMap The OpMode's hardware map used to access the motor ports.
     */
    public FourWheelDriveTrain (HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Open-loop default. Velocity mode (RUN_USING_ENCODER + PIDF) is engaged
        // lazily by setWheelPowers() when DriveConstants.USE_VELOCITY flips true,
        // so Dashboard toggling doesn't require a restart/re-init.
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ---- Future individual-motor PID tuning (works alongside shared gains) ----

    /** Override PIDF for one motor. Pass null to fall back to shared DriveConstants gains. */
    public void setPidfOverrides(PIDFCoefficients fl, PIDFCoefficients fr,
                                 PIDFCoefficients bl, PIDFCoefficients br) {
        pidfFL = fl;
        pidfFR = fr;
        pidfBL = bl;
        pidfBR = br;
        gainsInitialized = false; // force re-push on next velocity output
    }

    /** Convenience: override a single motor, null clears back to shared gains. */
    public void setPidfOverride(String wheel, PIDFCoefficients pidf) {
        switch (wheel) {
            case "FL": pidfFL = pidf; break;
            case "FR": pidfFR = pidf; break;
            case "BL": pidfBL = pidf; break;
            case "BR": pidfBR = pidf; break;
            default: throw new IllegalArgumentException("wheel must be FL/FR/BL/BR");
        }
        gainsInitialized = false;
    }

    /** True once a per-motor override is set (lets you run shared + individual together). */
    public boolean hasIndividualPidf() {
        return pidfFL != null || pidfFR != null || pidfBL != null || pidfBR != null;
    }

    private PIDFCoefficients sharedGains() {
        return new PIDFCoefficients((float) DriveConstants.V_KP, (float) DriveConstants.V_KI,
                (float) DriveConstants.V_KD, (float) DriveConstants.V_KF);
    }

    private boolean pidfEquals(PIDFCoefficients a, PIDFCoefficients b) {
        if (a == b) return true;
        if (a == null || b == null) return false;
        return a.p == b.p && a.i == b.i && a.d == b.d && a.f == b.f;
    }

    private void setRunMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    /** Push shared/individual PIDF gains when in velocity mode; no-op in power mode. */
    private void ensureVelocityGains() {
        PIDFCoefficients shared = sharedGains();
        PIDFCoefficients wantFL = pidfFL != null ? pidfFL : shared;
        PIDFCoefficients wantFR = pidfFR != null ? pidfFR : shared;
        PIDFCoefficients wantBL = pidfBL != null ? pidfBL : shared;
        PIDFCoefficients wantBR = pidfBR != null ? pidfBR : shared;
        if (gainsInitialized
                && pidfEquals(wantFL, lastPushedFL) && pidfEquals(wantFR, lastPushedFR)
                && pidfEquals(wantBL, lastPushedBL) && pidfEquals(wantBR, lastPushedBR)) {
            return;
        }
        leftFrontDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, wantFL);
        rightFrontDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, wantFR);
        leftBackDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, wantBL);
        rightBackDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, wantBR);
        lastPushedFL = wantFL;
        lastPushedFR = wantFR;
        lastPushedBL = wantBL;
        lastPushedBR = wantBR;
        gainsInitialized = true;
    }

    /**
     * Single output stage for all drivetrain modes.
     *
     * <p>Power mode (default): applies per-wheel scale, then the kS stiction
     * boost {@code sign * (kS + (1-kS)*|p|)} so post-mix attenuated wheels still
     * break free. Inputs near zero stay zero.
     *
     * <p>Velocity mode (DriveConstants.USE_VELOCITY): same shaped inputs drive
     * {@code setVelocity(p * scale * maxTps)} with shared or per-motor PIDF.
     * No kS jump needed there -- the velocity loop handles breakaway via kF/P.
     */
    public void setWheelPowers(double fl, double fr, double bl, double br) {
        if (DriveConstants.USE_VELOCITY) {
            if (!lastVelocityMode) {
                setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                gainsInitialized = false;
                lastVelocityMode = true;
            }
            ensureVelocityGains();
            double maxTps = DriveConstants.getMaxTicksPerSec();
            leftFrontDrive.setVelocity(fl * DriveConstants.SCALE_FL * maxTps);
            rightFrontDrive.setVelocity(fr * DriveConstants.SCALE_FR * maxTps);
            leftBackDrive.setVelocity(bl * DriveConstants.SCALE_BL * maxTps);
            rightBackDrive.setVelocity(br * DriveConstants.SCALE_BR * maxTps);
            return;
        }
        if (lastVelocityMode) {
            // Back to open loop: cut velocity loop, hold with brake.
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lastVelocityMode = false;
        }
        leftFrontDrive.setPower(boost(fl * DriveConstants.SCALE_FL, DriveConstants.KS_TRIM_FL));
        rightFrontDrive.setPower(boost(fr * DriveConstants.SCALE_FR, DriveConstants.KS_TRIM_FR));
        leftBackDrive.setPower(boost(bl * DriveConstants.SCALE_BL, DriveConstants.KS_TRIM_BL));
        rightBackDrive.setPower(boost(br * DriveConstants.SCALE_BR, DriveConstants.KS_TRIM_BR));
    }

    /** kS stiction boost: jumps small-but-nonzero powers to breakaway, maps 1 -> 1. */
    static double boost(double p, double ksTrim) {
        if (Math.abs(p) < 1e-6) return 0.0;
        double ks = DriveConstants.getMinPower() + ksTrim;
        ks = Math.max(0.0, Math.min(0.6, ks));
        double a = Math.abs(p);
        if (a > 1.0) a = 1.0;
        return Math.signum(p) * (ks + (1.0 - ks) * a);
    }
}
