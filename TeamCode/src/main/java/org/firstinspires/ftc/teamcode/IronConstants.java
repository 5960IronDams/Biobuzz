package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class IronConstants {
    public static boolean CompBot = true;
    public static final Pose FieldXYMaxPoint = new Pose(141.0,141.0,0);
    public static final Pose FieldCenter = new Pose(FieldXYMaxPoint.x()/2,FieldXYMaxPoint.y()/2,0);

    public static final String FLMotor = CompBot? "m1" : "leftFront";
    public static final String FRMotor = CompBot? "m2" : "rightFront";
    public static final String RLMotor = CompBot? "m0" : "leftBack";
    public static final String RRMotor = CompBot? "m3" : "rightBack";
    public static final String PinpointOdometryName = "odo";


    public static final String IntakeMotorName = CompBot? "ehm0" :"intake";
    public static final String TransferMotorName = CompBot? "ehm1" :"transfer";
    public static final String FlywheelMotorName = CompBot? "ehm2" :"flywheel";
    public static final String FrontServoName = CompBot? "shootgate" :"shootgate";

    // ---- Motor encoder constants ----
    /**
     * goBILDA 5203-series Yellow Jacket encoder: 28 ticks (PPR) per MOTOR-shaft
     * revolution (the 1:1/ungeared value - the encoder sits on the motor shaft,
     * before the planetary gearbox). Ticks per OUTPUT-shaft revolution = this
     * times the gear ratio - use {@link #gobilda5203TicksPerRev(double)}.
     */
    public static final double GOBILDA_5203_TICKS_PER_MOTOR_REV = 28.0;

    /**
     * Ticks per OUTPUT-shaft revolution for a goBILDA 5203 Yellow Jacket geared
     * {@code gearRatio}:1 - pass the ratio, get the TPR back. Feed the result
     * into RPM<->ticks conversions for {@code DcMotorEx.setVelocity()}
     * (e.g. 5.2 -> 145.6; 19.2 -> ~537.7, the classic 312 RPM value).
     */
    public static double gobilda5203TicksPerRev(double gearRatio) {
        return GOBILDA_5203_TICKS_PER_MOTOR_REV * gearRatio;
    }

    // Turret (ported from 2026-IndianaJoe TurretPivot — see TURRET_PORTING_NOTES.md)
    /**
     * goBILDA 5203 drive motor — the turret's only drive (the legacy CRServo
     * path was retired). Configure on a motor port; the RC config motor type
     * is not critical (we drive with setPower), but pick the 5203 variant
     * matching your gearbox.
     */
    public static final String TurretMotorName = CompBot? "ehm3" :"turret_motor";
    /** OctoQuad MK1 on Control Hub I2C port 3 (Comp bot has it on expansion hub i2c port 3); the Through Bores' ABS wires are on its channels 6 (B) and 7 (A). */
    public static final String TurretOctoQuadName = CompBot? "octoquad" :"octoquad";

    // Alliance reset poses for extreme localization loss, should be in a known location.
    public static final Pose RED_RESET_POSE = new Pose(10.5, 10.5, Math.toRadians(0));
    public static final Pose BLUE_RESET_POSE = new Pose(20.5, 20.5, Math.toRadians(180));


    //TODO: Test todo.
    //TODO: panels custom to make website open fast also make 1.0.7 biobuzz version work.
    //todo: try ftclib/Dairy/solverslib
    //todo: commandgamepad? triggers? linear-interpolation table.
    //todo: real Limelight OS and fmaps.
    //todo LL draw onto dashboard map our estimated Pose.
}
