package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IronConstants;

/**
 * Pedro Pathing 3 wiring: drivetrain + localizer + algorithm factories and the
 * tuned config holders. Paste AutoTune's generated code into the matching
 * block below (the page prints copy-pasteable Java per tuner).
 *
 * <p>Current status: UNCONFIGURED placeholders. Run AutoTune
 * ({@code http://<hub-ip>:10158}) first: Mecanum Tuner, then Pinpoint Tuner,
 * then paste results here and uncomment the Follower/Tests wiring in
 * {@link Tuning}.
 *
 * <p>Coordinate frame (Pedro = FIRST field frame, inches):
 * {@code Pose(x, y, heading)} with origin (0,0) at the field corner,
 * +X right/east, +Y forward/north, heading 0 = facing +X, CCW+ in radians.
 * Field is 0..144 on both axes; center is (72, 72). See
 * {@code PedroFieldBridge} for conversion to/from your center-origin
 * {@code FieldTracker} frame.
 */
public class Constants {

    // ================= Drivetrain (from Mecanum Tuner) =================
    // Paste the tuner's generated block here. Motor names MUST match your RC
    // config (yours: leftFront / leftBack / rightFront / rightBack).
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(IronConstants.FLMotor);
        c.frontRightName.set(IronConstants.FRMotor);
        c.backLeftName.set(IronConstants.RLMotor);
        c.backRightName.set(IronConstants.RRMotor);
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    // ================= Localizer (from Pinpoint Tuner) =================
    // You have the goBILDA Pinpoint computer (RC name "odo"). Paste the tuner's
    // generated block here (pod type, offsets, directions).
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(IronConstants.PinpointOdometryName);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.5851424209714875);
        c.yPodOffset.set(-1.3573616508423814);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // ================= Foresight algorithm (from Foresight Tuner) =================
    // Paste the tuner's generated block here once drivetrain+localizer are set.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.42338623497967826);
                Controller secondaryTranslationalForward = Controller.proportional(0.15642990073643634);
                Controller primaryTranslationalLateral = Controller.proportional(1.362189143787176);
                Controller secondaryTranslationalLateral = Controller.proportional(0.5032924902650813);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01455700584939155));
                c.brake.set(Controller.proportionalFeedforward(0.012373454971982816));

                c.headingFeedback.set(Controller.proportional(8.108301785412653));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07007308482493234, 0.0018574119466537302));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06294062814991255, 0.025328593152028418));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0022078089850833572, 0.0024082076238926674));

                c.maxAchievableForwardVelocity.set(47.4621240249149);
                c.maxAchievableStrafeVelocity.set(33.1429919978864);
                c.naturalForwardDeceleration.set(39.55342023066977);
                c.naturalStrafeDeceleration.set(91.86882129587802);
            }
    );

    /** Hub orientation for the TwoWheel/ThreeWheel+IMU localizers (if ever used). */
    public static RevHubOrientationOnRobot imuOrientation() {
        return new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }

    // ================= Fusion covariances (Pinpoint predictor + vision corrections) =================
    // FusionLocalizer(Localizer deadReckoning, Pose P0, Pose Q, Pose R, int bufferSize).
    // P/Q/R are digested as Matrix.diag(x, y, heading) VARIANCES (not stddevs):
    // larger == less trust. update() grows P by Q*|twist|*dt each loop; addMeasurement()
    // blends the vision pose with Kalman gain K = P/(P+R) and re-propagates history.
    // Heading units are radians^2, so 0.05 rad (~3deg stddev) -> 0.0025 variance.
    // Tune: raw Pinpoint in the tuners, Fusion in matches. Panels-tunable via FusionTune.
    /** Initial covariance P0 (in^2, in^2, rad^2). */
    public static Pose fusionInitialCovariance() {
        return new Pose(0.25, 0.25, Math.toRadians(3.0) * Math.toRadians(3.0));
    }

    /** Process noise Q: Pinpoint drift per unit twist (in^2, in^2, rad^2). */
    public static Pose fusionProcessNoise() {
        return new Pose(0.02, 0.02, Math.toRadians(0.5) * Math.toRadians(0.5));
    }

    /** Default measurement noise R for vision (in^2, in^2, rad^2); VisionFusion scales per-reading. */
    public static Pose fusionDefaultMeasurementNoise() {
        return new Pose(9.0, 9.0, Math.toRadians(6.0) * Math.toRadians(6.0));
    }

    /** History buffer: update() appends one entry per loop, oldest evicted past this. */
    public static int fusionBufferSize() {
        return 200;
    }

    // ================= Factories (used by Tuning Tests + future autos) =================
    // NOTE: getLocalizer() returns the Fusion wrapper so follower.update() fuses
    // automatically. Tuners (Foresight/Tests) call getPinpointLocalizer() for RAW
    // odometry; RobotMain/TeleOp/Auto get the fused pose via create().
     public static Drivetrain getDrivetrain(HardwareMap h) {
         return new Mecanum(h, drivetrainConfig);
     }
    //
    /**
     * RAW Pinpoint localizer. Use this in tuners (Foresight/Tests) so process
     * models are fit to odometry alone — vision corrections would corrupt them.
     * The tuner procedures in {@link Tuning} take this function.
     */
    public static Localizer getPinpointLocalizer(HardwareMap h) {
        return new PinpointLocalizer(h, localizerConfig);
    }

    /**
     * FUSED localizer: Pinpoint predictor wrapped in the Fusion Kalman filter.
     * This is what the follower drives on in TeleOp/Auto. Vision corrections
     * arrive via {@code ((FusionLocalizer) follower.localizer).addMeasurement(...)}.
     */
    public static Localizer getLocalizer(HardwareMap h) {
        return new FusionLocalizer(
                getPinpointLocalizer(h),
                fusionInitialCovariance(),
                fusionProcessNoise(),
                fusionDefaultMeasurementNoise(),
                fusionBufferSize());
    }

    public static Follower create(HardwareMap h) {
        return new Follower(getLocalizer(h), getDrivetrain(h), new Foresight(foresightConfig));
    }
}
