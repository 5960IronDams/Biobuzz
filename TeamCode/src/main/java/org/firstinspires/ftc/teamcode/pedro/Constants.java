package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
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
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    // ================= Localizer (from Pinpoint Tuner) =================
    // You have the goBILDA Pinpoint computer (RC name "odo"). Paste the tuner's
    // generated block here (pod type, offsets, directions).
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
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

    // ================= Factories (used by Tuning Tests + future autos) =================
    // Uncomment once the configs above are pasted in.
     public static Drivetrain getDrivetrain(HardwareMap h) {
         return new Mecanum(h, drivetrainConfig);
     }
    //
     public static Localizer getLocalizer(HardwareMap h) {
         return new PinpointLocalizer(h, localizerConfig);
     }
    //
     public static Follower create(HardwareMap h) {
         return new Follower(getLocalizer(h), getDrivetrain(h), new Foresight(foresightConfig));
     }
}
