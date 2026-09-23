package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OTOSTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelIMUTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.TwoWheelTuner;

/**
 * AutoTune registration for Pedro Pathing 3.
 *
 * <p>How tuning works in Pedro 3: there is NO Tuning OpMode. The
 * {@code com.pedropathing:tuning} artifact hosts a robot webpage (AutoTune)
 * at {@code http://<hub-ip>:10158} (served from the AAR's
 * {@code assets/pedro} bundle). It auto-discovers every {@code static}
 * zero-arg method annotated with {@link Tuner} that returns a
 * {@link Procedure} (via {@code TunerScanner}), and each procedure's
 * {@code runOpMode(...)} steps register a temporary "Pedro Tuning" utility
 * OpMode on the fly. Open the page on a laptop on the same WiFi, pick a
 * tuner, and follow the prompts. Results print as copy-pasteable Java in the
 * page — paste them into {@link Constants}.
 *
 * <p>Wire order (AutoTune page order):
 * <ol>
 *   <li>Mecanum — motor names + directions (spins each wheel, asks fwd/rev)</li>
 *   <li>Localizer — pick ONE: Pinpoint (you have the goBILDA computer),
 *       OTOS / OctoQuad / TwoWheel / ThreeWheel(+IMU) only if you add that HW</li>
 *   <li>Foresight + Tests — need drivetrain + localizer functions, wired in
 *       {@link Constants} once 1+2 are known. NOT registered until then
 *       (see commented stubs below).</li>
 * </ol>
 */
public class Tuning {

    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "OTOS Tuner")
    public static Procedure otosTuner() {
        return new OTOSTuner();
    }

    @Tuner(name = "OctoQuad Tuner")
    public static Procedure octoQuadTuner() {
        return new OctoQuadTuner();
    }

    @Tuner(name = "Two Wheel Tuner")
    public static Procedure twoWheelTuner() {
        return new TwoWheelTuner();
    }

    @Tuner(name = "Three Wheel Tuner")
    public static Procedure threeWheelTuner() {
        return new ThreeWheelTuner();
    }

    @Tuner(name = "Three Wheel + IMU Tuner")
    public static Procedure threeWheelIMUTuner() {
        return new ThreeWheelIMUTuner();
    }

    // ---- Enable AFTER Constants wires drivetrain + localizer (post Mecanum + Pinpoint) ----
    // NOTE: tuners use getPinpointLocalizer() (RAW odometry). Fitting process
    // models on the Fusion wrapper would let vision corrections leak into the
    // drivetrain/algorithm constants. Match play uses Constants.create()
    // (fused) via RobotMain.
     @Tuner(name = "Foresight Tuner")
     public static Procedure foresightTuner() {
         return new ForesightTuner(Constants::getPinpointLocalizer, Constants::getDrivetrain);
     }
    //
     @Tuner(name = "Tests")
     public static Procedure tests() {
         return new Tests(Constants::getDrivetrain, Constants::getPinpointLocalizer,
                 () -> new Foresight(Constants.foresightConfig));
     }
}
