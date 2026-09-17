package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SharedObjects;
import org.firstinspires.ftc.teamcode.killerwatts.Intake;
import org.firstinspires.ftc.teamcode.killerwatts.lib.DashboardFieldRenderer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.clearAll;

@Autonomous(name = "ExampleAuto1", group = "org/firstinspires/ftc/teamcode/Autos")
public class ExampleAuto extends OpMode {
    //ROBOT BOILERPLATE -- doesnt change between OpModes
    public FtcDashboard dashboard = SharedObjects.dashboard;
    public Telemetry telemetry = SharedObjects.telemetry;
    private Follower follower;// = SharedObjects.follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Pose-agnostic dashboard renderer: Pedro poses go straight in, no Pinpoint/Kalman needed.
    private final DashboardFieldRenderer fieldRenderer = new DashboardFieldRenderer();

    private Intake intake;
    //END ROBOT BOILERPLATE -- doesnt change between OpModes
    // our poses
    private final Pose startPose = poseFactory.of(24, 24, 0);
    private final Pose park = poseFactory.of(48, 48, 90);
    // other poses...
    private final Pose controlPose = poseFactory.of(36, 60, 45);
    private final Pose startPose2 = poseFactory.of(24, 24, 0);
    private final Pose scorePose = poseFactory.of(48, 48, 90);
    private final Pose parkPose = poseFactory.of(72, 48, 90);

    private void SetupRobotBoilerPlate()
    {
        clearAll(); // avoid double-registration on re-run
        intake = new Intake(this);

    }

    @Override
    public void init() {
        SetupRobotBoilerPlate();
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        UpdateTelemetry();
    }

    @Override
    public void start() {
        //schedule(follow(follower, parkonly()));
        schedule(autoRoutine());
    }
    @Override
    public void stop() {
        SharedObjects.autonomousEndPose = follower.pose(); //saves your position in that file
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        UpdateTelemetry();

    }
    private void UpdateTelemetry()
    {
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
        // Draw the Pedro pose on the dashboard field overlay (corner-origin -> field frame
        // conversion handled inside the renderer).
        if (follower != null && follower.pose() != null) {
            fieldRenderer.drawPedroPose(follower.pose());
        }
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                // Add mechanism commands here.
                parallel(
                        RunIntake,
                        follow(follower, path2())
                        ),
                StopIntake
        );
    }

    Command RunIntake = instant(() -> intake.runIntake());
    Command StopIntake = instant(() -> intake.stop());
    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }

    private Path scoreTopark(){
        return line(scorePose, parkPose).linear(scorePose, parkPose);
    }
    private Path parkonly() {
        return line(startPose, park).linear(startPose, park);
    }
    private Path parkFollowCurve() {
        return curve(startPose, controlPose, park).linear(startPose, park);
    }


    private final Pose start = poseFactory.of(55, 9, 90);
    private final Pose path1 = poseFactory.of(36.0, 9.75, 60);
    private final Pose point2 = poseFactory.of(36.0, 27.0, 0);

    public Path path1() {
        return line(start, path1).linear(path1,start );//why is linear backwards !?
    }

    public Path path2() {
        return line(path1, point2).linear(point2,path1);//why is linear backwards !?
    }
}