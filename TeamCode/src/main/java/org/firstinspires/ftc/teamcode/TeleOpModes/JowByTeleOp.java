package org.firstinspires.ftc.teamcode.TeleOpModes;

import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.RunPeriodic;
import static org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase.clearAll;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SharedObjects;
import org.firstinspires.ftc.teamcode.killerwatts.ContinuousServo;
import org.firstinspires.ftc.teamcode.killerwatts.Drivetrain;
import org.firstinspires.ftc.teamcode.killerwatts.FieldTracker;
import org.firstinspires.ftc.teamcode.killerwatts.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.killerwatts.Intake;
import org.firstinspires.ftc.teamcode.killerwatts.PositionalServo;
import org.firstinspires.ftc.teamcode.killerwatts.lib.ALLIANCE_COLOR;
import org.firstinspires.ftc.teamcode.killerwatts.lib.DashboardFieldRenderer;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Red-JowByTeleOp", group = "TeleOp")

public class JowByTeleOp extends OpMode {
    public ALLIANCE_COLOR CurrentAlliance = ALLIANCE_COLOR.ALLIANCE_RED;
    //private GyroMecanumDriveTrain DamDrive;
    //private ServoMove servermovetest;
    private Drivetrain DT;
    private GoBildaPinpoint PinOdo;
    private FieldTracker tracker;
    private Intake intake;
    private PositionalServo PosServ;
    private ContinuousServo ContServ;
    private Follower follower;// = SharedObjects.follower;
    private final DashboardFieldRenderer fieldRenderer = new DashboardFieldRenderer();
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        fieldRenderer.drawPedroPose(follower.pose());
        clearAll(); // avoid double-registration on re-run
        //DamDrive = new GyroMecanumDriveTrain(this,)
        //build subsystems — order matters: pinpoint before tracker (tracker reads pinpoint.pos).
        //servermovetest = new ServoMove();
        //DT = new Drivetrain(this);
        //PinOdo = new GoBildaPinpoint(this);
        //tracker = new FieldTracker();
        intake = new Intake(this);
        //tracker.setPinpoint(PinOdo);
        // Heading assist if pinpoint drops: Control-Hub IMU yaw (radians CCW+).
        //tracker.setImuYawSupplier(() ->DT.GMDT.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        //DT.setFieldTracker(tracker);

        PosServ = new PositionalServo(this);
    }
    public void PedroFollowerFieldCentricDrivetrainloop()
    {
        if(CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_RED) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            );
            follower.manual(powers);
        }
        else{//we are on blue alliance
            DrivePowers powers = ManualDrive.fieldCentric(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    follower.pose().heading()
            );
            follower.manual(powers);
        }


        // relocalise button
        if (gamepad1.startWasPressed()) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose
        }

        follower.update();
        fieldRenderer.drawPedroPose(follower.pose());
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }
    @Override
    public void start() {
        follower.setPose(SharedObjects.autonomousEndPose);
        follower.update();
    }

    @Override
    public void loop() {
        RunPeriodic();
        PedroFollowerFieldCentricDrivetrainloop();
        UpdateTelemetry();
    }
    public void UpdateTelemetry()
    {
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
