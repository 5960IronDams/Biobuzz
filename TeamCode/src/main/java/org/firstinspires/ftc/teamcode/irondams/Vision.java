package org.firstinspires.ftc.teamcode.irondams;

import androidx.annotation.Nullable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Vision {

    private final Limelight3A limelight;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.getAll(Limelight3A.class).get(0);
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void start() {
        start(0);
    }

    public void start(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    @Nullable
    public Pose3D getBotpose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }

    @Nullable
    public Pose3D updateRobotOrientation(YawPitchRollAngles orientation) {
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose_MT2();
        }
        return null;
    }
}
