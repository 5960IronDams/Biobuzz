package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.CommandFactory;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.turret.TurretPivot;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

@Autonomous(name = "ZDiagnostics", group = "Autos")
public class ZDiagnostics extends AutoOpModeBase {

    // ---- Compile-time configurable delays & durations (ms) ----
    private static final long STEP_DELAY_MS = 500;
    private static final long HOOD_TIMEOUT_MS = 3000;
    private static final long TURRET_TIMEOUT_MS = 4000;
    private static final long RPM_TIMEOUT_MS = 5000;
    private static final long SPOOL_DOWN_WAIT_MS = 5000;
    private static final long RESERVOIR_RUN_MS = 4000;
    private static final long INTAKE_RUN_MS = 4000;
    private static final long RAMP_WAIT_MS = 3000;

    public ZDiagnostics() {
        super.assetPath = "pathfiles/exampleAuto1.pp.json";
        super.SetRobotToThisColor = ALLIANCE_COLOR.ALLIANCE_RED;
    }

    @Override
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                // 1. Hood: all the way up, down, up, down
                robot.hood.toAngleCmd(Hood.HOOD_UP_DEG).withTimeout(HOOD_TIMEOUT_MS),
                new WaitCommand(STEP_DELAY_MS),
                robot.hood.toAngleCmd(Hood.HOOD_DOWN_DEG).withTimeout(HOOD_TIMEOUT_MS),
                new WaitCommand(STEP_DELAY_MS),
                robot.hood.toAngleCmd(Hood.HOOD_UP_DEG).withTimeout(HOOD_TIMEOUT_MS),
                new WaitCommand(STEP_DELAY_MS),
                robot.hood.toAngleCmd(Hood.HOOD_DOWN_DEG).withTimeout(HOOD_TIMEOUT_MS),
                new WaitCommand(STEP_DELAY_MS),

                // 2. Turret: CCW max, CW min, back to 0 (directly forward)
                robot.turret.C_GotoPositon(TurretPivot.MAX_ANGLE_DEG).withTimeout(TURRET_TIMEOUT_MS),
                new WaitCommand(STEP_DELAY_MS),
                robot.turret.C_GotoPositon(TurretPivot.MIN_ANGLE_DEG).withTimeout(TURRET_TIMEOUT_MS),
                new WaitCommand(STEP_DELAY_MS),
                robot.turret.C_GotoPositon(0.0).withTimeout(TURRET_TIMEOUT_MS),
                new WaitCommand(STEP_DELAY_MS),

                // 3. Flywheel: closest LUT speed, highest LUT speed, relax 5s to spool down
                robot.flywheel.toRpm(getClosestLutRpm()),
                new WaitUntilCommand(() -> robot.flywheel.isAtTargetRpm()).withTimeout(RPM_TIMEOUT_MS),
                new WaitCommand(STEP_DELAY_MS),
                robot.flywheel.toRpm(getHighestLutRpm()),
                new WaitUntilCommand(() -> robot.flywheel.isAtTargetRpm()).withTimeout(RPM_TIMEOUT_MS),
                new WaitCommand(STEP_DELAY_MS),
                robot.flywheel.runOnce(robot.flywheel::coastToIdle),
                new WaitCommand(SPOOL_DOWN_WAIT_MS),

                // 4. Reservoir (Transfer): run for 4 seconds, then stop
                robot.transfer.runCmd(),
                new WaitCommand(RESERVOIR_RUN_MS),
                robot.transfer.stopCmd(),
                new WaitCommand(STEP_DELAY_MS),

                // 5. Intake: run for 4 seconds, then stop
                robot.intake.runCmd(),
                new WaitCommand(INTAKE_RUN_MS),
                robot.intake.stopCmd(),
                new WaitCommand(STEP_DELAY_MS),

                // 6. Ramp: up then down (with 3 second waits for travel time)
                rampUp(),
                new WaitCommand(RAMP_WAIT_MS),
                rampDown(),
                new WaitCommand(RAMP_WAIT_MS),

                // Release turret hold when finished
                robot.turret.C_Stop()
        );
    }

    /** RPM corresponding to the closest distance entry in CommandFactory's LUT. */
    private static double getClosestLutRpm() {
        double[] dists = CommandFactory.LUT_DISTANCES_IN;
        double[] rpms = CommandFactory.LUT_RPMS;
        if (rpms == null || rpms.length == 0) {
            return CommandFactory.MANUAL_SHOOT_RPM;
        }
        int count = dists != null ? Math.min(dists.length, rpms.length) : 0;
        if (count == 0) {
            return rpms[0];
        }
        int minIdx = 0;
        for (int i = 1; i < count; i++) {
            if (dists[i] < dists[minIdx]) {
                minIdx = i;
            }
        }
        return rpms[minIdx];
    }

    /** Highest RPM entry in CommandFactory's LUT. */
    private static double getHighestLutRpm() {
        double[] rpms = CommandFactory.LUT_RPMS;
        if (rpms == null || rpms.length == 0) {
            return CommandFactory.MANUAL_SHOOT_RPM;
        }
        double maxRpm = rpms[0];
        for (int i = 1; i < rpms.length; i++) {
            if (rpms[i] > maxRpm) {
                maxRpm = rpms[i];
            }
        }
        return maxRpm;
    }
}
