package org.firstinspires.ftc.teamcode.killerwatts.lib;

import java.util.ArrayList;
import java.util.List;

public abstract class SubsystemBase {
    /*
    subsystem list
     */
    public static List<SubsystemBase> subsystemBaseList = new ArrayList<>();
    /*
    Runs all registered subsystems run commands.
     */
    public static void RunPeriodic()
    {
        // Copy to tolerate a subsystem registering late mid-loop.
        for(SubsystemBase item:new ArrayList<>(subsystemBaseList))
        {
            item.Periodic();
        }
    }

    /** Call once at the top of runOpMode so re-runs don't double-register. */
    public static void clearAll() {
        subsystemBaseList.clear();
    }
    public SubsystemBase()
    {
        subsystemBaseList.add(this);
    }

    public abstract void Periodic();
}
