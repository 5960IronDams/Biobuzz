package org.firstinspires.ftc.teamcode.irondams.killerwatts.lib;

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
        for(SubsystemBase item:subsystemBaseList)
        {
            item.Periodic();
        }
    }
    public SubsystemBase()
    {
        subsystemBaseList.add(this);
    }

    public abstract void Periodic();
}
