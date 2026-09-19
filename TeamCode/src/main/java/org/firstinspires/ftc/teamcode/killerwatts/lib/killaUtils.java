package org.firstinspires.ftc.teamcode.killerwatts.lib;

public class killaUtils {

    public static double clamp(double v, double lo, double hi) {
        return Math.min(hi, Math.max(lo, v));
    }
}
