package org.firstinspires.ftc.teamcode.irondams.drivetrain;

public final class Acceleration {
    /**
     * Calculates the target motor power using a trapezoidal motion profile approach.
     * Smoothly ramps up power from minPower near the start position, maintains maxPower,
     * and ramps down towards minPower as it approaches the target to prevent stalling and overshoot.
     *
     * @param startPos   The starting encoder or coordinate position of the movement.
     * @param currentPos The current encoder or coordinate position of the robot.
     * @param targetPos  The final destination encoder or coordinate position.
     * @param accelZone  The distance over which the robot should accelerate up to maxPower.
     * @param decelZone  The distance over which the robot should decelerate down to minPower.
     * @param minPower   The baseline power required to overcome static friction and avoid stalling.
     * @param maxPower   The maximum allowable power for the movement.
     * @return The calculated absolute power to apply.
     */
    public static double getPower(double startPos, double currentPos, double targetPos,
                                  double accelZone, double decelZone,
                                  double minPower, double maxPower) {

        double totalDist = Math.abs(targetPos - startPos);
        double traveled = Math.abs(currentPos - startPos);
        double remainingDist = Math.abs(targetPos - currentPos);

        // Adjust zones proportionally if the total distance is too short to fully accelerate and decelerate
        double sumZones = accelZone + decelZone;
        if (sumZones > totalDist && totalDist > 0) {
            double scale = totalDist / sumZones;
            accelZone *= scale;
            decelZone *= scale;
        }

        double powerRange = maxPower - minPower;

        if (traveled < accelZone && accelZone > 0) {
            double percentage = traveled / accelZone;
            return minPower + percentage * powerRange;
        } else if (remainingDist < decelZone && decelZone > 0) {
            double percentage = remainingDist / decelZone;
            return minPower + percentage * powerRange;
        } else {
            return maxPower;
        }
    }

    /**
     * Slew-rate limiter to smoothly ramp teleoperated driver inputs over time.
     * Prevents wheel slippage, extreme battery voltage drops, and mechanical wear by
     * restricting sudden spikes or drops in requested power.
     *
     * @param currentPower   The motor power applied in the previous loop cycle.
     * @param requestedPower The raw joystick or driver command power input.
     * @param maxDeltaPerSec The maximum allowable change in power per single second (e.g., 2.0 allows 0 to 1.0 in 0.5s).
     * @param deltaTimeSec   The time elapsed in seconds since the last loop execution.
     * @return The rate-limited power output.
     */
    public static double rampPower(double currentPower, double requestedPower,
                                   double maxDeltaPerSec, double deltaTimeSec) {
        double maxDelta = maxDeltaPerSec * deltaTimeSec;
        double delta = requestedPower - currentPower;

        if (Math.abs(delta) > maxDelta) {
            delta = Math.signum(delta) * maxDelta;
        }

        return currentPower + delta;
    }
}