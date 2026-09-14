package org.firstinspires.ftc.teamcode.irondams.drivetrain;

/**
 * Common abstraction interface representing an active robot drivetrain mechanism.
 * Allows switching seamlessly between different control perspectives (e.g. Robot-Centric vs. Field-Centric)
 * using the same unified vector input layout.
 */
public interface IDriveTrain {

    /**
     * Executes wheel-power calculations and updates motor states based on directional inputs.
     *
     * @param x    The horizontal lateral velocity/strafe command component, typically mapped from a joystick X-axis [-1.0, 1.0].
     * @param y    The vertical forward/reverse velocity command component, typically mapped from a joystick Y-axis [-1.0, 1.0].
     * @param turn The rotational angular steering command component, typically mapped from a secondary joystick X-axis [-1.0, 1.0].
     */
    void drive(double x, double y, double turn);
}
