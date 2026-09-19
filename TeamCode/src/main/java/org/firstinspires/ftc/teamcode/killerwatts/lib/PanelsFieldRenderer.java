package org.firstinspires.ftc.teamcode.killerwatts.lib;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.pedro.PedroFieldBridge;

/**
 * Pose-agnostic Panels field overlay renderer.
 *
 * <p>Drop-in replacement for the old FTC Dashboard {@code DashboardFieldRenderer}:
 * same entry points, but poses are drawn on the Panels Field widget
 * ({@code http://192.168.43.1:8080} -&gt; Field) instead of an FTC Dashboard
 * {@code TelemetryPacket} field overlay. The BIOBUZZ background comes from
 * {@code BiobuzzFieldConfig} + the field plugin, so this class only draws the
 * robot (body circle + heading tick) and nothing else — no Pinpoint, no IMU,
 * no Kalman filter. Feed it poses from any source:</p>
 * <ul>
 *   <li>{@link #drawPedroPose} — Pedro {@link Pose} (corner origin 0..144,
 *       H = 0 faces +X, CCW+). This is what actually gets drawn: the Field
 *       widget is pinned to the PEDRO_PATHING preset, which already applies
 *       the -72in shift, 90deg rotation and Y-flip in the web UI.</li>
 *   <li>{@link #drawFieldPose} — canonical FIRST field frame (X = left/right
 *       from the red wall, Y = forward/back up-field, H = CCW+ from +Y,
 *       inches, origin at field center). Converted to Pedro-native via
 *       {@link PedroFieldBridge} before drawing.</li>
 *   <li>{@link #drawFtcPose} — FTC {@link Pose2D} in inches, same frame as
 *       {@link #drawFieldPose}.</li>
 * </ul>
 *
 * <p>Do NOT pre-convert Pedro poses (-72 / -90deg) before calling
 * {@link #drawPedroPose}: that double-applies the preset transform and
 * reintroduces the 90deg heading offset.</p>
 *
 * <p>Usage: keep one instance in your OpMode/subsystem and call a draw method
 * every loop, e.g. {@code renderer.drawPedroPose(follower.pose())}.</p>
 */
@Configurable
public class PanelsFieldRenderer {

    // ---- Field overlay config (Panels > Configurables > PanelsFieldRenderer) ----
    // Pedro-native: (72,72,90deg) == field center facing up-field. Leave at 0;
    // this is a fine-trim in Pedro inches, not a frame conversion.
    public static double ORIGIN_OFFSET_X = 0;
    public static double ORIGIN_OFFSET_Y = 0;
    /** Robot body circle radius, inches (14in robot => r = 7). */
    public static double ROBOT_RADIUS_IN = 7;
    /** Outline width for the body circle + heading tick. */
    public static double OUTLINE_WIDTH = 2;
    /**
     * When true (default), the robot body follows {@link RobotMain#CurrentAlliance}:
     * red alliance -> RED_STROKE/RED_FILL, blue -> BLUE_STROKE/BLUE_FILL.
     * Set false to force the manual ROBOT_STROKE/ROBOT_FILL instead.
     */
    public static boolean COLOR_BY_ALLIANCE = true;
    /** Manual fallback fill when COLOR_BY_ALLIANCE is false. */
    public static String ROBOT_FILL = "rgba(0,0,255,0.25)";
    /** Manual fallback stroke when COLOR_BY_ALLIANCE is false. */
    public static String ROBOT_STROKE = "blue";
    public static String RED_STROKE = "red";
    public static String RED_FILL = "rgba(255,0,0,0.25)";
    public static String BLUE_STROKE = "blue";
    public static String BLUE_FILL = "rgba(0,0,255,0.25)";

    private final FieldManager field = PanelsField.INSTANCE.getField();

    public PanelsFieldRenderer() {
        // Pedro-native mode: the Field widget's PEDRO_PATHING preset already
        // applies the corner-origin shift (offset -72,-72), the 90deg canvas
        // rotation and the Y-flip in the web UI (see field-1.0.6 svelte.js
        // getItemTransform). So we feed raw Pedro coords (0..144, H = 0 faces
        // +X, CCW+) and do NO manual -72 / -90deg conversion here. Pin the
        // preset so a stale web-UI selection (e.g. PANELS center-origin) can't
        // silently reintroduce a double-shift.
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * Canonical entry point: FIRST field frame (center-origin, -72..+72in,
     * H = 0 faces +Y, CCW+). Converted to Pedro-native before drawing via
     * {@link PedroFieldBridge#fieldToPedro}.
     */
    public void drawFieldPose(double xFieldIn, double yFieldIn, double headingFieldRad) {
        Pose pedro = PedroFieldBridge.fieldToPedro(xFieldIn, yFieldIn, headingFieldRad);
        sendPedroPacket(pedro.x(), pedro.y(), pedro.heading(), resolveStroke(), resolveFill());
    }

    /** Field frame with custom stroke/fill (e.g. different color per auto path). */
    public void drawFieldPose(double xFieldIn, double yFieldIn, double headingFieldRad,
                              String stroke, String fill) {
        Pose pedro = PedroFieldBridge.fieldToPedro(xFieldIn, yFieldIn, headingFieldRad);
        sendPedroPacket(pedro.x(), pedro.y(), pedro.heading(), stroke, fill);
    }

    /** FTC Pose2D in inches (field frame, same contract as FieldTracker). */
    public void drawFtcPose(Pose2D fieldPoseInches) {
        if (fieldPoseInches == null) return;
        drawFieldPose(
                fieldPoseInches.getX(DistanceUnit.INCH),
                fieldPoseInches.getY(DistanceUnit.INCH),
                fieldPoseInches.getHeading(AngleUnit.RADIANS));
    }

    /**
     * Pedro Pose (corner origin 0..144, H = 0 faces +X, CCW+). Drawn as-is:
     * the PEDRO_PATHING preset already applies the -72in shift, 90deg canvas
     * rotation and Y-flip in the web UI.
     */
    public void drawPedroPose(Pose pedroPose) {
        if (pedroPose == null) return;
        sendPedroPacket(pedroPose.x(), pedroPose.y(), pedroPose.heading(),
                resolveStroke(), resolveFill());
    }

    /** Back-compat alias for drawFieldPose (field frame, not Pedro frame). */
    public void drawRobot(double xFieldIn, double yFieldIn, double headingFieldRad) {
        drawFieldPose(xFieldIn, yFieldIn, headingFieldRad);
    }

    /**
     * Alliance-aware colors, read live from {@link RobotMain#CurrentAlliance} so a
     * mid-session alliance switch takes effect on the next draw. Explicit
     * stroke/fill overloads bypass this and draw exactly what was passed.
     */
    private static String resolveStroke() {
        if (!COLOR_BY_ALLIANCE) return ROBOT_STROKE;
        return RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE ? BLUE_STROKE : RED_STROKE;
    }

    private static String resolveFill() {
        if (!COLOR_BY_ALLIANCE) return ROBOT_FILL;
        return RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE ? BLUE_FILL : RED_FILL;
    }

    /**
     * Pedro-native draw. Heading hp is CCW+ measured from +X, so robot
     * forward = (cos hp, sin hp). At hp=0 the nose points +X (Pedro's
     * convention). The old code used the field-frame tick
     * (-sin rh, cos rh) (0 = +Y) here, which is exactly +90deg CCW off in
     * this preset — the bug being fixed. The canvas transform (rotation /
     * flip in the Field widget) applies to both circle + tick endpoints, so
     * computing the nose in Pedro space is sufficient; do NOT pre-rotate.
     */
    private void sendPedroPacket(double xPedroIn, double yPedroIn, double headingPedroRad,
                                 String stroke, String fill) {
        double rx = xPedroIn + ORIGIN_OFFSET_X;
        double ry = yPedroIn + ORIGIN_OFFSET_Y;

        double fx = Math.cos(headingPedroRad);
        double fy = Math.sin(headingPedroRad);
        double noseX = rx + ROBOT_RADIUS_IN * fx;
        double noseY = ry + ROBOT_RADIUS_IN * fy;

        // Body: circle (rotation-invariant, so no rect-rotation math needed).
        field.moveCursor(rx, ry);
        field.setFill(fill);
        field.setOutline(stroke, OUTLINE_WIDTH);
        field.circle(ROBOT_RADIUS_IN);
        // Heading tick: center -> nose.
        field.moveCursor(rx, ry);
        field.setOutline(stroke, OUTLINE_WIDTH);
        field.line(noseX, noseY);

        field.update();
    }
}
