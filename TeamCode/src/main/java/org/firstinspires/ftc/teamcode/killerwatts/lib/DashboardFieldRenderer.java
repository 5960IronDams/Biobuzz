package org.firstinspires.ftc.teamcode.killerwatts.lib;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.PedroFieldBridge;

/**
 * Pose-agnostic FTC Dashboard field overlay renderer.
 *
 * <p>Owns everything needed to draw a robot pose on the dashboard field view
 * (custom background image, origin/rotation/scale, axis legend, robot
 * rectangle + heading tick) and nothing else — no Pinpoint, no IMU, no Kalman
 * filter. Feed it poses from any source:</p>
 * <ul>
 *   <li>{@link #drawFieldPose} — canonical FIRST field frame (X = left/right
 *       from the red wall, Y = forward/back up-field, H = CCW+ from +Y,
 *       inches, origin at field center). This is what actually gets drawn.</li>
 *   <li>{@link #drawFtcPose} — FTC {@link Pose2D} in inches, same frame.</li>
 *   <li>{@link #drawPedroPose} — Pedro {@link Pose} (corner origin, H = 0
 *       faces +X); converted via {@link PedroFieldBridge}.</li>
 * </ul>
 *
 * <p>Usage: keep one instance in your OpMode/subsystem and call a draw method
 * every loop. FieldTracker delegates to this class; Pedro autos/teleop can use it directly with
 * {@code renderer.drawPedroPose(follower.pose())}.</p>
 */
@Config
public class DashboardFieldRenderer {

    // ---- Field overlay config (moved out of FieldTracker) ----
    // Pose (0,0,0) == dashboard field center. Leave offsets at 0 so the robot
    // overlay starts in the middle of the field; auton/vision seed the start pose.
    public static double ORIGIN_OFFSET_X = 0;
    public static double ORIGIN_OFFSET_Y = 0;
    public static boolean RED_ALLIANCE = true;
    // Custom field background served from TeamCode/src/main/assets/images/.
    // webp is browser-supported; biobuzz-field.png is the fallback if webp fails.
    public static String FIELD_IMAGE = "/images/biobuzz-field-pedro.png";
    public static boolean USE_CUSTOM_FIELD_IMAGE = true;
    // Rotation was baked into the asset itself (1080x1080 CCW) so the draw call
    // stays a plain full-field blit — dashboard's image theta/pivot convention
    // moved the blit off-center, this avoids that entirely.
    public static boolean DRAW_DEFAULT_FIELD = false;
    public static double SCALEX = 1.0;
    public static double SCALEY = 1.0;
    public static boolean DRAW_GRID = false;
    /** Axis legend anchor: inset (inches) from the -X/-Y field corner, which is
     * bottom-right as seen in the browser (field is 144x144in, corner at -72,-72). */
    public static double AXIS_LEGEND_MARGIN_IN = 4;
    public static double AXIS_LEGEND_LEN_IN = 16;
    public static double ROBOT_LENGTH_IN = 14;
    public static double ROBOT_WIDTH_IN = 14;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public DashboardFieldRenderer() { }

    /** Canonical entry point: field-frame inches + radians. */
    public void drawFieldPose(double xFieldIn, double yFieldIn, double headingFieldRad) {
        sendFieldPacket(xFieldIn, yFieldIn, headingFieldRad, "blue", "rgba(0,0,255,0.25)");
    }

    /** Canonical entry point with custom stroke/fill (e.g. different color per auto path). */
    public void drawFieldPose(double xFieldIn, double yFieldIn, double headingFieldRad,
                              String stroke, String fill) {
        sendFieldPacket(xFieldIn, yFieldIn, headingFieldRad, stroke, fill);
    }

    /** FTC Pose2D in inches (field frame, same contract as FieldTracker). */
    public void drawFtcPose(Pose2D fieldPoseInches) {
        if (fieldPoseInches == null) return;
        drawFieldPose(
                fieldPoseInches.getX(DistanceUnit.INCH),
                fieldPoseInches.getY(DistanceUnit.INCH),
                fieldPoseInches.getHeading(AngleUnit.RADIANS));
    }

    /** Pedro Pose (corner origin). Converted to field frame via PedroFieldBridge. */
    public void drawPedroPose(Pose pedroPose) {
        if (pedroPose == null) return;
        double[] xy = PedroFieldBridge.pedroToFieldXY(pedroPose);
        double headingField = PedroFieldBridge.pedroHeadingToField(pedroPose.heading());
        drawFieldPose(xy[0], xy[1], headingField);
    }

    /** Back-compat alias for drawFieldPose. */
    public void drawRobot(double xFieldIn, double yFieldIn, double headingFieldRad) {
        drawFieldPose(xFieldIn, yFieldIn, headingFieldRad);
    }

    private void sendFieldPacket(double rxIn, double ryIn, double headingRad,
                                 String stroke, String fill) {
        TelemetryPacket packet = new TelemetryPacket(DRAW_DEFAULT_FIELD);

        // Glyph geometry in FIELD frame: heading rh is CCW+ measured from +Y.
        // Robot forward = (-sin rh, cos rh), robot right = (cos rh, sin rh).
        // At rh=0: nose points +Y (up-field, away from red wall). Corners are
        // (fwd,right) combos so hl lies along the nose axis, hw across it.
        double fx = -Math.sin(headingRad);
        double fy = Math.cos(headingRad);
        double gx = Math.cos(headingRad);
        double gy = Math.sin(headingRad);
        double hl = ROBOT_LENGTH_IN / 2.0;
        double hw = ROBOT_WIDTH_IN / 2.0;
        double[] px = {
                rxIn + hl * fx + hw * gx, // front-right x
                rxIn + hl * fx - hw * gx, // front-left x
                rxIn - hl * fx - hw * gx, // back-left x
                rxIn - hl * fx + hw * gx, // back-right x
        };
        double[] py = {
                ryIn + hl * fy + hw * gy, // front-right y
                ryIn + hl * fy - hw * gy, // front-left y
                ryIn - hl * fy - hw * gy, // back-left y
                ryIn - hl * fy + hw * gy, // back-right y
        };
        // Heading tick: center -> nose (along robot forward).
        double noseX = rxIn + hl * fx;
        double noseY = ryIn + hl * fy;

        // Axis legend anchor: -X/-Y field corner (bottom-right in browser).
        double legX = -72 + AXIS_LEGEND_MARGIN_IN;
        double legY = -72 + AXIS_LEGEND_MARGIN_IN;

        packet.fieldOverlay()
                .setAlpha(1.0)
                .setStrokeWidth(1);
        if (USE_CUSTOM_FIELD_IMAGE && FIELD_IMAGE != null && !FIELD_IMAGE.isEmpty()) {
            // Plain full-field page-frame blit: 144x144in square, stays put.
            // (Rotation is pre-baked into the image file, not the draw call.)
            packet.fieldOverlay().drawImage(FIELD_IMAGE, 0, 0, 144, 144);
        }
        if (DRAW_GRID) {
            packet.fieldOverlay().drawGrid(0, 0, 144, 144, 7, 7);
        }
        packet.fieldOverlay()
                .setRotation(RED_ALLIANCE ? 0 : Math.PI)
                .setTranslation(ORIGIN_OFFSET_X, ORIGIN_OFFSET_Y * (RED_ALLIANCE ? -1 : 1))
                .setScale(SCALEX, SCALEY)
                // Axis legend in the field (centered) frame, anchored at the
                // -X/-Y corner = bottom-right as seen in the browser.
                // X line runs up the right edge, Y line runs left along the bottom.
                .setStroke("red")
                .strokeLine(legX, legY, legX + AXIS_LEGEND_LEN_IN, legY)
                .setFill("red")
                .fillText("X axis", legX + AXIS_LEGEND_LEN_IN / 2, legY + 5,
                        "8px Arial", 0, false)
                .setStroke("green")
                .strokeLine(legX, legY, legX, legY + AXIS_LEGEND_LEN_IN)
                .setFill("green")
                .fillText("Y axis", legX + 5, legY + AXIS_LEGEND_LEN_IN / 2,
                        "8px serif", 90, false)
                // Fused robot pose (rectangle + heading tick, no text label).
                .setStroke(stroke)
                .setFill(fill)
                .fillPolygon(px, py)
                .setStroke(stroke)
                .strokeLine(rxIn, ryIn, noseX, noseY);

        dashboard.sendTelemetryPacket(packet);
    }
}
