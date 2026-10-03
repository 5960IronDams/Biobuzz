package org.firstinspires.ftc.teamcode.WattageLib.lib;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Circle;
import com.bylazar.field.Drawable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.Line;
import com.bylazar.field.PanelsField;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.WattageLib.pedroHelpers.PedroFieldBridge;

import java.util.List;

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

    // ---- Raw vision overlay config (same Panels page as the robot colors) ----
    // Toggles for the raw Limelight solves drawn by drawFusedAndVision().
    // MT1 = independent 6DOF solve (yellow), MT2 = gyro-seeded solve (green).
    // Poses arrive already in Pedro frame via Vision.toPedroPose(); this class
    // only draws them, same body-circle + heading-tick style as the robot.
    /** Draw the raw MT1 Pedro pose (yellow) in drawFusedAndVision(). */
    public static boolean DRAW_MT1_RAW = true;
    /** Draw the raw MT2 Pedro pose (green) in drawFusedAndVision(). */
    public static boolean DRAW_MT2_RAW = true;
    public static String MT1_STROKE = "yellow";
    public static String MT1_FILL = "rgba(255,255,0,0.25)";
    public static String MT2_STROKE = "green";
    public static String MT2_FILL = "rgba(0,255,0,0.25)";

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
        // Purge any poisoned items left by an older build so the first
        // post-deploy send starts from a clean canvas.
        try {
            field.getCanvas().reset();
        } catch (Exception ignored) {
        }
    }

    /**
     * Canonical entry point: FIRST field frame (center-origin, -72..+72in,
     * H = 0 faces +Y, CCW+). Converted to Pedro-native before drawing via
     * {@link PedroFieldBridge#fieldToPedro}.
     */
    public void drawFieldPose(double xFieldIn, double yFieldIn, double headingFieldRad) {
        Pose pedro = PedroFieldBridge.fieldToPedro(xFieldIn, yFieldIn, headingFieldRad);
        sendPedroPacket(pedro.x(), pedro.y(), pedro.heading(), getStrokebyAlliance(), getFillbyAlliance());
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
     * rotation and Y-flip in the web UI. Queues robot body + tick; caller must
     * call {@link #flush()} once per loop (single canvas send, throttled by
     * the plugin to ~10Hz — see batching note below).
     */
    public void drawPedroPose(Pose pedroPose) {
        if (pedroPose == null) return;
        sendPedroPacket(pedroPose.x(), pedroPose.y(), pedroPose.heading(),
                getStrokebyAlliance(), getFillbyAlliance());
    }

    /**
     * Fused robot + raw MegaTag solves in ONE canvas flush (no trails, no
     * per-frame throttle loss). Draw order: fused body (alliance colors),
     * then MT1 raw (yellow) and MT2 raw (green) when non-null and their
     * DRAW_* toggle is on. All three are Pedro-frame; null raw poses are
     * skipped so fusion-off / no-solve loops just draw the robot.
     *
     * <p>This is the preferred loop call when vision is present — replaces
     * separate drawPedroPose() calls. Get the raw pedro poses from
     * {@code VisionFusion.lastMt1Pedro()/lastMt2Pedro()} (already
     * Pedro-mapped via Vision.toPedroPose).
     */
    public void drawFusedAndVision(Pose fusedPedro, Pose mt1Pedro, Pose mt2Pedro) {
        // Per-shape guards: one bad ghost (NaN/Inf from a marginal solve) must
        // never poison the whole canvas. FieldManager serializes the canvas to
        // JSON for the web UI — a single NaN coordinate can fail the send (or
        // break SVG rendering), and since update() resets every loop the failure
        // would look exactly like a freeze at the last good frame. So each
        // shape is validated independently; bad ones are skipped, good ones
        // still paint.
        if (isDrawablePose(fusedPedro)) {
            try {
                queuePoseShape(fusedPedro.x(), fusedPedro.y(), fusedPedro.heading(),
                        getStrokebyAlliance(), getFillbyAlliance());
            } catch (Exception ignored) {
            }
        }
        if (DRAW_MT1_RAW && isDrawablePose(mt1Pedro)) {
            try {
                queuePoseShape(mt1Pedro.x(), mt1Pedro.y(), mt1Pedro.heading(),
                        MT1_STROKE, MT1_FILL);
            } catch (Exception ignored) {
            }
        }
        if (DRAW_MT2_RAW && isDrawablePose(mt2Pedro)) {
            try {
                queuePoseShape(mt2Pedro.x(), mt2Pedro.y(), mt2Pedro.heading(),
                        MT2_STROKE, MT2_FILL);
            } catch (Exception ignored) {
            }
        }
        try {
            flush();
        } catch (Exception ignored) {
        }
    }

    /**
     * Finite + on-field guard for a Pedro-frame pose. Rejects null, NaN/Inf
     * on any component, and poses far outside the 144in field (plus margin)
     * — an off-planet solve is a mapping bug, not something to paint.
     */
    private static boolean isDrawablePose(Pose p) {
        if (p == null) return false;
        double x;
        double y;
        double h;
        try {
            x = p.x();
            y = p.y();
            h = p.heading();
        } catch (Exception e) {
            return false;
        }
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(h)) return false;
        // 144in field + 48in margin: keeps a wild solve from painting across
        // the widget (or producing huge SVG coords) while tolerating the
        // FIELD_MARGIN_IN overshoot the fusion gates allow.
        return x >= -48.0 && x <= 192.0 && y >= -48.0 && y <= 192.0;
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
    private static String getStrokebyAlliance() {
        if (!COLOR_BY_ALLIANCE) return ROBOT_STROKE;
        return RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE ? BLUE_STROKE : RED_STROKE;
    }

    private static String getFillbyAlliance() {
        if (!COLOR_BY_ALLIANCE) return ROBOT_FILL;
        return RobotMain.CurrentAlliance == ALLIANCE_COLOR.ALLIANCE_BLUE ? BLUE_FILL : RED_FILL;
    }

    /**
     * Pedro-native draw: queue body circle + heading tick into the canvas and
     * flush immediately (single-shape loop call: robot only). Heading hp is
     * CCW+ measured from +X, so robot forward = (cos hp, sin hp). At hp=0 the
     * nose points +X (Pedro's convention). The old code used the field-frame
     * tick (-sin rh, cos rh) (0 = +Y) here, which is exactly +90deg CCW off in
     * this preset — the bug being fixed. The canvas transform (rotation /
     * flip in the Field widget) applies to both circle + tick endpoints, so
     * computing the nose in Pedro space is sufficient; do NOT pre-rotate.
     *
     * <p>Batching: FieldManager.update() only sends when 100ms elapsed since
     * the last send (field:1.0.6 default), then ALWAYS resets the canvas. So
     * N draw calls in one loop MUST be followed by exactly ONE update() —
     * separate draw-then-update per shape would drop all but the last shape
     * on most loops. Multi-shape callers: queue each shape with
     * {@link #queuePoseShape} and finish with one {@link #flush()} (see
     * {@link #drawFusedAndVision}).
     */
    private void sendPedroPacket(double xPedroIn, double yPedroIn, double headingPedroRad,
                                 String stroke, String fill) {
        queuePoseShape(xPedroIn, yPedroIn, headingPedroRad, stroke, fill);
        flush();
    }

    /** Queue one body-circle + heading-tick shape; no canvas send. */
    private void queuePoseShape(double xPedroIn, double yPedroIn, double headingPedroRad,
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
    }

    /**
     * Single canvas send for this loop's queued shapes. FRESH FRAME EVERY
     * LOOP: each loop queues a complete frame (robot + ghosts) then sends.
     *
     * <p>Throttle note (verified in field:1.0.6 bytecode): update() SENDS at
     * most every canvasUpdateInterval (100ms default) but ALWAYS resets the
     * canvas — throttled loops silently drop their frame. That is fine and
     * intended: the next send window paints the then-current pose at ~10Hz.
     * No backlog is kept on purpose. Replaying stale shapes across loops
     * accumulates items unboundedly (2 per robot-only loop, 6 with ghosts —
     * which is why the widget froze exactly when tags first appeared) until
     * the payload chokes the send and the display sticks at the last good
     * frame.
     *
     * <p>A single NaN/Inf coordinate anywhere in the canvas poisons the JSON
     * send, so validate first; on failure reset and drop the frame.
     */
    public void flush() {
        try {
            validateCanvas();
        } catch (Exception e) {
            try {
                field.getCanvas().reset();
            } catch (Exception ignored) {
            }
            return;
        }
        try {
            field.update();
        } catch (Exception e) {
            try {
                field.getCanvas().reset();
            } catch (Exception ignored) {
            }
        }
    }

    /** Throw when any queued shape carries a non-finite coordinate. */
    private void validateCanvas() {
        List<Drawable> items;
        try {
            items = field.getCanvas().getItems();
        } catch (Exception e) {
            return;
        }
        if (items == null) return;
        for (Drawable d : items) {
            if (d instanceof Circle) {
                Circle c = (Circle) d;
                if (!Double.isFinite(c.getX()) || !Double.isFinite(c.getY())
                        || !Double.isFinite(c.getR())) {
                    throw new IllegalStateException("non-finite circle");
                }
            } else if (d instanceof Line) {
                Line l = (Line) d;
                if (!Double.isFinite(l.getX1()) || !Double.isFinite(l.getY1())
                        || !Double.isFinite(l.getX2()) || !Double.isFinite(l.getY2())) {
                    throw new IllegalStateException("non-finite line");
                }
            }
        }
    }
}
