package org.firstinspires.ftc.teamcode.killerwatts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.killerwatts.lib.FieldPoseMapper;
import org.firstinspires.ftc.teamcode.killerwatts.lib.PoseKalmanFilter;
import org.firstinspires.ftc.teamcode.killerwatts.lib.SubsystemBase;
import org.firstinspires.ftc.teamcode.killerwatts.lib.VisionMeasurement;

import java.util.Locale;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;

/**
 * Field-frame pose owner for the whole robot. See {@link FieldPoseMapper} for
 * the frame contract: everything in/out of here is FIRST field frame
 * (X = left/right from the red wall, Y = forward/back up-field, H = CCW+,
 * inches, origin at field center).
 *
 * <p>Fuses (in priority order):
 * <ol>
 *   <li>goBILDA Pinpoint odometry — read as a delta from its boot zero
 *       (robot frame), rotated once by the start heading into field frame</li>
 *   <li>Control-Hub IMU yaw (optional, via {@link #setImuYawSupplier}) — heading assist
 *       when Pinpoint is missing/unplugged</li>
 *   <li>Vision (future Limelight 3A) via {@link #addVisionMeasurement} — absolute
 *       field-frame corrections fused with per-sample std-dev through the Kalman filter</li>
 * </ol>
 * Default pose is 0,0,0 (field center, facing +Y up-field). Dashboard shows the fused
 * pose on the field overlay every {@link #Periodic()} call.
 *
 * <p>Downstream path followers (Road Runner, Pedro) must consume
 * {@link #getPose2D()} / {@link #getXIn()} / {@link #getYIn()} /
 * {@link #getHeadingRad()} directly — never Pinpoint's raw robot-frame pose.</p>
 */
@Config
public class FieldTracker extends SubsystemBase {

    // ---- Field overlay config ----
    // Pose (0,0,0) == dashboard field center. Leave offsets at 0 so the robot
    // overlay starts in the middle of the field; auton/vision seed the start pose.
    public static double ORIGIN_OFFSET_X = 0;
    public static double ORIGIN_OFFSET_Y = 0;
    public static boolean RED_ALLIANCE = true;
    // Custom field background served from TeamCode/src/main/assets/images/.
    // webp is browser-supported; biobuzz-field.png is the fallback if webp fails.
    public static String FIELD_IMAGE = "/images/biobuzz-field.webp";
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

    // ---- Pose filter tuning (dashboard-tunable) ----
    // Start pose = where the robot physically sits at init, in FIELD frame.
    // Auton will set these (or Limelight will seed them); Pinpoint's own zero
    // is captured automatically as the delta reference, never hand-entered.
    public static double START_X_IN = 0;
    public static double START_Y_IN = 0;
    // 0 deg = facing +Y (up-field, away from red wall), CCW+.
    public static double START_HEADING_DEG = 0;
    /** Set true if the Pinpoint yaw sign disagrees with field CCW+ (mount/IMU
     * dependent). Flip from the dashboard: nose hash should point AWAY from
     * the red wall at heading 0; if it points AT the red wall, toggle this.
     * Changing this forces the Pinpoint zero to re-capture next loop. */
    public static boolean PINPOINT_INVERT_HEADING = false;
    /** Pinpoint trust: lower = trust pinpoint more. Inches, 1-sigma. */
    public static double PINPOINT_XY_STD_IN = 0.75;
    /** Pinpoint heading trust: radians, 1-sigma. ~2 deg default. */
    public static double PINPOINT_HEADING_STD_DEG = 2.0;
    /** IMU heading trust used when pinpoint is absent. Degrees, 1-sigma. */
    public static double IMU_HEADING_STD_DEG = 3.0;
    /** Process noise: how fast we allow the estimate to drift between updates. */
    public static double PROCESS_XY_STD_IN_PER_SEC = 0.5;
    public static double PROCESS_HEADING_STD_DEG_PER_SEC = 2.0;
    /** Dead-reckoning velocity when no pinpoint (in/sec). Set from drivetrain if desired. */
    public static double FALLBACK_VX_IN_PER_SEC = 0;
    public static double FALLBACK_VY_IN_PER_SEC = 0;
    public static double FALLBACK_OMEGA_DEG_PER_SEC = 0;
    public static double ROBOT_LENGTH_IN = 14;
    public static double ROBOT_WIDTH_IN = 14;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry telemetry = dashboard.getTelemetry();

    private final PoseKalmanFilter filter = new PoseKalmanFilter(
            START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG),
            1.0, Math.toRadians(5.0),
            PROCESS_XY_STD_IN_PER_SEC, Math.toRadians(PROCESS_HEADING_STD_DEG_PER_SEC));

    private GoBildaPinpoint pinpoint;
    private Supplier<Double> imuYawRadSupplier; // CCW+ radians, field-relative
    private final Queue<VisionMeasurement> visionQueue = new ConcurrentLinkedQueue<>();

    // Manual velocity injection (e.g. from Drivetrain when pinpoint unplugged).
    private volatile double manualVx, manualVy, manualOmega;

    private long lastNano = -1;
    private double lastStartX = START_X_IN;
    private double lastStartY = START_Y_IN;
    private double lastStartH = START_HEADING_DEG;
    private boolean lastInvertH = PINPOINT_INVERT_HEADING;

    // Pinpoint boot-zero reference (robot frame, inches + rad). Captured on the
    // first Periodic with valid data so later reads are start-anchored deltas.
    private boolean zeroCaptured = false;
    private double zeroFwdIn, zeroLeftIn, zeroH;

    /** Field-frame pose as Pose2D (inches, radians) for path followers. */
    public Pose2D getPose2D() {
        return FieldPoseMapper.fieldInToPose2D(getXIn(), getYIn(), getHeadingRad());
    }

    public FieldTracker() { }

    /** Attach after construction. Call before first Periodic; order matters. */
    public void setPinpoint(GoBildaPinpoint pinpoint) {
        this.pinpoint = pinpoint;
    }

    /** Optional IMU yaw supplier (radians CCW+). Used when pinpoint is null/faulted. */
    public void setImuYawSupplier(Supplier<Double> yawRadSupplier) {
        this.imuYawRadSupplier = yawRadSupplier;
    }

    /** Future Limelight 3A path: enqueue an absolute field measurement (inches, rad). */
    public void addVisionMeasurement(VisionMeasurement m) {
        if (m != null) visionQueue.offer(m);
    }

    /** Convenience overload for vision sources reporting degrees. */
    public void addVisionMeasurementInDeg(double xIn, double yIn, double headingDeg,
                                          double xyStdIn, double headingStdDeg, String source) {
        visionQueue.offer(new VisionMeasurement(xIn, yIn, Math.toRadians(headingDeg),
                xyStdIn, Math.toRadians(headingStdDeg), System.nanoTime(), source));
    }

    /** Drivetrain can push its commanded/robot-relative velocity here as fallback. */
    public void noteVelocity(double vxInPerSec, double vyInPerSec, double omegaRadPerSec) {
        manualVx = vxInPerSec;
        manualVy = vyInPerSec;
        manualOmega = omegaRadPerSec;
    }

    /** Reset fused pose to the dashboard START_* values; re-captures Pinpoint zero. */
    public void resetPose() {
        filter.reset(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG),
                PINPOINT_XY_STD_IN, Math.toRadians(PINPOINT_HEADING_STD_DEG));
        visionQueue.clear();
        zeroCaptured = false;
        lastNano = -1;
    }

    /** Teleport fused pose (e.g. known AprilTag start). Inches + degrees. */
    public void setPose(double xIn, double yIn, double headingDeg) {
        filter.reset(xIn, yIn, Math.toRadians(headingDeg),
                PINPOINT_XY_STD_IN, Math.toRadians(PINPOINT_HEADING_STD_DEG));
    }

    public double getXIn() { return filter.getX(); }
    public double getYIn() { return filter.getY(); }
    public double getHeadingRad() { return filter.getHeading(); }
    public double getHeadingDeg() { return Math.toDegrees(filter.getHeading()); }

    @Override
    public void Periodic() {
        // Pick up dashboard edits to START_* / invert flag as a live reset.
        if (START_X_IN != lastStartX || START_Y_IN != lastStartY || START_HEADING_DEG != lastStartH
                || PINPOINT_INVERT_HEADING != lastInvertH) {
            lastStartX = START_X_IN;
            lastStartY = START_Y_IN;
            lastStartH = START_HEADING_DEG;
            lastInvertH = PINPOINT_INVERT_HEADING;
            resetPose();
        }

        long now = System.nanoTime();
        double dt = lastNano < 0 ? 0.02 : (now - lastNano) / 1e9;
        dt = Math.min(Math.max(dt, 0.0), 0.25); // clamp I2C hiccups
        lastNano = now;

        boolean havePinpoint = pinpoint != null && pinpoint.pos != null && pinpoint.vel != null;

        double startH = Math.toRadians(START_HEADING_DEG);
        if (havePinpoint) {
            double[] delta = FieldPoseMapper.pinpointPoseToDeltaIn(pinpoint.pos);
            double[] vDelta = FieldPoseMapper.pinpointVelToDelta(pinpoint.vel);
            // Some mounts/IMUs report yaw CW+ or with a boot offset; normalize
            // once so downstream math stays pure field-frame CCW+.
            double hSign = PINPOINT_INVERT_HEADING ? -1.0 : 1.0;
            double hNow = hSign * delta[2];
            double wNow = hSign * vDelta[2];
            if (!zeroCaptured) {
                zeroFwdIn = delta[0];
                zeroLeftIn = delta[1];
                zeroH = hNow;
                zeroCaptured = true;
                filter.reset(START_X_IN, START_Y_IN, startH,
                        PINPOINT_XY_STD_IN, Math.toRadians(PINPOINT_HEADING_STD_DEG));
            }
            double dFwd = delta[0] - zeroFwdIn;
            double dLeft = delta[1] - zeroLeftIn;
            double dH = PoseKalmanFilter.angleDiff(hNow, zeroH);
            double[] field = FieldPoseMapper.pinpointDeltaToField(
                    dFwd, dLeft, dH, START_X_IN, START_Y_IN, startH);
            double[] fieldVel = FieldPoseMapper.robotVelToField(vDelta[0], vDelta[1], startH);

            // Predict with field-frame velocity, then correct with field-frame absolute.
            filter.predict(fieldVel[0], fieldVel[1], wNow, dt);
            filter.updateXY(field[0], field[1], PINPOINT_XY_STD_IN);
            filter.updateHeading(field[2], Math.toRadians(PINPOINT_HEADING_STD_DEG));
        } else {
            // No odometry: hold with manual/fallback velocity + optional IMU heading.
            double vx = manualVx != 0 || manualVy != 0 || manualOmega != 0
                    ? manualVx : FALLBACK_VX_IN_PER_SEC;
            double vy = manualVx != 0 || manualVy != 0 || manualOmega != 0
                    ? manualVy : FALLBACK_VY_IN_PER_SEC;
            double om = manualVx != 0 || manualVy != 0 || manualOmega != 0
                    ? manualOmega : Math.toRadians(FALLBACK_OMEGA_DEG_PER_SEC);
            filter.predict(vx, vy, om, dt);
            if (imuYawRadSupplier != null) {
                try {
                    // IMU yaw is a delta from ITS reset; anchor with the start heading.
                    filter.updateHeading(startH + imuYawRadSupplier.get(),
                            Math.toRadians(IMU_HEADING_STD_DEG));
                } catch (Exception ignored) { }
            }
        }

        // Fuse all pending vision measurements (Limelight 3A future path).
        VisionMeasurement vm;
        while ((vm = visionQueue.poll()) != null) {
            if (vm.xyStdIn > 0 && vm.xyStdIn < 1e6) {
                filter.updateXY(vm.xIn, vm.yIn, vm.xyStdIn);
            }
            if (vm.headingStdRad > 0 && vm.headingStdRad < 1e6) {
                filter.updateHeading(vm.headingRad, vm.headingStdRad);
            }
        }

        double rx = filter.getX();
        double ry = filter.getY();
        double rh = filter.getHeading();

        telemetry.addData("FieldTracker",
                String.format(Locale.US, "{X: %.2f, Y: %.2f, H: %.1f} in/deg",
                        rx, ry, Math.toDegrees(rh)));
        telemetry.addData("FieldTracker std",
                String.format(Locale.US, "{sx: %.2f, sy: %.2f, sh: %.1f}",
                        filter.getStdX(), filter.getStdY(), Math.toDegrees(filter.getStdH())));
        telemetry.addData("FieldTracker src", havePinpoint ? "pinpoint" : "dead-reckon/imu");
        telemetry.update();

        sendFieldPacket(rx, ry, rh);
    }

    private void sendFieldPacket(double rxIn, double ryIn, double headingRad) {
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
                        "8px serif", 0, false)
                // Fused robot pose (rectangle + heading tick, no text label).
                .setStroke("blue")
                .setFill("rgba(0,0,255,0.25)")
                .fillPolygon(px, py)
                .setStroke("blue")
                .strokeLine(rxIn, ryIn, noseX, noseY);

        dashboard.sendTelemetryPacket(packet);
    }
}
