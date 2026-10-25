package org.firstinspires.ftc.teamcode.Subsystems.turret;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IronConstants;

import java.util.Locale;

/**
 * Bench bring-up OpMode for {@link TurretPivot} — runs the turret alone (no
 * drivetrain/vision init), so wiring and tuning can be debugged without the
 * rest of the robot.
 *
 * <p>BOOT PROCEDURE (CRT dual-encoder boot solve):
 * <ol>
 *   <li>Wire the TRACKING encoder B's ABS wire (32T pinion) to OctoQuad
 *       channel 6 and encoder A's (31T pinion, CRT partner) to channel 7
 *       (bank2 pulse-width, already configured).</li>
 *   <li>Select this OpMode and press INIT — the CRT solve finds the true
 *       turret angle from the two encoders (no pointing needed once
 *       TurretCrt.CRT_ZERO_CALIBRATED = true).</li>
 *   <li>Still in init: hand-move the turret slightly — the boot check must
 *       flip to VERIFIED (proves the OctoQuad channel is actually counting).</li>
 *   <li>CALIBRATE once (with the turret at forward): read "CRT A raw revs" and
 *       "CRT B raw revs" below, enter them as TurretCrt.CRT_ZERO_OFFSET_A_REV /
 *       CRT_ZERO_OFFSET_B_REV in Panels (Panels -&gt; TurretCrt). These raw
 *       values are PRE-reversal (raw magnet frame), so flipping
 *       OctoQuadAbsoluteSource.ENCODER_REVERSED / TurretCrt.ENCODER_A_REVERSED
 *       later — e.g. because the dashboard rotated the wrong way — NEVER
 *       invalidates them; only the rotation direction changes.</li>
 *   <li>Press PLAY.</li>
 * </ol>
 *
 * <p>WRAP SEAM: encoder B repeats every 360/5.5 ≈ 65.5° of turret travel, so
 * overshoots past +/-180 cross the seam constantly — that's handled (wrapped
 * deltas, wrap-event counter, soft-limit guard). Watch "wrap/glitch events"
 * tick up once per limit excursion; anything frequent means a wiring/CRC
 * problem.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *   <li>Left stick X — jog the setpoint continuously (+ = CCW).</li>
 *   <li>D-pad left/right — nudge setpoint +15 (CCW) / -15 (CW).</li>
 *   <li>A — goto 0 (forward). Y — goto +90 (left). B — goto -90 (right).
 *       X — goto +180 via GotoSafeAngle (soft-limit aware).</li>
 *   <li>Right bumper — BRAKE (0 power; the 5203's BRAKE behavior holds).
 *       Left bumper — re-zero at forward (re-declares "this is 0 deg" — the
 *       recovery for a wrong boot).</li>
 *   <li>Back — clear latched faults. Right stick click — start/stop the
 *       hand-spin gearing verifier.</li>
 * </ul>
 *
 * <p>90-DEGREE ABS PWM BENCH TEST (dpad up + down): discriminates wiring from
 * calibration, on BOTH encoder channels:
 * <ol>
 *   <li>Hold the turret still, press dpad UP — records the raw µs on BOTH
 *       channels.</li>
 *   <li>Rotate the turret EXACTLY 90 degrees (any direction).</li>
 *   <li>Press dpad DOWN — prints the µs delta on both channels. A healthy
 *       path shows ~±473 µs on B and ~±452 µs on A for a 90° turret rotation
 *       (the pulse train is 1024 µs per ENCODER rev), and every reading
 *       inside 1-1024. A delta near 0, wildly different between channels, or
 *       out-of-window values means the channel is NOT receiving the ABS pulse
 *       train — check the cable (the yellow wire must be moved to the ABS
 *       pin-5 position on the 6-pin REV connector end, per the SDK sample)
 *       and the seating.</li>
 * </ol>
 */
@TeleOp(name = "Turret Debug", group = "Debug")
@Disabled
public class TurretPivotDebug extends OpMode {

    /** Setpoint jog speed for the left stick (deg/s at full stick). */
    public static double JOG_RATE_DEG_S = 90.0;
    /** D-pad nudge size (deg). */
    public static double NUDGE_DEG = 15.0;

    private TurretPivot turret;
    private Telemetry dash = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // hand-spin gearing verifier
    private boolean handSpinActive = false;
    private double handSpinRef = 0;

    // 90-degree ABS PWM bench test
    private boolean pwmTestArmed = false;
    private int pwmTestStartUs = Integer.MIN_VALUE;

    // previous-button snapshot for edge detection
    boolean lastA, lastB, lastX, lastY, lastDl, lastDr, lastDu, lastDd, lastRb, lastLb, lastBack, lastRsClick;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset(); // clear cross-opmode leftovers, like RobotMain
        turret = TurretPivot.tryCreate(this);

        telemetry.addData("Turret Debug", turret == null
                ? "MISSING — configure '" + IronConstants.TurretMotorName + "' (5203 motor) + '"
                + IronConstants.TurretOctoQuadName + "' (OctoQuad, I2C)"
                : "ready");
        telemetry.addData("BOOT", "CRT solves the angle at INIT; hand-move the turret to verify the encoder");
        telemetry.update();
        if (turret != null) {
            dash.addData("Turret Debug", "ready — see DS for boot procedure");
            dash.update();
        }
    }

    @Override
    public void init_loop() {
        if (turret != null) {
            turret.pollBootEncoderCheck();
            telemetry.addData("Turret boot encoder", turret.isBootEncoderVerified()
                    ? "VERIFIED (encoder counts)" : "not yet — hand-move the turret a little");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if (turret != null) {
            // Re-anchor first (the turret may have been spun after init), then
            // park — never auto-move on PLAY; the drivers command motion.
            turret.rezeroAtStart();
            turret.BRAKE();
        }
    }

    @Override
    public void loop() {
        if (turret == null) {
            telemetry.addData("Turret Debug", "no hardware configured");
            telemetry.update();
            return;
        }

        handleGamepad();

        CommandScheduler.getInstance().run(); // turret.periodic() -> encoder update + PID + telemetry
        updateTelemetry();
        telemetry.update();
        dash.update(); // single Panels flush at end of loop (team convention)
    }

    @Override
    public void stop() {
        if (turret != null) {
            turret.BRAKE();
        }
    }

    private void handleGamepad() {
        boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
        boolean dl = gamepad1.dpad_left, dr = gamepad1.dpad_right;
        boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;
        boolean back = gamepad1.back, rsClick = gamepad1.right_stick_button;
        boolean du = gamepad1.dpad_up, dd = gamepad1.dpad_down;

        // Continuous jog: move the SETPOINT (position control stays engaged —
        // this debugs the same PID the robot will actually use).
        double jog = -gamepad1.left_stick_x; // stick right = CW = negative angle
        if (Math.abs(jog) > 0.05) {
            turret.NudgeDeg(jog * JOG_RATE_DEG_S * 0.02);
        }

        if (dl && !lastDl) turret.NudgeDeg(NUDGE_DEG);      // + = CCW (left)
        if (dr && !lastDr) turret.NudgeDeg(-NUDGE_DEG);     // - = CW (right)
        if (a && !lastA) turret.GotoSafeAngle(0);
        if (y && !lastY) turret.GotoSafeAngle(90);
        if (b && !lastB) turret.GotoSafeAngle(-90);
        if (x && !lastX) turret.GotoSafeAngle(180);
        if (rb && !lastRb) turret.BRAKE();
        if (lb && !lastLb) turret.reZeroAtForward();
        if (back && !lastBack) turret.clearFaults();

        // Hand-spin gearing verifier
        if (rsClick && !lastRsClick) {
            handSpinActive = !handSpinActive;
            handSpinRef = turret.getUnwrappedDeg();
        }

        // 90-degree ABS PWM bench test (dpad up = mark, down = finish)
        if (du && !lastDu) {
            pwmTestArmed = true;
            pwmTestStartUs = turret.getAbsoluteRawUs();
        }
        if (dd && !lastDd && pwmTestArmed) {
            pwmTestArmed = false;
            int nowUs = turret.getAbsoluteRawUs();
            if (pwmTestStartUs == Integer.MIN_VALUE || nowUs == Integer.MIN_VALUE) {
                telemetry.addData("PWM test", "no raw us available");
            } else {
                int delta = nowUs - pwmTestStartUs;
                // wrap the delta to the pulse window (a seam crossing shows as ~±1024)
                int wrapped = delta;
                if (wrapped > 512) wrapped -= 1024;
                if (wrapped < -512) wrapped += 1024;
                telemetry.addData("PWM test", "delta %d us for your rotation (healthy 90 deg ~ %d us, 1 us ~ 0.35 deg)",
                        wrapped, Math.round(1024.0 / 4.0));
            }
        }

        lastA = a; lastB = b; lastX = x; lastY = y;
        lastDl = dl; lastDr = dr; lastDu = du; lastDd = dd;
        lastRb = rb; lastLb = lb; lastBack = back; lastRsClick = rsClick;
    }

    private void updateTelemetry() {
        telemetry.addData("angle", String.format(Locale.US, "%+.2f deg", turret.getPosition()));
        telemetry.addData("setpoint", String.format(Locale.US, "%+.2f deg (req %+.2f)",
                turret.getSetpoint(), turret.getRequestedPosition()));
        telemetry.addData("state", turret.State);
        telemetry.addData("velocity", String.format(Locale.US, "%+.1f deg/s", turret.getVelocityDegPerSec()));
        telemetry.addData("at setpoint", turret.atSetpoint());
        telemetry.addData("feedback", turret.getFeedbackSourceName());
        telemetry.addData("drive", turret.getDriveName());
        String echo = turret.getAbsoluteConfigEcho();
        if (echo != null) {
            telemetry.addData("OQ config", echo);
        }
        int[] diag = turret.getAbsoluteDiagCounters();
        telemetry.addData("OQ diag OOR/CRC/glitch", diag[0] + " / " + diag[1] + " / " + diag[2]);
        int rawUs = turret.getAbsoluteRawUs();
        telemetry.addData("OQ raw us", rawUs == Integer.MIN_VALUE ? "n/a" : String.valueOf(rawUs));
        double amps = turret.getMotorAmps();
        if (!Double.isNaN(amps)) {
            telemetry.addData("motor current", String.format(Locale.US, "%.2f A", amps));
        }
        double absDeg = turret.getAbsoluteRawDeg();
        telemetry.addData("abs raw deg", Double.isNaN(absDeg) ? "n/a" : String.format(Locale.US, "%.1f", absDeg));

        // ---- CRT dual-encoder bring-up telemetry ----
        // "CRT A raw revs" / "CRT B raw revs" are the calibration values: park
        // the turret at forward, read both, enter them as TurretCrt's
        // CRT_ZERO_OFFSET_A_REV / CRT_ZERO_OFFSET_B_REV in Panels. These are
        // RAW (pre-reversal) values — flipping the direction flags later never
        // invalidates the calibration; it only mirrors the shown direction.
        int crossUs = turret.getCrossRawUs();
        telemetry.addData("CRT A raw us", crossUs == Integer.MIN_VALUE ? "n/a" : String.valueOf(crossUs));
        telemetry.addData("CRT A raw revs", crossUs == Integer.MIN_VALUE ? "n/a"
                : String.format(Locale.US, "%.4f", crossUs / 1024.0));
        telemetry.addData("CRT B raw revs", rawUs == Integer.MIN_VALUE ? "n/a"
                : String.format(Locale.US, "%.4f", rawUs / 1024.0));
        telemetry.addData("CRT boot", turret.isCrtBootUsed()
                ? String.format(Locale.US, "SOLVED (diff %.4f enc-revs, %d candidates)",
                        turret.getCrtSolveDiff(), turret.getCrtSolveCandidates())
                : "not used (assumed forward — check the CRT calibration)");
        double ccErr = turret.getCrtCrosscheckErrDeg();
        telemetry.addData("CRT crosscheck", Double.isNaN(ccErr) ? "n/a"
                : String.format(Locale.US, "%+.2f deg%s", ccErr,
                        turret.isCrtCrosscheckFaulted() ? " FAULTED" : ""));

        telemetry.addData("boot", turret.isBootEncoderVerified() ? "encoder VERIFIED" : "encoder not verified");
        telemetry.addData("faults", turret.getFaults());

        if (handSpinActive) {
            double deltaDeg = turret.getUnwrappedDeg() - handSpinRef;
            telemetry.addData("HAND-SPIN verifier", String.format(
                    Locale.US, "delta %+.2f turret deg (one turret rev = 360 deg)", deltaDeg));
        }
    }
}
