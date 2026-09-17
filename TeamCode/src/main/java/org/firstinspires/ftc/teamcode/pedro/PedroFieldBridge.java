package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Coordinate bridge between your {@code FieldTracker} frame and Pedro's
 * {@link Pose} frame. Both are inches + radians CCW+, and both face +Y at
 * heading 0 — they differ ONLY by origin:
 *
 * <ul>
 *   <li>FieldTracker / {@link Pose2D}: origin at FIELD CENTER, -72..+72.</li>
 *   <li>Pedro {@link Pose}: origin at the FIELD CORNER, 0..144.</li>
 * </ul>
 *
 * So the conversion is a pure +72in shift on x and y. Heading passes through
 * untouched. Verify on the field: robot at field center facing up-field
 * (away from red wall) must read (0,0,0deg) on FieldTracker and (72,72,90deg)
 * on Pedro. If the Pedro heading is off by 90deg, Pedro's 0=+X convention is
 * biting you — use {@link #fieldHeadingToPedro} / {@link #pedroHeadingToField},
 * not a raw copy.
 *
 * <p>Which frame faces which way (both viewed from the red wall):
 * <pre>
 *   FieldTracker: +X east (right), +Y north (up-field), H=0 faces +Y, CCW+
 *   Pedro:        +X east (right), +Y north (up-field), H=0 faces +X, CCW+
 * </pre>
 */
public final class PedroFieldBridge {

    /** Half the 144in FTC field. */
    public static final double HALF_FIELD_IN = 72.0;

    private PedroFieldBridge() {}

    // ================= Position =================

    /** FieldTracker center-origin inches -> Pedro corner-origin inches. */
    public static Pose fieldToPedro(double xFieldIn, double yFieldIn, double headingFieldRad) {
        return new Pose(xFieldIn + HALF_FIELD_IN, yFieldIn + HALF_FIELD_IN,
                fieldHeadingToPedro(headingFieldRad));
    }

    /** FieldTracker Pose2D (inches) -> Pedro Pose. */
    public static Pose fieldToPedro(Pose2D fieldPoseInches) {
        return fieldToPedro(
                fieldPoseInches.getX(DistanceUnit.INCH),
                fieldPoseInches.getY(DistanceUnit.INCH),
                fieldPoseInches.getHeading(AngleUnit.RADIANS));
    }

    /** Pedro corner-origin inches -> FieldTracker center-origin inches {x, y}. Heading via {@link #pedroHeadingToField}. */
    public static double[] pedroToFieldXY(Pose pedro) {
        return new double[]{pedro.x() - HALF_FIELD_IN, pedro.y() - HALF_FIELD_IN};
    }

    /** Pedro Pose -> FieldTracker Pose2D in inches. */
    public static Pose2D pedroToField(Pose pedro) {
        return new Pose2D(DistanceUnit.INCH,
                pedro.x() - HALF_FIELD_IN,
                pedro.y() - HALF_FIELD_IN,
                AngleUnit.RADIANS, pedroHeadingToField(pedro.heading()));
    }

    // ================= Heading =================
    // Field H=0 faces +Y; Pedro H=0 faces +X. Field H = Pedro H - 90deg.

    /** Field heading (0 = +Y) -> Pedro heading (0 = +X). */
    public static double fieldHeadingToPedro(double fieldRad) {
        return wrap2Pi(fieldRad + Math.PI / 2.0);
    }

    /** Pedro heading (0 = +X) -> Field heading (0 = +Y). */
    public static double pedroHeadingToField(double pedroRad) {
        return wrapPi(pedroRad - Math.PI / 2.0);
    }

    /** Degrees convenience: field deg (0 = up-field) -> Pedro radians. */
    public static double fieldDegToPedroRad(double fieldDeg) {
        return fieldHeadingToPedro(Math.toRadians(fieldDeg));
    }

    /** Start-pose helper: field inches + field degrees -> Pedro Pose. */
    public static Pose startPose(double xFieldIn, double yFieldIn, double headingFieldDeg) {
        return fieldToPedro(xFieldIn, yFieldIn, Math.toRadians(headingFieldDeg));
    }

    private static double wrapPi(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double wrap2Pi(double a) {
        a %= 2.0 * Math.PI;
        if (a < 0) a += 2.0 * Math.PI;
        return a;
    }
}
