package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Loads a Pedro Pathing Visualizer {@code .pp} file (JSON) and exposes its points and paths.
 *
 * <p>Indexing convention (as requested):
 * <ul>
 *   <li>{@code getPoint(0)} returns the file's {@code startPoint}.</li>
 *   <li>{@code getPoint(i)} for {@code i > 0} returns the end point of the
 *       {@code i}-th path in execution order ({@code 1}-based into {@code lines}).</li>
 *   <li>{@code getPath(i)} returns the path from {@code getPoint(i)} to
 *       {@code getPoint(i + 1)} with the heading behavior stored in the file
 *       (linear / constant / tangential / facingPoint), matching the
 *       {@code Paths.line(...).linear(...)}/{@code .tangent()} style used in
 *       {@code Red_ExampleAuto}.</li>
 * </ul>
 *
 * <p><b>Naming model — points vs. lines.</b> Only <em>lines</em> are named in the
 * visualizer; <em>points</em> borrow the name of the line that arrives at them:
 * <ul>
 *   <li>point 0 = the {@code startPoint} (its own {@code name}, plus the aliases
 *       {@code "start"}, {@code "startpoint"}, {@code "point0"}/{@code "p0"}).</li>
 *   <li>point {@code i > 0} = the end point of the {@code i}-th line in execution
 *       order, so it inherits that line's {@code name}.</li>
 * </ul>
 * Example ({@code exampleAuto1.pp}: start {@code "StartPoint"}, lines
 * {@code "StartToOffset"}, {@code "OffsetToPark"}):
 * <ul>
 *   <li>points: 0={@code "StartPoint"}, 1={@code "StartToOffset"}, 2={@code "OffsetToPark"}.</li>
 *   <li>paths: 0 is line {@code "StartToOffset"} (point 0 -&gt; 1), 1 is line
 *       {@code "OffsetToPark"} (point 1 -&gt; 2).</li>
 *   <li>{@code getPath("StartToOffset")} = the path <em>starting at</em> point
 *       {@code "StartToOffset"} (path 1 -&gt; 2).</li>
 *   <li>{@code getPathByLineName("StartToOffset")} = the <em>line</em> called
 *       {@code "StartToOffset"} (path 0 -&gt; 1), i.e. the path that
 *       <em>arrives at</em> the point sharing its name.</li>
 * </ul>
 *
 * <p>All {@link Pose}s are created through the supplied {@link PoseFactory} exactly once,
 * so a mirrored factory (Blue alliance) automatically mirrors both the poses <em>and</em>
 * the Bezier control points. Pass the same {@code poseFactory} you already use in the auto
 * (e.g. {@code PoseFactory.degrees().mirrorY(...).mirrorX(...)} for Blue).
 *
 * <p>Note: the {@code .pp} files currently live next to the autos in
 * {@code Autos/PathFiles/}, which is <b>not</b> packaged onto the robot. Copy the file you
 * need into {@code TeamCode/src/main/assets/pathfiles/} and load it with
 * {@link #fromAsset(HardwareMap, String, PoseFactory)}, e.g. asset path
 * {@code "pathfiles/exampleAuto1.pp"}.
 */
public class PPFile {

    /** One path segment in execution order, stored in mirrored (field-ready) space. */
    private static class Segment {
        final Vector2D startPos;
        final Vector2D endPos;
        final double startHeadingRad;
        final double endHeadingRad;
        final String headingType; // "linear", "constant", "tangential", "facingpoint"
        final boolean reverse;
        final double endT; // NaN when absent; only used by linear
        final Vector2D facingTarget; // nullable
        final List<Vector2D> controls;
        final String name;

        Segment(Vector2D startPos, Vector2D endPos,
                double startHeadingRad, double endHeadingRad,
                String headingType, boolean reverse, double endT,
                Vector2D facingTarget, List<Vector2D> controls, String name) {
            this.startPos = startPos;
            this.endPos = endPos;
            this.startHeadingRad = startHeadingRad;
            this.endHeadingRad = endHeadingRad;
            this.headingType = headingType;
            this.reverse = reverse;
            this.endT = endT;
            this.facingTarget = facingTarget;
            this.controls = controls;
            this.name = name;
        }
    }

    private final List<Pose> points = new ArrayList<>();
    private final List<Segment> segments = new ArrayList<>();
    /** Parallel to {@link #points}: addressable name of each point. */
    private final List<String> pointNames = new ArrayList<>();
    /** Lower-cased point name -> point index (first wins on duplicates). */
    private final Map<String, Integer> pointIndexByName = new HashMap<>();
    /** Lower-cased path/line name -> path index (first wins on duplicates). */
    private final Map<String, Integer> pathIndexByName = new HashMap<>();

    private PPFile() {
    }

    /**
     * Builds the Blue-alliance factory for a 180-degree rotationally symmetric
     * field (Red design rotated half a turn around {@code (centerX, centerY)}).
     *
     * <p>Why this exists instead of {@code mirrorY().mirrorX()}: Pedro's
     * {@code mirrorX} negates the heading ({@code h -> -h}) and {@code mirrorY}
     * leaves it unchanged, so the chained factory maps heading {@code h -> -h}.
     * That matches a 180-degree rotation <em>only</em> for {@code +/-90} deg
     * (e.g. Red 90 -> Blue 270, correct). For anything else it is wrong: Red 60
     * -> Blue 300 (should be 240), Red 0 -> Blue 0 (should be 180). Positions
     * ({@code x -> 2*cx - x}, {@code y -> 2*cy - y}) are a true point reflection
     * either way, so XY looks right on Blue while headings do not.
     *
     * <p>Use this in the auto instead, e.g.
     * {@code PPFile.blueRotationFactory(GameConst.FieldCenter.x(),
     * GameConst.FieldCenter.y())}. The {@code mapHeading} lambda operates in
     * radians (the factory stores radians internally; {@code of()} converts the
     * degrees you pass it before the ops run).
     */
    public static PoseFactory blueRotationFactory(double centerX, double centerY) {
        return PoseFactory.degrees()
                .mapX(x -> 2 * centerX - x)
                .mapY(y -> 2 * centerY - y)
                .mapHeading(h -> h + Math.PI);
    }

    /**
     * Straight segments are built as 3-point quadratic Beziers
     * ({@code start, midpoint, end}) instead of {@code Line}s. The midpoint
     * Bezier is geometrically <em>identical</em> to the line
     * ({@code B(t) = P0*(1-t) + P2*t}), so XY tracking is unchanged — but it
     * keeps us on Pedro's native heading interpolators for everything.
     *
     * <p>Why: Pedro 3.0.0's {@code Interpolator.linear} scales by
     * {@code Curve.pathCompletion(t)}, and {@code Line} inherits the default
     * {@code pathCompletion(t) == 1 - t} (remaining-distance based), so native
     * {@code .linear(s, e)} runs <em>backwards</em> on straight segments
     * ({@code heading(0) == e}, verified empirically against core-3.0.0) while
     * {@code BezierCurve} overrides it with a true arc-length fraction and runs
     * correctly. {@code PiecewiseInterpolator} keys off the same completion, so
     * the {@code endT} variant is broken for lines too. Building straights as
     * Beziers makes native {@code .linear(s, e)} / {@code .linear(s, e, endT)}
     * correct for every segment type with no custom interpolator and no
     * arg-swapping. If Pedro fixes {@code Line.pathCompletion}, this stays
     * correct as-is.
     *
     * <p>NOTE on the visualizer {@code reverse} checkbox: it means "drive this
     * segment backwards", it does <em>not</em> add 180 deg to a linear heading
     * (exported code keeps {@code setLinearHeadingInterpolation(start, end)}
     * and adds a separate drive-direction flag). Core 3.0.0 has no
     * drive-direction flag on {@link Path} — {@code reverse} only exists as
     * {@code reverseTangent()} — so {@code reverse} is honored for tangential
     * headings and deliberately <em>ignored</em> for linear/constant/facingPoint
     * (adding PI facest the robot backwards and causes the 180 deg pre-spin).
     */

    // ------------------------------------------------------------------ loading

    /** Parse from a JSON string. The factory must be degrees-based (as in the autos). */
    public static PPFile fromJsonString(String json, PoseFactory poseFactory) throws Exception {
        PPFile file = new PPFile();
        file.parse(new JSONObject(json), poseFactory);
        return file;
    }

    /** Load from {@code TeamCode/src/main/assets/...} via the RC hardware map. */
    public static PPFile fromAsset(HardwareMap hardwareMap, String assetPath,
                                   PoseFactory poseFactory) throws Exception {
        try (InputStream in = hardwareMap.appContext.getAssets().open(assetPath)) {
            return fromJsonString(readFully(in), poseFactory);
        }
    }

    private static String readFully(InputStream in) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buf = new byte[4096];
        int n;
        while ((n = in.read(buf)) != -1) {
            out.write(buf, 0, n);
        }
        return out.toString("UTF-8");
    }

    /**
     * Registers the addressable name for point {@code index}.
     *
     * <p>Empty/blank names fall back to {@code fallback}. Lookup is
     * case-insensitive with first-wins on duplicates. Point 0 also answers to
     * {@code "start"}, {@code "startpoint"} and {@code "point0"}; every point
     * also answers to {@code "point<i>"} and {@code "p<i>"}.
     */
    private void registerPointName(String raw, int index, String fallback) {
        String primary = raw == null ? "" : raw.trim();
        if (primary.isEmpty()) {
            primary = fallback;
        }
        pointNames.add(primary);
        pointIndexByName.putIfAbsent(primary.toLowerCase(Locale.US), index);
        pointIndexByName.putIfAbsent(("point" + index).toLowerCase(Locale.US), index);
        pointIndexByName.putIfAbsent(("p" + index).toLowerCase(Locale.US), index);
        if (index == 0) {
            pointIndexByName.putIfAbsent("start", 0);
            pointIndexByName.putIfAbsent("startpoint", 0);
            pointIndexByName.putIfAbsent("start point", 0);
        }
    }

    private void registerPathName(String raw, int pathIndex) {
        String clean = raw == null ? "" : raw.trim();
        if (clean.isEmpty()) {
            return;
        }
        pathIndexByName.putIfAbsent(clean.toLowerCase(Locale.US), pathIndex);
    }

    private static String normalizeKey(String name) {
        return name == null ? "" : name.trim().toLowerCase(Locale.US);
    }

    // ------------------------------------------------------------------ parsing

    private void parse(JSONObject root, PoseFactory poseFactory) throws Exception {
        JSONObject startObj = root.getJSONObject("startPoint");
        double startX = startObj.getDouble("x");
        double startY = startObj.getDouble("y");
        double startDeg = readDeg(startObj, "headingDeg", "heading", 0.0);

        Pose startPose = poseFactory.of(startX, startY, startDeg);

        JSONArray lines = root.optJSONArray("lines");
        if (lines == null) {
            lines = new JSONArray();
        }

        // Map line id -> index so we can honor the "sequence" execution order.
        Map<String, Integer> idToIndex = new HashMap<>();
        for (int i = 0; i < lines.length(); i++) {
            String id = lines.optJSONObject(i).optString("id", "");
            if (!id.isEmpty()) {
                idToIndex.put(id, i);
            }
        }
        List<Integer> order = new ArrayList<>();
        JSONArray sequence = root.optJSONArray("sequence");
        if (sequence != null) {
            for (int i = 0; i < sequence.length(); i++) {
                JSONObject entry = sequence.optJSONObject(i);
                if (entry == null || !"path".equals(entry.optString("kind", "path"))) {
                    continue;
                }
                Integer idx = idToIndex.get(entry.optString("lineId", ""));
                if (idx != null) {
                    order.add(idx);
                }
            }
        }
        if (order.isEmpty()) {
            for (int i = 0; i < lines.length(); i++) {
                order.add(i);
            }
        }

        points.add(startPose);
        registerPointName(startObj.optString("name", ""), 0, "start");
        Pose cursor = startPose; // mirrored pose of the previous point
        // Raw (file-space) cursor so fallback headings stay in file space and the
        // factory is applied exactly once per raw (x, y, deg) triple.
        double rawX = startX;
        double rawY = startY;
        double rawDeg = startDeg;

        for (int lineIndex : order) {
            JSONObject line = lines.getJSONObject(lineIndex);

            JSONObject endObj = line.getJSONObject("endPoint");
            double endX = endObj.getDouble("x");
            double endY = endObj.getDouble("y");

            // Mirror the raw end position through the factory exactly once. Using
            // factory.of(x, y, 0) reuses whatever mirror/map ops the caller configured
            // (heading 0 is unaffected by the mirror negations).
            Vector2D endPos = poseFactory.of(endX, endY, 0).toVector2D();
            Vector2D startPos = cursor.toVector2D();

            // ---- control points: new "controlPoints" array + legacy one/two keys.
            List<Vector2D> controls = new ArrayList<>();
            JSONArray cpArray = line.optJSONArray("controlPoints");
            if (cpArray != null) {
                for (int i = 0; i < cpArray.length(); i++) {
                    JSONObject cp = cpArray.optJSONObject(i);
                    if (cp != null && cp.has("x") && cp.has("y")) {
                        controls.add(poseFactory.of(cp.getDouble("x"), cp.getDouble("y"), 0).toVector2D());
                    }
                }
            }
            JSONObject cpOne = line.optJSONObject("controlPointOne");
            if (cpOne != null && cpOne.has("x")) {
                controls.add(poseFactory.of(cpOne.getDouble("x"), cpOne.getDouble("y"), 0).toVector2D());
            }
            JSONObject cpTwo = line.optJSONObject("controlPointTwo");
            if (cpTwo != null && cpTwo.has("x")) {
                controls.add(poseFactory.of(cpTwo.getDouble("x"), cpTwo.getDouble("y"), 0).toVector2D());
            }

            // ---- heading spec (all degrees here are in file/Red space; the factory
            // mirrors them exactly once below). Fallback is continuity in file space.
            JSONObject heading = line.optJSONObject("heading");
            String type = heading != null ? heading.optString("type", "linear") : "linear";
            boolean reverse = heading != null && heading.optBoolean("reverse", false);
            double segStartDeg = rawDeg;
            double segEndDeg = rawDeg;
            double endT = Double.NaN;
            Vector2D facingTarget = null;

            if (heading != null) {
                // Linear / constant degree keys, with radian fallbacks for older files.
                if (heading.has("startDeg") || heading.has("start")) {
                    segStartDeg = readDeg(heading, "startDeg", "start", segStartDeg);
                }
                // else: no per-segment start -> keep file-space continuity (rawDeg).
                if (heading.has("endDeg") || heading.has("end") || heading.has("headingDeg")
                        || heading.has("heading")) {
                    segEndDeg = readDeg(heading, "endDeg",
                            heading.has("end") ? "end" : (heading.has("headingDeg") ? "headingDeg" : "heading"),
                            segStartDeg);
                } else if (endObj.has("headingDeg") || endObj.has("heading")) {
                    segEndDeg = readDeg(endObj, "headingDeg", "heading", segStartDeg);
                } else if ("constant".equalsIgnoreCase(type)) {
                    segEndDeg = segStartDeg;
                }
                if (heading.has("endT")) {
                    endT = heading.optDouble("endT", Double.NaN);
                }
                if (heading.has("targetX") && heading.has("targetY")) {
                    facingTarget = poseFactory.of(
                            heading.getDouble("targetX"), heading.getDouble("targetY"), 0).toVector2D();
                } else if (heading.has("x") && heading.has("y") && "facingpoint".equalsIgnoreCase(type)) {
                    facingTarget = poseFactory.of(
                            heading.getDouble("x"), heading.getDouble("y"), 0).toVector2D();
                }
            } else if (endObj.has("headingDeg") || endObj.has("heading")) {
                segEndDeg = readDeg(endObj, "headingDeg", "heading", segStartDeg);
            }

            String normType = type.toLowerCase();
            String name = line.optString("name", "");

            double segStartRad;
            double segEndRad;
            Pose pointEnd;
            if (normType.startsWith("tangent") || normType.startsWith("facing")) {
                // Tangential / facing-point headings come from the path tangent at
                // runtime; the stored point carries the incoming heading so
                // getPoint() stays sensible.
                segStartRad = cursor.heading();
                segEndRad = cursor.heading();
                pointEnd = new Pose(endPos.x(), endPos.y(), cursor.heading());
            } else {
                // Linear / constant: raw file-space degrees through the factory
                // exactly once (handles Blue mirroring of both position and heading).
                segStartRad = poseFactory.of(rawX, rawY, segStartDeg).heading();
                Pose endProbe = poseFactory.of(endX, endY, segEndDeg);
                segEndRad = endProbe.heading();
                pointEnd = new Pose(endPos.x(), endPos.y(), segEndRad);
            }

            // Interpolator start pose uses the segment's own start heading (file authority),
            // which may differ from the cursor heading at a heading discontinuity.
            Pose segStartPose = new Pose(startPos.x(), startPos.y(), segStartRad);
            Pose segEndPose = new Pose(endPos.x(), endPos.y(), segEndRad);

            segments.add(new Segment(segStartPose.toVector2D(), segEndPose.toVector2D(),
                    segStartRad, segEndRad, normType, reverse, endT,
                    facingTarget, controls, name));
            registerPathName(name, segments.size() - 1);

            points.add(pointEnd);
            registerPointName(line.optString("name", ""), points.size() - 1, "point" + (points.size() - 1));
            cursor = pointEnd;
            rawX = endX;
            rawY = endY;
            rawDeg = segEndDeg;
        }
    }

    /** Read a heading in degrees, accepting either a degree key or a radian key. */
    private static double readDeg(JSONObject obj, String degKey, String radKey, double fallbackDeg) {
        try {
            if (obj.has(degKey)) {
                return obj.getDouble(degKey);
            }
            if (radKey != null && obj.has(radKey)) {
                return Math.toDegrees(obj.getDouble(radKey));
            }
        } catch (Exception ignored) {
        }
        return fallbackDeg;
    }

    // ------------------------------------------------------------------ API

    /** Number of points: 1 (start) + one per path. */
    public int getPointCount() {
        return points.size();
    }

    /**
     * Returns point {@code i}: {@code 0} is the file's {@code startPoint}, higher indices
     * are the successive {@code lines} end points in {@code sequence} order.
     */
    public Pose getPoint(int i) {
        return points.get(i);
    }

    /** Index of the point called {@code name} (case-insensitive, trimmed). */
    public int getPointIndex(String name) {
        Integer idx = pointIndexByName.get(normalizeKey(name));
        if (idx == null) {
            throw new IllegalArgumentException(
                    "No point named '" + name + "'. Known points: " + pointNames);
        }
        return idx;
    }

    /** Returns the point called {@code name} (case-insensitive, trimmed). */
    public Pose getPoint(String name) {
        return getPoint(getPointIndex(name));
    }

    /** Addressable name of point {@code i} (line name, or {@code "point<i>"} fallback). */
    public String getPointName(int i) {
        return pointNames.get(i);
    }

    /** True if a point called {@code name} exists (case-insensitive, trimmed). */
    public boolean hasPoint(String name) {
        return pointIndexByName.containsKey(normalizeKey(name));
    }

    /** Sugar for {@code getPoint(0)} — always the file's {@code startPoint}. */
    public Pose getStartPoint() {
        return getPoint(0);
    }

    /** Sugar for {@code getPoint(getPointCount() - 1)} — the final end point. */
    public Pose getEndPoint() {
        return getPoint(getPointCount() - 1);
    }

    /** Same as {@code getPoint(0)}. */
    public Pose getStartPose() {
        return getStartPoint();
    }

    /** Same as {@code getPoint(getPointCount() - 1)}. */
    public Pose getEndPose() {
        return getEndPoint();
    }

    /** Number of paths: always {@code getPointCount() - 1}. */
    public int getPathCount() {
        return segments.size();
    }

    /**
     * Returns the path from {@code getPoint(i)} to {@code getPoint(i + 1)} using the
     * heading mode stored in the file. Everything here uses Pedro's native
     * heading interpolators ({@code .linear(s, e)}, {@code .constant(e)},
     * {@code .tangent()}); straight segments are built as midpoint Beziers so
     * native {@code .linear(s, e)} runs the right direction (see the note above
     * on why {@code Line} can't be used with it in 3.0.0).
     */
    public Path getPath(int i) {
        Segment seg = segments.get(i);
        Pose s = new Pose(seg.startPos.x(), seg.startPos.y(), seg.startHeadingRad);
        Pose e = new Pose(seg.endPos.x(), seg.endPos.y(), seg.endHeadingRad);

        Path base;
        if (seg.controls.isEmpty()) {
            // Straight: midpoint quadratic Bezier, geometrically identical to the
            // line (B(t) = P0*(1-t) + P2*t), so native .linear runs correctly.
            Vector2D mid = Vector2D.cartesian(
                    (seg.startPos.x() + seg.endPos.x()) / 2.0,
                    (seg.startPos.y() + seg.endPos.y()) / 2.0);
            base = Paths.curve(seg.startPos, mid, seg.endPos);
        } else {
            Vector2D[] pts = new Vector2D[seg.controls.size() + 2];
            pts[0] = seg.startPos;
            for (int k = 0; k < seg.controls.size(); k++) {
                pts[k + 1] = seg.controls.get(k);
            }
            pts[pts.length - 1] = seg.endPos;
            base = Paths.curve(pts);
        }

        switch (seg.headingType) {
            case "constant":
                return base.constant(e);
            case "tangential":
                return seg.reverse ? base.reverseTangent() : base.tangent();
            case "facingpoint":
            case "facing_point":
            case "facing":
                if (seg.facingTarget != null) {
                    return base.facingPoint(seg.facingTarget);
                }
                return seg.reverse ? base.reverseTangent() : base.tangent();
            case "linear":
            default:
                if (!Double.isNaN(seg.endT) && seg.endT > 0 && seg.endT < 1) {
                    return base.linear(s, e, seg.endT);
                }
                return base.linear(s, e);
        }
    }

    /** Display name of the {@code i}-th path (the visualizer's line name, may be empty). */
    public String getPathName(int i) {
        return segments.get(i).name;
    }

    /** Heading mode of the {@code i}-th path as stored in the file. */
    public String getHeadingType(int i) {
        return segments.get(i).headingType;
    }

    /**
     * Index of the path that <em>starts</em> at the point called {@code startPointName}
     * (case-insensitive, trimmed). For example, if {@code getPoint(2)} is named
     * {@code "Score"}, then {@code getPathIndex("Score") == 2} and
     * {@code getPath("Score")} is the path from {@code getPoint(2)} to
     * {@code getPoint(3)}.
     *
     * @throws IllegalArgumentException if no such point exists, or the named point
     *         is the final point (it has no outgoing path).
     */
    public int getPathIndex(String startPointName) {
        int pointIndex = getPointIndex(startPointName);
        if (pointIndex >= getPathCount()) {
            throw new IllegalArgumentException(
                    "Point '" + startPointName + "' is the final point; it has no outgoing path.");
        }
        return pointIndex;
    }

    /**
     * Same as {@code getPath(int)}, but the path is selected by the name of its
     * <em>starting</em> point ("by start point"): {@code getPath(name)} is
     * {@code getPath(getPointIndex(name))}, i.e. the path that <em>leaves</em> the
     * named point for the next point. Lookup is case-insensitive and trimmed.
     *
     * <p>Example ({@code exampleAuto1.pp}): {@code getPath("StartToOffset")} is
     * the path <em>starting at</em> point {@code "StartToOffset"} (point 1 -&gt;
     * 2) — <b>not</b> the line called {@code "StartToOffset"} (path 0 -&gt; 1).
     * Points borrow the name of the line that arrives at them, so the
     * "same-named" line is the <em>inbound</em> one; use
     * {@link #getPathByLineName(String)} for the line lookup ("by line name" =
     * the path that <em>arrives at</em> the point sharing its name).
     */
    public Path getPath(String startPointName) {
        return getPath(getPathIndex(startPointName));
    }

    /**
     * Index of the path whose visualizer line is called {@code lineName} ("by
     * line name", case-insensitive, trimmed): the path that <em>arrives at</em>
     * the point sharing the line's name.
     *
     * <p>Example: {@code getPathIndexByLineName("StartToOffset") == 0} — line
     * {@code "StartToOffset"} runs point 0 -&gt; 1, ending at point
     * {@code "StartToOffset"}.
     */
    public int getPathIndexByLineName(String lineName) {
        Integer idx = pathIndexByName.get(normalizeKey(lineName));
        if (idx == null) {
            throw new IllegalArgumentException(
                    "No path (line) named '" + lineName + "'.");
        }
        return idx;
    }

    /**
     * The path whose visualizer line is called {@code lineName} ("by line
     * name"): the path that <em>arrives at</em> the point sharing the line's
     * name. Contrast with {@link #getPath(String)}, which selects by the
     * <em>starting</em> point's name. Lookup is case-insensitive and trimmed.
     *
     * <p>Example: {@code getPathByLineName("StartToOffset")} is path 0 -&gt; 1,
     * while {@code getPath("StartToOffset")} is path 1 -&gt; 2.
     */
    public Path getPathByLineName(String lineName) {
        return getPath(getPathIndexByLineName(lineName));
    }

    /**
     * True if a path <em>starts at</em> the point called {@code startPointName}
     * ("by start point"; false for the final point, which has no outgoing path).
     */
    public boolean hasPath(String startPointName) {
        Integer idx = pointIndexByName.get(normalizeKey(startPointName));
        return idx != null && idx < getPathCount();
    }

    /**
     * True if a visualizer line called {@code lineName} exists ("by line
     * name" — the path that <em>arrives at</em> the point sharing its name).
     */
    public boolean hasPathLine(String lineName) {
        return pathIndexByName.containsKey(normalizeKey(lineName));
    }
}
