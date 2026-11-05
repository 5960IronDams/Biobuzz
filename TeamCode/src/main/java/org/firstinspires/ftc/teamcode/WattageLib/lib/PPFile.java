package org.firstinspires.ftc.teamcode.WattageLib.lib;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.interpolator.PiecewiseInterpolator;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Parses a Pedro Pathing Visualizer project file into {@link Pose} points and {@link Path} paths.
 *
 * <p>The visualizer saves JSON under a {@code .pp.json} name; older builds used {@code .pp} and
 * both still load. The file's {@code version} is not interpreted: the structure itself is read,
 * so 1.5 and 2.0 documents are handled the same way.
 *
 * <p>Point 0 is the file's start point and point {@code i > 0} is where the {@code i}-th path ends,
 * so a point borrows the name of the line that arrives at it (unnamed points fall back to
 * {@code point<i>}). {@link #path(String)} fetches a drawn line by name, while
 * {@link #pathAfter(String)} fetches the path leaving a named point. {@link #steps()} walks the
 * file's run plan — its paths with the waits between them — and {@link #waitBeforeMs(int)} and
 * {@link #waitAfterMs(int)} expose a line's own pauses.
 *
 * <pre>{@code
 * PPFile pp = PPFile.parse(json, PoseFactory.degrees().mirrorAroundPoint(72, 72)); // blue side of a 180° field
 * follower.setPose(pp.startPoint());
 * for (Step step : pp.steps()) {
 *     if (step.isWait()) sleep(step.waitMs());
 *     else follower.follow(pp.path(step.pathIndex()));
 * }
 * }</pre>
 *
 * <p>Every position and heading goes through the {@link PoseFactory} exactly once, so alliance
 * mirroring and any other transform are the factory's job. The visualizer's per-line
 * {@code reverse} checkbox means "drive this segment backwards": it is honored for tangential
 * headings and piecewise nodes, and ignored for plain linear and constant headings, which store
 * explicit angles a backwards-driven robot would face away from.
 */
public final class PPFile {

    private final List<Pose> points = new ArrayList<>();
    private final List<String> pointNames = new ArrayList<>();
    private final List<Path> paths = new ArrayList<>();
    private final List<Long> waitBeforeMs = new ArrayList<>();
    private final List<Long> waitAfterMs = new ArrayList<>();
    private final List<Step> steps = new ArrayList<>();
    private final Map<String, Integer> pointsByName = new HashMap<>();
    private final Map<String, Integer> linesByName = new HashMap<>();

    /** Parses a {@code .pp} file's JSON with a degrees-based factory, as the visualizer writes it. */
    public static PPFile parse(String json) {
        return parse(json, PoseFactory.degrees());
    }

    /** Parses a {@code .pp} file's JSON, applying {@code poseFactory} to every position and heading. */
    public static PPFile parse(String json, PoseFactory poseFactory) {
        return new PPFile(Json.parse(json), poseFactory);
    }

    private PPFile(Object json, PoseFactory poseFactory) {
        Map<String, Object> root = requiredObject(json, "a JSON object");
        Map<String, Object> start = requiredObject(root.get("startPoint"), "a startPoint");
        addPoint(startPointName(start),
                poseFactory.of(number(start, "x"), number(start, "y"), startHeadingDeg(start)));

        Cursor cursor = new Cursor();
        cursor.x = number(start, "x");
        cursor.y = number(start, "y");
        cursor.deg = startHeadingDeg(start);

        // Lines are built in declaration order, chaining geometry from one to the next, and then
        // exposed in the file's run order, so a repeated line replays its drawn geometry.
        List<Map<String, Object>> lines = maps(root.get("lines"));
        List<Path> built = new ArrayList<>();
        List<Map<String, Object>> lineOf = new ArrayList<>();
        Map<String, Integer> builtByLineId = new HashMap<>();
        for (Map<String, Object> line : lines) {
            built.add(buildPath(line, false, cursor, poseFactory));
            lineOf.add(line);
            for (String id : lineIds(line)) {
                if (!id.isEmpty()) {
                    builtByLineId.putIfAbsent(id.toLowerCase(Locale.US), built.size() - 1);
                }
            }
        }

        // A group is one drivable path, but the visualizer lists its children in the sequence
        // separately. Tracking which top-level line has already been exposed keeps a group from
        // being driven once per child.
        Set<Integer> exposedLines = new HashSet<>();
        for (Map<String, Object> step : maps(root.get("sequence"))) {
            if ("wait".equals(string(step.get("kind"), ""))) {
                steps.add(new Step(-1, (long) Math.max(0, number(step, "durationMs", 0))));
                continue;
            }
            Integer index = builtByLineId.get(string(step.get("lineId")));
            if (index == null) {
                continue;
            }
            if (!exposedLines.add(index)) {
                // This line (or its group) was already added by an earlier step.
                if (isCompound(lineOf.get(index))) {
                    continue;
                }
            }
            expose(built.get(index), lineOf.get(index));
            steps.add(new Step(paths.size() - 1, 0));
            // The visualizer pauses after a path for as long as that line's waitAfterMs asks,
            // and the preview timeline shows it, so the pause belongs in the run plan too.
            long pauseAfter = waitAfterMs.get(paths.size() - 1);
            if (pauseAfter > 0) {
                steps.add(new Step(-1, pauseAfter));
            }
        }
        if (paths.isEmpty()) {
            for (int i = 0; i < built.size(); i++) {
                expose(built.get(i), lineOf.get(i));
                steps.add(new Step(i, 0));
                long pauseAfter = waitAfterMs.get(paths.size() - 1);
                if (pauseAfter > 0) {
                    steps.add(new Step(-1, pauseAfter));
                }
            }
        }
    }

    private static boolean isCompound(Map<String, Object> line) {
        return line.get("segments") instanceof List;
    }

    /**
     * One entry of the file's run plan: a path, or a wait before the next path.
     */
    public static final class Step {
        private final int pathIndex;
        private final long waitMs;

        private Step(int pathIndex, long waitMs) {
            this.pathIndex = pathIndex;
            this.waitMs = waitMs;
        }

        /** True when this step is a wait, false when it is a path. */
        public boolean isWait() {
            return pathIndex < 0;
        }

        /** Index of the path to follow; only valid when {@link #isWait()} is false. */
        public int pathIndex() {
            if (pathIndex < 0) {
                throw new IllegalStateException("This step is a wait, not a path.");
            }
            return pathIndex;
        }

        /** Milliseconds to wait; only valid when {@link #isWait()} is true. */
        public long waitMs() {
            if (pathIndex >= 0) {
                throw new IllegalStateException("This step is a path, not a wait.");
            }
            return waitMs;
        }
    }

    /** Number of points: the start point plus one end point per exposed path. */
    public int pointCount() {
        return points.size();
    }

    /** Number of paths. */
    public int pathCount() {
        return paths.size();
    }

    /**
     * Point {@code index}: {@code 0} is the file's start point, higher indices are the line end
     * points in run order.
     */
    public Pose point(int index) {
        return points.get(index);
    }

    /** The point named {@code name} (case-insensitive). */
    public Pose point(String name) {
        return points.get(pointIndex(name));
    }

    /** Index of the point named {@code name} (case-insensitive). */
    public int pointIndex(String name) {
        Integer index = pointsByName.get(name.trim().toLowerCase(Locale.US));
        if (index == null) {
            throw new IllegalArgumentException("No point named '" + name + "'; points are " + pointNames);
        }
        return index;
    }

    /** Name of point {@code index}. */
    public String pointName(int index) {
        return pointNames.get(index);
    }

    /** The file's start point. */
    public Pose startPoint() {
        return points.get(0);
    }

    /** The final end point. */
    public Pose endPoint() {
        return points.get(points.size() - 1);
    }

    /** Path {@code index} in run order, with the heading mode stored in the file. */
    public Path path(int index) {
        return paths.get(index);
    }

    /** The drawn line named {@code name}, i.e. the path arriving at the point sharing its name. */
    public Path path(String name) {
        return paths.get(lineIndex(name));
    }

    /** The path leaving the point named {@code name} for the next point. */
    public Path pathAfter(String name) {
        int index = pointIndex(name);
        if (index >= paths.size()) {
            throw new IllegalArgumentException("Point '" + name + "' is the final point; it has no outgoing path.");
        }
        return paths.get(index);
    }

    /** Milliseconds the visualizer schedules before the {@code index}-th path starts. */
    public long waitBeforeMs(int index) {
        return waitBeforeMs.get(index);
    }

    /** Milliseconds the visualizer schedules after the {@code index}-th path ends. */
    public long waitAfterMs(int index) {
        return waitAfterMs.get(index);
    }

    /** The file's run plan: each path to follow and each wait, in the file's sequence order. */
    public List<Step> steps() {
        return steps;
    }

    private void expose(Path path, Map<String, Object> line) {
        paths.add(path);
        int index = paths.size() - 1;
        String name = string(line.get("name")).trim();
        addLineName(name, index);
        for (String id : lineIds(line)) {
            if (!id.isEmpty()) {
                linesByName.putIfAbsent(id.toLowerCase(Locale.US), index);
            }
        }
        addPoint(name, path.endPose());
        waitBeforeMs.add(waitMs(line, "waitBeforeMs", "waitBefore"));
        waitAfterMs.add(waitMs(line, "waitAfterMs", "waitAfter"));
    }

    private void addPoint(String name, Pose pose) {
        int index = points.size();
        points.add(pose);
        String key = name.isEmpty() ? (index == 0 ? "start" : "point" + index) : name;
        pointNames.add(key);
        pointsByName.putIfAbsent(key.toLowerCase(Locale.US), index);
        if (index == 0) {
            pointsByName.putIfAbsent("start", 0);
        }
    }

    private void addLineName(String name, int index) {
        if (!name.isEmpty()) {
            linesByName.putIfAbsent(name.toLowerCase(Locale.US), index);
        }
    }

    private int lineIndex(String name) {
        Integer index = linesByName.get(name.trim().toLowerCase(Locale.US));
        if (index == null) {
            throw new IllegalArgumentException("No path (line) named '" + name + "'");
        }
        return index;
    }

    private static Path buildPath(Map<String, Object> line, boolean overridden, Cursor cursor, PoseFactory poseFactory) {
        if (line.get("segments") instanceof List) {
            return buildCompound(line, overridden, cursor, poseFactory);
        }
        return atomicPath(line, overridden, cursor, poseFactory);
    }

    private static Path buildCompound(Map<String, Object> line, boolean overridden, Cursor cursor, PoseFactory poseFactory) {
        List<Map<String, Object>> segments = maps(line.get("segments"));
        if (segments.isEmpty()) {
            throw new IllegalArgumentException(".pp file has a compound line without segments");
        }
        Map<String, Object> groupHeading = optObject(line.get("heading"));
        // Only the outermost group heading applies; a nested group's rule is shadowed by an
        // ancestor's, matching the visualizer's effectiveHeadingAt().
        boolean groupOverrides = !overridden && groupHeading != null;
        double startX = cursor.x;
        double startY = cursor.y;

        if (!groupOverrides) {
            List<Path> children = new ArrayList<>();
            for (Map<String, Object> child : segments) {
                children.add(buildPath(child, overridden, cursor, poseFactory));
            }
            return Paths.path(children.toArray(new Path[0]));
        }

        // A group rule runs across the whole group measured by DISTANCE, so child i covers the
        // slice [travelled/total, (travelled + length_i)/total] of its group's ramp. Applying the
        // group rule to each child on its own would restart the ramp for every child instead.
        List<Path> children = new ArrayList<>();
        List<Double> lengths = new ArrayList<>();
        for (Map<String, Object> child : segments) {
            Path built = buildPath(child, true, cursor, poseFactory);
            children.add(built);
            lengths.add(built.curve.length());
        }
        double total = 0;
        for (double length : lengths) {
            total += length;
        }

        Path[] shaped = new Path[children.size()];
        HeadingSpec group = headingSpec(groupHeading, cursor.deg);
        double travelled = 0;
        for (int i = 0; i < children.size(); i++) {
            double from = total <= 1e-9 ? 0 : travelled / total;
            travelled += lengths.get(i);
            double to = total <= 1e-9 ? 1 : travelled / total;
            // Angles ride on the child's own endpoint, as applySpec does for atomic lines.
            Map<String, Object> childEnd = endpointOf(segments.get(i));
            shaped[i] = applyGroupRule(children.get(i), group, from, to, poseFactory,
                    number(childEnd, "x"), number(childEnd, "y"));
        }
        cursor.deg = group.endDeg;
        return Paths.path(shaped);
    }

    /** The endpoint a compound child finishes at, whether atomic or nested. */
    private static Map<String, Object> endpointOf(Map<String, Object> child) {
        Map<String, Object> end = optObject(child.get("endPoint"));
        if (end != null) {
            return end;
        }
        List<Map<String, Object>> nested = maps(child.get("segments"));
        return nested.isEmpty() ? Collections.emptyMap() : endpointOf(nested.get(nested.size() - 1));
    }

    /**
     * Applies the {@code [from, to]} slice of a group's heading rule to one child path. The slice
     * is what that child would have shown if the group were driven as one lump, so the ramp stays
     * continuous across the group's children.
     */
    private static Path applyGroupRule(
            Path child, HeadingSpec group, double from, double to, PoseFactory poseFactory,
            double endX, double endY) {
        if (group.type.startsWith("tangent")) {
            // Tangential is purely geometric: the same rule for every child, no ramp to slice.
            return group.reverse ? child.reverseTangent() : child.tangent();
        }
        if (group.type.equals("piecewise")) {
            return piecewise(child, slicePiecewise(group.piecewise, from, to), endX, endY, poseFactory);
        }
        if (group.type.equals("constant")) {
            return child.constant(poseFactory.of(endX, endY, group.endDeg).heading());
        }
        // linear: re-ramp over this child's own window of the group rule, using the same
        // degree-exact rotation the rule itself uses.
        double startDeg = interpolateDegrees(group.startDeg, group.endDeg, from);
        double endDeg = interpolateDegrees(group.startDeg, group.endDeg, to);
        return child.heading(linearRule(startDeg, endDeg));
    }

    /**
     * The visualizer's shortest rotation from {@code startDeg} to {@code endDeg} at
     * {@code fraction} of the way along, computed in degrees exactly as its math.ts does: an
     * exact half turn resolves to a POSITIVE rotation, which Pedro's radian helpers do not.
     */
    private static double interpolateDegrees(double startDeg, double endDeg, double fraction) {
        double normalizedStart = ((startDeg % 360) + 360) % 360;
        double normalizedEnd = ((endDeg % 360) + 360) % 360;
        double difference = normalizedEnd - normalizedStart;
        if (difference > 180) {
            difference -= 360;
        } else if (difference < -180) {
            difference += 360;
        }
        return startDeg + difference * Math.min(Math.max(fraction, 0), 1);
    }

    /**
     * Re-maps a piecewise rule's nodes onto the {@code [from, to]} window of the group rule, so a
     * child covering part of its group gets the nodes falling inside that window, rescaled to span
     * the child. Nodes entirely outside the window are dropped.
     */
    private static Map<String, Object> slicePiecewise(Map<String, Object> interpolation, double from, double to) {
        List<Map<String, Object>> nodes = maps(interpolation == null ? null : interpolation.get("segments"));
        nodes.sort((left, right) -> Double.compare(number(left, "startProgress", 0), number(right, "startProgress", 0)));
        double span = to - from;
        List<Object> sliced = new ArrayList<>();
        for (Map<String, Object> node : nodes) {
            double overlapStart = Math.max(number(node, "startProgress", 0), from);
            double overlapEnd = Math.min(number(node, "endProgress", 1), to);
            if (span <= 1e-9 || overlapEnd <= overlapStart) {
                continue;
            }
            Map<String, Object> copy = new LinkedHashMap<>(node);
            copy.put("startProgress", (overlapStart - from) / span);
            copy.put("endProgress", (overlapEnd - from) / span);
            sliced.add(copy);
        }
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("segments", sliced);
        return out;
    }

    private static Path atomicPath(Map<String, Object> line, boolean overridden, Cursor cursor, PoseFactory poseFactory) {
        Map<String, Object> end = requiredObject(line.get("endPoint"), "a line endPoint");
        HeadingSpec spec = headingSpec(headingOf(line, end), cursor.deg);
        Path path = curve(cursor.x, cursor.y, end, line, poseFactory);
        if (!overridden) {
            path = applySpec(path, spec, cursor.x, cursor.y, number(end, "x"), number(end, "y"), cursor, poseFactory);
        } else {
            cursor.deg = spec.endDeg;
        }
        cursor.x = number(end, "x");
        cursor.y = number(end, "y");
        return path;
    }

    private static Path curve(double startX, double startY, Map<String, Object> end, Map<String, Object> line, PoseFactory poseFactory) {
        List<Map<String, Object>> through = maps(line.get("throughPoints"));
        if (!through.isEmpty()) {
            Pose[] poses = new Pose[through.size() + 2];
            poses[0] = poseFactory.of(startX, startY, 0);
            for (int i = 0; i < through.size(); i++) {
                poses[i + 1] = poseFactory.of(number(through.get(i), "x"), number(through.get(i), "y"), 0);
            }
            poses[poses.length - 1] = poseFactory.of(number(end, "x"), number(end, "y"), 0);
            return Paths.through(poses);
        }
        List<Vector2D> controls = controls(line, poseFactory);
        if (controls.isEmpty()) {
            return Paths.line(vector(startX, startY, poseFactory), vector(end, poseFactory));
        }
        Vector2D[] points = new Vector2D[controls.size() + 2];
        points[0] = vector(startX, startY, poseFactory);
        points[points.length - 1] = vector(end, poseFactory);
        for (int i = 0; i < controls.size(); i++) {
            points[i + 1] = controls.get(i);
        }
        return Paths.curve(points);
    }

    private static Path applySpec(
            Path path, HeadingSpec spec, double startX, double startY,
            double endX, double endY, Cursor cursor, PoseFactory poseFactory) {
        cursor.deg = spec.endDeg;
        if (spec.type.equals("piecewise")) {
            return piecewise(path, spec.piecewise, endX, endY, poseFactory);
        }
        if (spec.type.startsWith("tangent")) {
            return path.heading(tangentRule(spec.reverse));
        }
        if (spec.type.equals("constant")) {
            // `reverse` on a constant heading is meaningless (the angle never moves), and the
            // visualizer's codegen drops it, so the stored angle is used as-is.
            return path.constant(poseFactory.of(endX, endY, spec.endDeg).heading());
        }
        // A linear rule ramps by DISTANCE the short way round, with an exact half turn resolved
        // as a positive rotation. Pedro's linear() lerps radians and always turns a half turn
        // negative, so the rule is supplied directly to match the visualizer. The visualizer's
        // codegen ignores `reverse` on a linear heading, so the stored angles are used as-is.
        return path.heading(linearRule(spec.startDeg, spec.endDeg));
    }

    /**
     * The visualizer's tangential rule: face along the direction of travel, optionally a half turn
     * round. Pedro's own {@code Interpolator.tangent} samples the curve at the arc-length
     * completion as if it were the Bezier parameter, which visibly twists the heading on curved
     * paths, so the true tangent at the real parameter is used instead.
     */
    private static Interpolator tangentRule(boolean reverse) {
        return Interpolator.geometric((curve, t) -> {
            Vector2D direction = curve.tangent(t);
            double theta = Math.atan2(direction.y(), direction.x());
            return reverse ? theta + Math.PI : theta;
        });
    }

    /**
     * The visualizer's linear rule: shortest rotation from {@code startDeg} to {@code endDeg} as a
     * function of arc-length progress, with a tie at 180 degrees going positive.
     */
    private static Interpolator linearRule(double startDeg, double endDeg) {
        return Interpolator.geometric((curve, t) ->
                Math.toRadians(interpolateDegrees(startDeg, endDeg, curve.pathCompletion(t))));
    }

    private static Path piecewise(Path path, Map<String, Object> interpolation, double endX, double endY, PoseFactory poseFactory) {
        List<Map<String, Object>> segments = new ArrayList<>(
                maps(interpolation == null ? null : interpolation.get("segments")));
        segments.sort((left, right) -> Double.compare(number(left, "startProgress", 0), number(right, "startProgress", 0)));

        List<Double> bounds = new ArrayList<>();
        List<Interpolator> nodes = new ArrayList<>();
        double previousT = 0;
        for (int i = 0; i < segments.size(); i++) {
            double untilT = Math.min(Math.max(number(segments.get(i), "endProgress", 1), 0), 1);
            if (untilT <= previousT) {
                continue;
            }
            Interpolator node = nodeInterpolator(segments, i, endX, endY, path, poseFactory);
            if (node == null) {
                continue;
            }
            bounds.add(untilT);
            nodes.add(node);
            previousT = untilT;
        }
        if (nodes.isEmpty()) {
            return path.heading(tangentRule(false));
        }
        bounds.set(bounds.size() - 1, 1.0);
        PiecewiseInterpolator piecewise = Interpolator.piecewise();
        for (int i = 0; i < nodes.size(); i++) {
            piecewise.until(bounds.get(i), nodes.get(i));
        }
        return path.heading(piecewise);
    }

    private static Interpolator nodeInterpolator(
            List<Map<String, Object>> segments, int index, double endX, double endY, Path path, PoseFactory poseFactory) {
        Map<String, Object> segment = segments.get(index);
        Map<String, Object> parameters = optObject(segment.get("parameters"));
        String type = string(segment.get("interpolationType"), "linear").toLowerCase(Locale.US);
        boolean reversed = Boolean.TRUE.equals(segment.get("reversed"));
        Double linked = linkedHeading(segments, index, endX, endY, path, poseFactory);
        switch (type) {
            case "tangential":
                // Piecewise nodes receive arc-length completion, so a plain tangent() would sample
                // the curve at the wrong parameter; the geometric rule recovers the real one.
                return geometricTangent(reversed);
            case "facing-point": {
                Map<String, Object> target = optObject(parameters == null ? null : parameters.get("point"));
                if (target == null) {
                    return null;
                }
                Interpolator facing = Interpolator.facingPoint(vector(target, poseFactory));
                return reversed ? facing.reverse() : facing;
            }
            case "constant":
                return Interpolator.constant(
                        linked != null ? linked : poseFactory.of(endX, endY, number(parameters, "degrees", 0)).heading());
            case "linear":
            default: {
                double startHeading = linked != null
                        ? linked
                        : poseFactory.of(endX, endY, number(parameters, "startDeg", 0)).heading();
                double endHeading = poseFactory.of(endX, endY, number(parameters, "endDeg", 0)).heading();
                return reversed
                        ? Interpolator.longLinear(startHeading, endHeading)
                        : Interpolator.linear(startHeading, endHeading);
            }
        }
    }

    /**
     * A tangential node for the piecewise interpolator, which hands its nodes the arc-length
     * completion rather than the Bezier parameter. Completion and parameter are inverse functions
     * of each other, so the parameter is looked back up before sampling the tangent.
     */
    private static Interpolator geometricTangent(boolean reversed) {
        return Interpolator.geometric((curve, completion) -> {
            // Solve completion(t) = c by bisection; curve.pathCompletion is monotonic.
            double low = 0;
            double high = 1;
            for (int i = 0; i < 30; i++) {
                double mid = (low + high) / 2;
                if (curve.pathCompletion(mid) < completion) {
                    low = mid;
                } else {
                    high = mid;
                }
            }
            Vector2D direction = curve.tangent((low + high) / 2);
            double theta = Math.atan2(direction.y(), direction.x());
            return reversed ? theta + Math.PI : theta;
        });
    }

    /**
     * A linear or constant node set to continue from the previous one inherits the angle that
     * segment ended on; Pedro cannot express the link, so it is resolved to a number here.
     */
    private static Double linkedHeading(
            List<Map<String, Object>> segments, int index, double endX, double endY, Path path, PoseFactory poseFactory) {
        if (index == 0 || !Boolean.TRUE.equals(segments.get(index).get("continueFromPrevious"))) {
            return null;
        }
        double boundary = number(segments.get(index), "startProgress", 0);
        return previousEndHeading(segments, index - 1, boundary, endX, endY, path, poseFactory);
    }

    private static Double previousEndHeading(
            List<Map<String, Object>> segments, int index, double boundary, double endX, double endY,
            Path path, PoseFactory poseFactory) {
        Map<String, Object> segment = segments.get(index);
        Map<String, Object> parameters = optObject(segment.get("parameters"));
        String type = string(segment.get("interpolationType"), "linear").toLowerCase(Locale.US);
        boolean reversed = Boolean.TRUE.equals(segment.get("reversed"));
        switch (type) {
            case "constant":
                if (index > 0 && Boolean.TRUE.equals(segment.get("continueFromPrevious"))) {
                    return previousEndHeading(segments, index - 1, boundary, endX, endY, path, poseFactory);
                }
                return poseFactory.of(endX, endY, number(parameters, "degrees", 0)).heading();
            case "tangential":
            case "facing-point": {
                double t = path.curve.parameter(Math.min(Math.max(boundary, 0), 1));
                if (type.equals("tangential")) {
                    return path.curve.tangent(t).theta() + (reversed ? Math.PI : 0);
                }
                Map<String, Object> target = optObject(parameters == null ? null : parameters.get("point"));
                if (target == null) {
                    return null;
                }
                return vector(target, poseFactory).minus(path.curve.get(t)).theta() + (reversed ? Math.PI : 0);
            }
            default:
                return poseFactory.of(endX, endY, number(parameters, "endDeg", 0)).heading();
        }
    }

    /**
     * Resolves a line's heading, preferring the current heading object and falling back to the
     * pre-migration form: a type string with its parameters stored on the end point.
     */
    private static Map<String, Object> headingOf(Map<String, Object> line, Map<String, Object> end) {
        Object heading = line.get("heading");
        if (heading instanceof Map) {
            return asMap(heading);
        }
        String type = heading instanceof String ? (String) heading : string(end.get("heading"), "");
        if (type.isEmpty()) {
            return null;
        }
        Map<String, Object> legacy = new LinkedHashMap<>();
        legacy.put("type", type);
        for (String key : new String[] {"startDeg", "endDeg", "degrees", "reverse", "piecewiseHeading"}) {
            if (end.containsKey(key)) {
                legacy.put(key, end.get(key));
            }
        }
        return legacy;
    }

    private static HeadingSpec headingSpec(Map<String, Object> heading, double fallbackStartDeg) {
        if (heading == null) {
            return new HeadingSpec("tangential", false, fallbackStartDeg, fallbackStartDeg, null);
        }
        String type = string(heading.get("type"), "tangential").toLowerCase(Locale.US);
        double startDeg = number(heading, "startDeg", fallbackStartDeg);
        double endDeg = type.equals("constant")
                ? number(heading, "degrees", startDeg)
                : number(heading, "endDeg", startDeg);
        return new HeadingSpec(type, Boolean.TRUE.equals(heading.get("reverse")), startDeg, endDeg,
                optObject(heading.get("piecewiseHeading")));
    }

    private static final class HeadingSpec {
        private final String type;
        private final boolean reverse;
        private final double startDeg;
        private final double endDeg;
        private final Map<String, Object> piecewise;

        private HeadingSpec(
                String type, boolean reverse, double startDeg, double endDeg, Map<String, Object> piecewise) {
            this.type = type;
            this.reverse = reverse;
            this.startDeg = startDeg;
            this.endDeg = endDeg;
            this.piecewise = piecewise;
        }
    }

    /** File-space (pre-factory) position and heading, chained from one line to the next. */
    private static final class Cursor {
        private double x;
        private double y;
        private double deg;
    }

    /**
     * The start point's display name. The visualizer only writes {@code name} when the point was
     * named, so an anonymous start point is normal and falls back to {@code StartPoint} rather than
     * leaving point 0 unnamed (which {@link #pointName(int)} would otherwise report as "start").
     */
    private static String startPointName(Map<String, Object> start) {
        String name = string(start.get("name")).trim();
        return name.isEmpty() ? "StartPoint" : name;
    }

    /**
     * The start point's heading in file degrees, from either the current {@code headingDeg} field or
     * the pre-migration heading type stored on the point. An unreadable heading means 0, matching
     * the visualizer's own normalization.
     */
    private static double startHeadingDeg(Map<String, Object> start) {
        if (start.get("headingDeg") instanceof Number) {
            return number(start, "headingDeg", 0);
        }
        String type = string(start.get("heading"), "");
        if (type.equals("linear")) {
            return number(start, "startDeg", 0);
        }
        if (type.equals("constant")) {
            return number(start, "degrees", 0);
        }
        if (type.equals("tangential")) {
            // A tangential start has no stored angle; the pose factory treats 0 as "as drawn".
            return 0;
        }
        return 0;
    }

    private static List<Vector2D> controls(Map<String, Object> line, PoseFactory poseFactory) {
        List<Object> entries = new ArrayList<>(list(line.get("controlPoints")));
        entries.add(line.get("controlPointOne"));
        entries.add(line.get("controlPointTwo"));
        List<Vector2D> controls = new ArrayList<>();
        for (Object entry : entries) {
            Map<String, Object> point = optObject(entry);
            if (point != null && point.get("x") instanceof Number && point.get("y") instanceof Number) {
                controls.add(vector(point, poseFactory));
            }
        }
        return controls;
    }

    private static void collectLineIds(Map<String, Object> line, List<String> ids) {
        ids.add(string(line.get("id")));
        for (Map<String, Object> child : maps(line.get("segments"))) {
            collectLineIds(child, ids);
        }
    }

    private static List<String> lineIds(Map<String, Object> line) {
        List<String> ids = new ArrayList<>();
        collectLineIds(line, ids);
        return ids;
    }

    private static long waitMs(Map<String, Object> line, String msKey, String segmentKey) {
        Object direct = line.get(msKey);
        double ms = direct instanceof Number
                ? ((Number) direct).doubleValue()
                : number(optObject(line.get(segmentKey)), "durationMs", 0);
        return (long) Math.max(0, ms);
    }

    private static Vector2D vector(double x, double y, PoseFactory poseFactory) {
        return poseFactory.of(x, y, 0).toVector2D();
    }

    private static Vector2D vector(Map<String, Object> point, PoseFactory poseFactory) {
        return vector(number(point, "x"), number(point, "y"), poseFactory);
    }

    private static double number(Map<String, Object> map, String key) {
        if (map == null || !(map.get(key) instanceof Number)) {
            throw new IllegalArgumentException(".pp file is missing number '" + key + "'");
        }
        return ((Number) map.get(key)).doubleValue();
    }

    private static double number(Map<String, Object> map, String key, double fallback) {
        Object value = map == null ? null : map.get(key);
        return value instanceof Number ? ((Number) value).doubleValue() : fallback;
    }

    private static String string(Object value) {
        return value instanceof String ? (String) value : "";
    }

    private static String string(Object value, String fallback) {
        return value instanceof String ? (String) value : fallback;
    }

    private static Map<String, Object> requiredObject(Object value, String what) {
        if (!(value instanceof Map)) {
            throw new IllegalArgumentException(".pp file is missing " + what);
        }
        return asMap(value);
    }

    private static Map<String, Object> optObject(Object value) {
        return value instanceof Map ? asMap(value) : null;
    }

    private static Map<String, Object> asMap(Object value) {
        @SuppressWarnings("unchecked")
        Map<String, Object> map = (Map<String, Object>) value;
        return map;
    }

    private static List<?> list(Object value) {
        return value instanceof List ? (List<?>) value : Collections.emptyList();
    }

    private static List<Map<String, Object>> maps(Object value) {
        List<Map<String, Object>> maps = new ArrayList<>();
        for (Object entry : list(value)) {
            Map<String, Object> map = optObject(entry);
            if (map != null) {
                maps.add(map);
            }
        }
        return maps;
    }
}
