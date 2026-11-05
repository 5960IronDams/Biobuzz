#!/usr/bin/env python3
"""
Biobuzz AimPoints.pp generator — places one aim point per hive ON the hive
centerline encoded in the .fmap, pushed a tunable DEPTH further out from
field center.

WHERE THE NUMBERS COME FROM: this script does not re-derive hive geometry.
It IMPORTS fmap_generator.py (same folder) and builds the field map with the
SAME CONFIG that writes field_spec.fmap, then reads the tag centers straight
out of it. Change a tape override or the pitch in fmap_generator.py and the
aim points follow automatically — there is no second copy of the geometry.

FRAMES:
  * fmap_generator emits the Limelight/FTC field frame: center origin,
    +X_LL = Pedro south, +Y_LL = Pedro east, meters. The robot code maps it
    the same way (Vision.java: field_x = ll_y, field_y = -ll_x).
  * AimPoints.pp is written in the PEDRO frame the visualizer and
    AimAtGoal/PPFile expect: corner origin, 0..144 in, heading 0 = +X.
      px = ll_y + 72,   py = -ll_x + 72

GEOMETRY (what "on the centerline" means):
  * A hive's four tags are strung along Pedro X. The hive CENTERLINE is the
    line px = <tag-row midpoint>; horizontally it crosses the field at a
    fixed px and every py. (The brace gap runs along it.)
  * The tag ROW is at a fixed py per side. DEPTH walks the aim point off that
    row along the centerline, toward field center (audience side) or toward
    the scoring wall (scoring side):
        AUDIENCE  py = tag_row_py - DEPTH_IN  (field center, py 72, is
                                              audience-side)
        SCORING   py = tag_row_py + DEPTH_IN  (scoring wall at py 144)
    DEPTH 0       -> on the tag row, middle of the brace gap
    DEPTH 10.6875 -> the TRUE hive center (4-tag cluster midpoint)
    DEPTH 6       -> the starting guess: ~4.7 in past the tag row, still
                     ~4.7 in shy of the hive center

OUTPUT: a .pp in the AimPoints.pp layout — startPoint "Robot" (where the
robot parks, so nothing aims at the stored start) plus atomic lines named
RedFront / RedBack / BlueFront / BlueBack. AimAtGoal filters names on the
"Red*"/"Blue*" prefix and picks the closest point, so names only need the
alliance word.

INTERFACE: edit the CONFIG block; the only CLI knob is --out.
"""

import argparse
import json
import uuid

import fmap_generator as fg

# ------------------------- CONFIG (defaults) -------------------------
# ---- DEPTH ----
# How far OUT the aim points sit, in inches, walked off the tag row along
# the hive centerline. See GEOMETRY above. Set this and forget the rest.
DEPTH_IN = 6.0

# ---- POINT NAMES ----
# The "front" points are the audience-side hives, the "back" points the
# scoring-side hives, matching the names in the hand-built AimPoints.pp.
AUDIENCE_PREFIX = "Front"
SCORING_PREFIX = "Back"

# ---- PARKED START (the "Robot" point; NOT an aim target) ----
START_Y_IN = 9.639629200463489
START_HEADING_DEG = 90.0
START_NAME = "Robot"

# ---- FILE FORMAT ----
COLORS = {"RedFront": "#ffc516", "RedBack": "#6AAA6B",
          "BlueBack": "#7DB9AC", "BlueFront": "#C97D77"}
VERSION = "1.5.0"
# ---------------------------------------------------------------------
# The tag clusters, as member-id groups in the order the fmap lays them
# along +Pedro X. Audience-side hives sit on one tag row, scoring-side hives
# on the other; red is the low-Pedro-X half (LL -Y), blue the high half.
CLUSTERS = {
    "audience": {"red": (34, 35, 36, 37), "blue": (38, 39, 40, 41)},
    "scoring": {"red": (30, 31, 32, 33), "blue": (42, 43, 44, 45)},
}

M2IN = 1.0 / 0.0254
FIELD_CENTER_IN = fg.FIELD_HALF_IN     # 72


# ------------------------- geometry -------------------------

def build_spec_fmap():
    """The same field map fmap_generator.main() writes, from the same CONFIG."""
    return fg.build_fmap(fg.TAG_SIZE_MM, fg.TAG_CENTER_IN, fg.FACE_PITCH_DEG,
                         fg.CELL_X_IN, fg.HIVE_SEPARATION_IN,
                         fg.INNER_TAG_OFFSET_IN, fg.OUTER_TAG_OFFSET_IN)


def tags_in_pedro(fmap):
    """{tag_id: (pedro_x, pedro_y)} in inches."""
    out = {}
    for f in fmap["fiducials"]:
        t = f["transform"]
        ll_x_in, ll_y_in = t[3] * M2IN, t[7] * M2IN
        # Vision.java: field_x = ll_y, field_y = -ll_x, then +72 for corner origin.
        out[f["id"]] = (ll_y_in + FIELD_CENTER_IN, -ll_x_in + FIELD_CENTER_IN)
    return out


# Built once: the fmap never changes inside one run.
_TAGS = tags_in_pedro(build_spec_fmap())


def centerline_x(tags, ids):
    """Hive centerline X: midpoint of the furthest-apart member tags.

    With symmetric tag offsets this is the tag-row midpoint; with an
    asymmetric inner/outer config it is still the hive centerline.
    """
    xs = [tags[i][0] for i in ids]
    return (min(xs) + max(xs)) / 2.0


def tag_row_y(tags, ids):
    """The py all four member tags share (the tag row for that side)."""
    return sum(tags[i][1] for i in ids) / len(ids)


def hive_center(tags, ids):
    """True hive center: the 4-tag midpoint, i.e. the middle of the brace gap."""
    return (centerline_x(tags, ids), tag_row_y(tags, ids))


def depth_out(side, tag_row_y_in, depth_in):
    """Pedro Y of a point `depth_in` out from the tag row, along the centerline."""
    if side == "audience":
        return tag_row_y_in - depth_in     # toward field center (py 72)
    return tag_row_y_in + depth_in         # toward the scoring wall (py 144)


def depth_to_hive_center(side, tags, ids):
    """The DEPTH_IN that lands exactly on the true hive center for this side."""
    return abs(tag_row_y(tags, ids) - FIELD_CENTER_IN)


def all_points(depth_in=DEPTH_IN, tags=_TAGS):
    """[(name, side, alliance, pedro_x, pedro_y)] — a stable, file-ready order."""
    pts = []
    for side in ("audience", "scoring"):
        for alliance in ("red", "blue"):
            ids = CLUSTERS[side][alliance]
            x = centerline_x(tags, ids)
            y = depth_out(side, tag_row_y(tags, ids), depth_in)
            name = alliance.capitalize() + (
                AUDIENCE_PREFIX if side == "audience" else SCORING_PREFIX)
            pts.append((name, side, alliance, x, y))
    return pts


# ------------------------- file emission -------------------------

def line_payload(name, color, x, y, reverse):
    """One atomic line placeholdering a point at (x, y).

    AimAtGoal reads only a line's END POINT, so the heading fields are inert
    decorations — copied from the hand-built file's style (a linear heading on
    the front points, a reversed tangential heading on the back ones).
    """
    heading = ({"type": "tangential", "reverse": True} if reverse
               else {"type": "linear", "startDeg": START_HEADING_DEG,
                     "endDeg": 180.0})
    return {
        "id": "line-" + uuid.uuid4().hex[:8] + "-" + uuid.uuid4().hex[:6],
        "color": color,
        "name": name,
        "locked": False,
        "waitBeforeMs": 0,
        "waitAfterMs": 0,
        "waitBeforeName": "",
        "waitAfterName": "",
        "kind": "atomic",
        "endPoint": {"x": round(x, 6), "y": round(y, 6)},
        "controlPoints": [],
        "heading": heading,
    }


def build_pp(depth_in=DEPTH_IN):
    pts = all_points(depth_in)
    lines = [
        line_payload(name, COLORS[name], x, y, reverse=(side == "scoring"))
        for name, side, _alliance, x, y in pts
    ]
    # Parked-start X: the scoring-row X of the red hive, matching the
    # hand-built file's approx. 58.1. The start point is not an aim target.
    start_x = next(x for name, side, alliance, x, _y in pts
                   if side == "scoring" and alliance == "red")
    return {
        "startPoint": {
            "x": round(start_x, 6),
            "y": START_Y_IN,
            "name": START_NAME,
            "locked": False,
            "headingDeg": START_HEADING_DEG,
        },
        "lines": lines,
        "shapes": [],
        "sequence": [],
        "fieldPoints": [],
        "version": VERSION,
    }


# ------------------------- reporting -------------------------

def sanity_report(depth_in=DEPTH_IN, tags=_TAGS):
    """Print the geometry so you can eyeball it against the field."""
    print(f"tag z (center height): {fg.TAG_CENTER_IN:.3f} in, pitch {fg.FACE_PITCH_DEG:g} deg")
    red_x = centerline_x(tags, CLUSTERS['audience']['red'])
    blue_x = centerline_x(tags, CLUSTERS['audience']['blue'])
    print(f"red<->blue centerline separation: {blue_x - red_x:.3f} in "
          f"(each {abs(red_x - FIELD_CENTER_IN):.3f} in from field center)")
    print(f"track row py: audience "
          f"{tag_row_y(tags, CLUSTERS['audience']['red']):.3f} / scoring "
          f"{tag_row_y(tags, CLUSTERS['scoring']['red']):.3f}  "
          f"(field center {FIELD_CENTER_IN:.1f}, scoring wall "
          f"{2 * FIELD_CENTER_IN:.1f})")
    print(f"depth: {depth_in:g} in out from the tag row")
    print()
    print(f"  {'name':>10}  {'px':>8}  {'py':>8}  {'hive cx':>8}  {'hive cy':>8}  "
          f"{'d_row':>7}  {'d_depth':>8}  {'miss':>6}")
    rows = all_points(depth_in, tags)
    for name, side, alliance, x, y in rows:
        ids = CLUSTERS[side][alliance]
        cx, cy = hive_center(tags, ids)
        print(f"  {name:>10}  {x:8.3f}  {y:8.3f}  {cx:8.3f}  {cy:8.3f}  "
              f"{x - cx:+7.3f}  {y - cy:+8.3f}  "
              f"{(abs(depth_in - depth_to_hive_center(side, tags, ids))):6.3f}")
    print()
    print("d_row and d_depth are offsets from the TRUE hive center (the 4-tag "
          "midpoint).")
    print("d_row is 0.000 by construction (on the centerline); d_depth is the "
          "depth error.")
    for side in ("audience", "scoring"):
        ids = CLUSTERS[side]["red"]
        print(f"depth on {side} hive center = "
              f"{depth_to_hive_center(side, tags, ids):.4f} in")


def main():
    ap = argparse.ArgumentParser(
        description="AimPoints.pp generator — hive centerlines from "
                    "fmap_generator.py, plus a tunable depth")
    ap.add_argument("--out", default="AimPoints.pp", help="output .pp path")
    ap.add_argument("--depth", type=float, default=None,
                    help="override DEPTH_IN for this run (inches out from the tag row)")
    args = ap.parse_args()
    depth = DEPTH_IN if args.depth is None else args.depth

    pp = build_pp(depth)
    with open(args.out, "w") as fh:
        json.dump(pp, fh, indent=2)
        fh.write("\n")

    print(f"wrote {args.out} ({len(pp['lines'])} aim points)")
    sanity_report(depth)


if __name__ == "__main__":
    main()
