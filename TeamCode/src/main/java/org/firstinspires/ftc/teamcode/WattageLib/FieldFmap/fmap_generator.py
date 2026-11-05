#!/usr/bin/env python3
"""
Biobuzz .fmap generator — builds a 16-tag Limelight field map for 4 mirrored
hive cells from a handful of physical measurements.

INTERFACE: edit the CONFIG block below — every physical knob lives there, and
the derived values (tag center height, member offsets) re-derive automatically
when their inputs change. The only command-line argument is --out (output
path); running with no arguments writes the standardized field map.

Frame conventions (validated on-field 2026-09-30):
  * The .fmap lives in the Limelight/FTC field frame: center origin, +X = Pedro
    south, +Y = Pedro east, +Z up, units meters. (Pedro's frame is corner-origin
    0-144 in, heading 0 = +X; the mapping is px = ll_y + 72, py = -ll_x + 72.)
  * Faces rest tipped FACE_PITCH_DEG from vertical (30 = rest): the normal is
    (cos p, 0, sin p) in the LL frame for the audience-facing cells. ONE
    angle for both sides -- the sides differ only in lean DIRECTION, always
    AWAY from field center, chosen by which side of the X axis the CELL sits
    on (the mirrored basis handles the flip; never the id group), so
    swapping id groups between cells keeps each face's lean correct.
  * HIVE CENTERLINES: each hive's centerline runs down the middle of its
    AprilTag cluster, so a cell's face-center sits ON its hive centerline.
    The red and blue centerlines are symmetric about field center 0,0 and
    HIVE_SEPARATION_IN apart, so each is HIVE_SEPARATION_IN/2 (12.75 in) from
    0,0 along the row axis (LL-Y = Pedro-X).
  * SPEC CHAIN: the tag references derive from the drawings, not a tape --
    pivot axis 43.95 above the floor; the cell pair (42.91 laid flat
    end-to-end) pivots at ITS center, the tag-bearing cell rising up-slope
    from the pivot; reference holes 9.938 along the surface from the cell's
    lower lip; tag centers 2.75 along the surface ABOVE the holes; the
    furthest-out bottom edge (the pivot lip) 53.5 horizontally from the
    audience/scoring wall. The chain closes on itself: it puts the tag
    bottom edge at 53.531 -- matching the drawings' 53.5 opening dimension
    to 1/32 in. From it: TAG_BOTTOM_IN (bottom-edge height) and
    CELL_X_BOTTOM_IN (perpendicular-centerline -> bottom edge) both derive --
    and either can be SET DIRECTLY via TAPED_TAG_BOTTOM_IN /
    TAPED_CELL_X_BOTTOM_IN (tape day): a direct tape supersedes the chain and
    absorbs the pivot's unmodeled drop center, the known unknown.
  * The four cells are axis-mirrors of one cell's position (CELL_X_IN,
    +/-HIVE_SEPARATION_IN/2), and mirrored cells' member positions are FULL
    point-mirrors of the original cell's member positions (so member offsets
    reverse in LL-Y on the blue cells):
      34-37 RED AUDIENCE   at (+cx, -sep/2)   ascending +LL-Y
      38-41 BLUE AUDIENCE  at (+cx, +sep/2)   ascending +LL-Y (offsets reversed)
      30-33 RED SCORING    at (-cx, -sep/2)   descending +LL-Y (offsets reversed)
      42-45 BLUE SCORING   at (-cx, +sep/2)   descending +LL-Y
    Mirroring the facing flips the tag basis; c1 flips sign to keep det = +1.
  * Member tags sit in a horizontal row on the tilted face: positions differ
    only in LL-Y by offsets measured FROM THE HIVE CENTERLINE --
    INNER_TAG_OFFSET_IN (2.5) for the two inner tags, OUTER_TAG_OFFSET_IN
    (6.5) for the two outer tags -> (-6.5, -2.5, +2.5, +6.5, gaps
    4.0 / 5.0 / 4.0 in). Same z for all four (the row is horizontal; the
    tilt is about the horizontal axis).
  * Row order along the row axis (+LL-Y = +Pedro-X), matching the real field:
    travelling in +LL-Y the audience side reads 34,35,36,37 then 38,39,40,41,
    and the scoring side reads 33,32,31,30 then 45,44,43,42 -- both hives on
    a side run the SAME physical direction, so the blue-audience and
    red-scoring cells take the REVERSED member offsets.
  * HEIGHT: the map stores each tag's CENTER. TAG_BOTTOM_IN (the black
    square's BOTTOM edge height -- derived from the spec chain above) feeds
    tag_center_in(): on a face pitched p from vertical the center sits
    size/2 ALONG THE FACE above the bottom edge but only (size/2)*cos(p) of
    that is vertical rise, so TAG_CENTER_IN = TAG_BOTTOM_IN + (size/2)*cos(p)
    -- ONE derived height shared by all 16 tags.
"""

import argparse
import json
import math

# ------------------------- CONFIG (defaults) -------------------------
TAG_SIZE_MM = 82.55            # black-square edge, mm (3.25 in)
FACE_PITCH_DEG = 30.0          # tilt of the hive faces from vertical (30 = rest).
                               # ONE angle for both sides: they differ only in
                               # lean DIRECTION (away from field center), which
                               # the mirror handles automatically.
FIELD_HALF_IN = 72.0           # field half-size (144 x 144 in field)

# ---- FIELD-TAPE OVERRIDES (tape day) ----
# Tape straight to the BOTTOM EDGE of the black squares (the solid lowest
# line): vertical tape for the height, flat floor tape from the perpendicular
# centerline for X. Fill these in and the map derives from YOUR tape,
# bypassing the chain below -- a direct tape also absorbs the pivot's
# unmodeled drop center, because it measures the final position. Leave None
# to derive from the spec chain.
TAPED_TAG_BOTTOM_IN = 48.75
TAPED_CELL_X_BOTTOM_IN = 11.5

# ---- SPEC CHAIN (Game Manual / production drawings; used while TAPED_* are None) ----
# The four numbers the drawings give; the references below re-derive from
# them. The pivot's drop center is NOT modeled -- it shifts tags
# ~ +0.87*D in X and +0.50*D in Z for a D-inch offset, so tape the real
# pivot before trusting the last decimal.
PIVOT_HEIGHT_IN = 43.95        # pivot axis above the floor
HOLES_FROM_LIP_IN = 9.938      # cell's lower lip -> reference holes, ALONG the surface
TAG_ABOVE_HOLES_IN = 2.75      # tag centers above the reference holes, ALONG the surface
LIP_FROM_WALL_IN = 53.5        # furthest-out bottom edge (the pivot lip) from
                               # the audience/scoring wall, horizontal

# along-surface distance, pivot/lip -> tag BOTTOM edge: up past the holes,
# then the tag CENTER rides (size/2) further up the face, so the BOTTOM edge
# is (HOLES + ABOVE - size/2) along the surface from the lip.
_LIP_TO_BOTTOM_SRF = HOLES_FROM_LIP_IN + TAG_ABOVE_HOLES_IN - TAG_SIZE_MM / 25.4 / 2.0
_TAG_BOTTOM_CHAIN = PIVOT_HEIGHT_IN + _LIP_TO_BOTTOM_SRF * math.cos(math.radians(FACE_PITCH_DEG))
_CELL_X_BOTTOM_CHAIN = (FIELD_HALF_IN - LIP_FROM_WALL_IN) - _LIP_TO_BOTTOM_SRF * math.sin(math.radians(FACE_PITCH_DEG))
TAG_BOTTOM_IN = _TAG_BOTTOM_CHAIN if TAPED_TAG_BOTTOM_IN is None else TAPED_TAG_BOTTOM_IN
                               # VERTICAL height of the BOTTOM EDGE of the black
                               # square -- the solid lowest line on each tag --
                               # above the floor, inches
CELL_X_BOTTOM_IN = _CELL_X_BOTTOM_CHAIN if TAPED_CELL_X_BOTTOM_IN is None else TAPED_CELL_X_BOTTOM_IN
                               # HORIZONTAL offset from the field's X centerline
                               # -- the centerline through 0,0 running
                               # PERPENDICULAR to the hive centerlines -- to the
                               # BOTTOM EDGE of the black squares (derived: the
                               # lip sits FIELD_HALF_IN - LIP_FROM_WALL_IN from
                               # the centerline, and the face rises inward as
                               # it rises up, pulling the bottom edge inward by
                               # (lip->bottom)*sin(pitch))

def tag_center_in(bottom_in, pitch_deg, size_mm=TAG_SIZE_MM):
    """Vertical height of a tilted tag's CENTER above the floor, inches.

    The face is pitched pitch_deg from vertical about the horizontal row axis,
    so the square's center lies size/2 ALONG THE FACE up from the bottom edge,
    but only (size/2)*cos(pitch) of that distance is vertical rise (ladder
    rule: rise per unit along the face = cos(angle from vertical)). A straight
    vertical face (pitch 0) reduces to the naive bottom + size/2; at the 30 deg
    rest pitch the true center sits (size/2)*(1 - cos 30) = 0.218 in LOWER.
    """
    return bottom_in + (size_mm / 25.4 / 2.0) * math.cos(math.radians(pitch_deg))


TAG_CENTER_IN = tag_center_in(TAG_BOTTOM_IN, FACE_PITCH_DEG)
                               # CENTER height of EVERY hive-cluster tag above
                               # the floor, inches: the tilt-corrected value
                               # derived from TAG_BOTTOM_IN above.

CELL_X_IN = CELL_X_BOTTOM_IN - (TAG_SIZE_MM / 25.4 / 2.0) * math.sin(math.radians(FACE_PITCH_DEG))
                               # CENTER-X magnitude for all 16 tags: the tilted
                               # face slopes down-and-away, so the bottom edge
                               # sits (size/2)*sin(pitch) FURTHER from the
                               # centerline than the tag centers. Audience cells
                               # sit +CELL_X_IN of it, scoring cells the mirror.
HIVE_SEPARATION_IN = 25.5      # red<->blue hive CENTERLINE separation, inches;
                               # symmetric about 0,0 -> each centerline sits
                               # HIVE_SEPARATION_IN/2 = 12.75 in from field center
INNER_TAG_OFFSET_IN = 2.5      # inner tag centers from their hive centerline, inches
OUTER_TAG_OFFSET_IN = 6.5      # outer tag centers from their hive centerline, inches
TAG_FAMILY = "apriltag3_36h11_classic"
# ---------------------------------------------------------------------


def basis_for_facing(pitch_rad: float, mirrored: bool):
    """Return (c0, c1, c2) tag basis columns for a face at rest pitch.

    mirrored=False: faces -X_LL audience side (normal (cos p, 0, sin p)).
    mirrored=True : the point-mirror across the LL X axis (Pedro's Y middle);
                    c1 flips sign to keep the basis right-handed (det = +1).
    """
    s, c = math.sin(pitch_rad), math.cos(pitch_rad)
    if not mirrored:
        c0 = (s, 0.0, -c)      # down-the-slope
        c1 = (0.0, 1.0, 0.0)   # face horizontal (Pedro east)
        c2 = (c, 0.0, s)       # normal, up at pitch
    else:
        c0 = (-s, 0.0, -c)
        c1 = (0.0, -1.0, 0.0)
        c2 = (-c, 0.0, s)
    return c0, c1, c2


def transform_row(basis, tx: float, ty: float, tz: float):
    """Row-major 4x4 with rotation columns c0|c1|c2 (Limelight fmap layout)."""
    c0, c1, c2 = basis
    return [
        c0[0], c1[0], c2[0], tx,
        c0[1], c1[1], c2[1], ty,
        c0[2], c1[2], c2[2], tz,
        0.0, 0.0, 0.0, 1.0,
    ]


def check_orthonormal(row, tol=1e-9):
    """Sanity: columns orthonormal, det = +1."""
    c0 = row[0:3]
    c1 = row[4:7]
    c2 = row[8:11]
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))
    cross = lambda a, b: (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    det = dot(c0, cross(c1, c2))
    ok = (abs(dot(c0, c1)) < tol and abs(dot(c0, c2)) < tol and abs(dot(c1, c2)) < tol
          and abs(det - 1.0) < tol)
    return ok, det


def build_fmap(size_mm=TAG_SIZE_MM, center_in=TAG_CENTER_IN, pitch_deg=FACE_PITCH_DEG,
               cell_x_in=CELL_X_IN, hive_separation_in=HIVE_SEPARATION_IN,
               inner_offset_in=INNER_TAG_OFFSET_IN, outer_offset_in=OUTER_TAG_OFFSET_IN,
               offsets_in=None):
    """offsets_in: optional explicit 4-tuple override; otherwise member offsets
    are derived from the hive centerline as (-outer, -inner, +inner, +outer).
    center_in: one shared, tilt-corrected tag-center height for all 16 tags."""
    center_h_m = center_in * 0.0254
    pitch_rad = math.radians(pitch_deg)
    half_m = FIELD_HALF_IN * 0.0254

    if offsets_in is None:
        offsets_in = (-outer_offset_in, -inner_offset_in,
                      inner_offset_in, outer_offset_in)
    rev = tuple(reversed(offsets_in))
    cell_cy_in = -hive_separation_in / 2.0  # RED cluster center; blue is its mirror
    cells = [
        # (mirror_x, mirror_y, cx_in, cy_in, member ids, member offsets)
        # Row order along +LL-Y (= +Pedro-X), per the real field:
        # audience side ascends 34->37 / 38->41, scoring side descends 33->30 / 45->42.
        (False, False,  cell_x_in, cell_cy_in, (34, 35, 36, 37), offsets_in),  # RED AUDIENCE
        (False, True,   cell_x_in, cell_cy_in, (38, 39, 40, 41), rev),         # BLUE AUDIENCE
        (True,  False, -cell_x_in, cell_cy_in, (30, 31, 32, 33), rev),         # RED SCORING
        (True,  True,  -cell_x_in, cell_cy_in, (42, 43, 44, 45), offsets_in),  # BLUE SCORING
    ]

    fiducials = []
    for mirror_x, mirror_y, cx_in, cy_in, ids, cell_offsets in cells:
        # One pitch for both sides; the lean away from field center and the
        # mirrored tag basis come from mirror_x -- which SIDE of the X axis
        # the cell sits on, never the id group.
        basis = basis_for_facing(pitch_rad, mirror_x)
        for tag_id, off_in in zip(ids, cell_offsets):
            tx = cx_in * 0.0254
            member_y_in = cy_in + off_in
            if mirror_y:
                member_y_in = -member_y_in   # full point-mirror of the member position
            ty = member_y_in * 0.0254
            row = transform_row(basis, tx, ty, center_h_m)
            ok, det = check_orthonormal(row)
            if not ok:
                raise ValueError(f"tag {tag_id}: invalid rotation (det={det})")
            fiducials.append({
                "transform": row,
                "size": size_mm,
                "id": tag_id,
                "family": TAG_FAMILY,
                "unique": 1,
            })

    return {
        "fiducials": fiducials,
        "type": "ftc",
        "fieldlength": 2 * half_m,
        "fieldwidth": 2 * half_m,
    }


def ll_table(fmap):
    """Human-readable sanity table: the LL-frame numbers actually stored in
    the map (meters, center origin). Framework-neutral on purpose -- any team
    can read it regardless of their pathing library. (Multiply by 39.37 for
    tape-comparable inches.)"""
    print(f"{'id':>3} {'ll X':>8} {'ll Y':>8} {'ll Z':>8}  facing")
    for f in sorted(fmap["fiducials"], key=lambda f: f["id"]):
        t = f["transform"]
        facing = "scored-mirror" if t[0] < 0 else "audience"
        print(f"{f['id']:>3} {t[3]:8.4f} {t[7]:8.4f} {t[11]:8.4f}  {facing}")


def main():
    ap = argparse.ArgumentParser(
        description="Biobuzz fmap generator — edit the CONFIG block in the "
                    "script to tune; the only CLI knob is the output path")
    ap.add_argument("--out", default="field_spec.fmap", help="output .fmap path")
    args = ap.parse_args()

    fmap = build_fmap(TAG_SIZE_MM, TAG_CENTER_IN, FACE_PITCH_DEG, CELL_X_IN,
                      HIVE_SEPARATION_IN, INNER_TAG_OFFSET_IN, OUTER_TAG_OFFSET_IN)
    with open(args.out, "w") as fh:
        json.dump(fmap, fh, separators=(",", ":"))

    print(f"wrote {args.out} ({len(fmap['fiducials'])} tags)")
    print(f"tag size: {TAG_SIZE_MM} mm ({TAG_SIZE_MM/25.4:.2f} in)")
    print(f"spec chain: pivot {PIVOT_HEIGHT_IN} in; holes +{HOLES_FROM_LIP_IN} and tags "
          f"+{TAG_ABOVE_HOLES_IN} along the surface; lip {LIP_FROM_WALL_IN} from the wall")
    print(f"tag bottom edge: Z {TAG_BOTTOM_IN:.4f} in "
          f"({'taped field measurement' if TAPED_TAG_BOTTOM_IN is not None else 'derived from the spec chain'}), "
          f"X {CELL_X_BOTTOM_IN:.4f} in "
          f"({'taped field measurement' if TAPED_CELL_X_BOTTOM_IN is not None else 'derived from the spec chain'})")
    print(f"tag center height: {TAG_CENTER_IN:.4f} in "
          f"(TAG_BOTTOM_IN {TAG_BOTTOM_IN:.4f} + (size/2)*cos({FACE_PITCH_DEG:g} deg))")
    print(f"face pitch: {FACE_PITCH_DEG:g} deg from vertical (one angle, mirrored lean)")
    print(f"cell X: bottom edge {CELL_X_BOTTOM_IN:.4f} in -> center {CELL_X_IN:.4f} in "
          "(all 16 tags; bottom edge is (size/2)*sin(pitch) further out)")
    print(f"hive centerline separation: {HIVE_SEPARATION_IN} in "
          f"(each centerline {HIVE_SEPARATION_IN/2:.3f} in from field center)")
    print(f"member offsets from hive centerline: "
          f"({-OUTER_TAG_OFFSET_IN}, {-INNER_TAG_OFFSET_IN}, "
          f"{INNER_TAG_OFFSET_IN}, {OUTER_TAG_OFFSET_IN}) in "
          "(from inner/outer tags; reversed on blue-audience and red-scoring cells)")
    ll_table(fmap)


if __name__ == "__main__":
    main()
