#!/usr/bin/env python3
"""
Biobuzz .fmap generator — builds a 16-tag Limelight field map for 4 mirrored
hive cells from a handful of physical measurements.

Frame conventions (validated on-field 2026-09-30):
  * The .fmap lives in the Limelight/FTC field frame: center origin, +X = Pedro
    south, +Y = Pedro east, +Z up, units meters. (Pedro's frame is corner-origin
    0-144 in, heading 0 = +X; the mapping is px = ll_y + 72, py = -ll_x + 72.)
  * Faces rest tipped FACE_PITCH_DEG from vertical (30 = rest): the normal is
    (cos p, 0, sin p) in the LL frame for the audience-facing cells.
  * The four cells are axis-mirrors of one cell's position (CELL_XY_IN), and
    mirrored cells' member positions are FULL point-mirrors of the original
    cell's member positions (so member offsets reverse in LL-Y on the blue
    cells):
      34-37 RED AUDIENCE   at (+cx, +cy)   facing audience
      38-41 BLUE AUDIENCE  at (+cx, -cy)   facing audience  (mirror across LL-Y)
      30-33 RED SCORING    at (-cx, +cy)   facing mirrored  (mirror across LL-X)
      42-45 BLUE SCORING   at (-cx, -cy)   facing mirrored  (both mirrors)
    Mirroring the facing flips the tag basis; c1 flips sign to keep det = +1.
  * Member tags sit in a horizontal row on the tilted face: positions differ
    only in LL-Y by MEMBER_OFFSETS_IN (the SDK cluster offsets: -6.5, -2.75,
    +2.75, +6.5 -> gaps 3.75 / 5.5 / 3.75 in). Same z for all four (the row is
    horizontal; the tilt is about the horizontal axis).
  * TAG_BOTTOM_IN is the height of the black square's BOTTOM edge; the map
    stores the CENTER: center = bottom + size/2.
"""

import argparse
import json
import math

# ------------------------- CONFIG (defaults) -------------------------
TAG_SIZE_MM = 82.55            # black-square edge, mm (3.25 in)
TAG_BOTTOM_IN = 49.5           # bottom edge of tag above floor, inches
FACE_PITCH_DEG = 30.0          # face tilt from vertical, degrees (30 = rest)
CELL_XY_IN = (11.92, -6.84)    # LL-frame face-center offset from field center,
                               # inches (X = Pedro-south, Y = Pedro-east)
MEMBER_OFFSETS_IN = (-6.5, -2.75, 2.75, 6.5)   # SDK cluster offsets, inches
FIELD_HALF_IN = 72.0           # field half-size (144 x 144 in field)
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


def build_fmap(size_mm=TAG_SIZE_MM, bottom_in=TAG_BOTTOM_IN, pitch_deg=FACE_PITCH_DEG,
               cell_xy_in=CELL_XY_IN, offsets_in=MEMBER_OFFSETS_IN):
    size_in = size_mm / 25.4
    center_h_m = (bottom_in + size_in / 2.0) * 0.0254
    pitch_rad = math.radians(pitch_deg)
    half_m = FIELD_HALF_IN * 0.0254

    cells = [
        # (mirror-x?, mirror-y?, cx_in, cy_in, member ids)
        (False, False,  cell_xy_in[0],  cell_xy_in[1], (34, 35, 36, 37)),  # RED AUDIENCE
        (False, True,   cell_xy_in[0],  cell_xy_in[1], (38, 39, 40, 41)),  # BLUE AUDIENCE
        (True,  False, -cell_xy_in[0],  cell_xy_in[1], (30, 31, 32, 33)),  # RED SCORING
        (True,  True,  -cell_xy_in[0],  cell_xy_in[1], (42, 43, 44, 45)),  # BLUE SCORING
    ]

    fiducials = []
    for mirror_x, mirror_y, cx_in, cy_in, ids in cells:
        basis = basis_for_facing(pitch_rad, mirror_x)
        for tag_id, off_in in zip(ids, offsets_in):
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


def pedro_table(fmap):
    """Human-readable sanity table: Pedro-frame positions (inches) per tag."""
    print(f"{'id':>3} {'pedroX':>8} {'pedroY':>8} {'centerH':>8}  facing")
    for f in sorted(fmap["fiducials"], key=lambda f: f["id"]):
        t = f["transform"]
        px = t[7] * 39.3701 + FIELD_HALF_IN   # Pedro X = ll_y + 72
        py = -t[3] * 39.3701 + FIELD_HALF_IN  # Pedro Y = -ll_x + 72
        h = t[11] * 39.3701
        mirrored = t[0] < 0
        facing = "scored-mirror" if mirrored else "audience"
        print(f"{f['id']:>3} {px:8.2f} {py:8.2f} {h:8.2f}  {facing}")


def main():
    ap = argparse.ArgumentParser(description="Biobuzz fmap generator")
    ap.add_argument("--out", default="field_spec.fmap", help="output .fmap path")
    ap.add_argument("--size-mm", type=float, default=TAG_SIZE_MM)
    ap.add_argument("--bottom-in", type=float, default=TAG_BOTTOM_IN,
                    help="height of the tag's BOTTOM edge, inches")
    ap.add_argument("--pitch-deg", type=float, default=FACE_PITCH_DEG,
                    help="face tilt from vertical, degrees (30 = rest)")
    ap.add_argument("--cell-xy-in", nargs=2, type=float, default=list(CELL_XY_IN),
                    metavar=("X", "Y"),
                    help="LL-frame cell-center offset from field center, inches")
    ap.add_argument("--offsets-in", nargs=4, type=float, default=list(MEMBER_OFFSETS_IN),
                    metavar=("O0", "O1", "O2", "O3"),
                    help="member tag offsets along the face horizontal, inches")
    args = ap.parse_args()

    fmap = build_fmap(args.size_mm, args.bottom_in, args.pitch_deg,
                      tuple(args.cell_xy_in), tuple(args.offsets_in))
    with open(args.out, "w") as fh:
        json.dump(fmap, fh, separators=(",", ":"))

    print(f"wrote {args.out} ({len(fmap['fiducials'])} tags)")
    print(f"tag size: {args.size_mm} mm ({args.size_mm/25.4:.2f} in)")
    print(f"tag center height: {(args.bottom_in + args.size_mm/25.4/2):.3f} in "
          f"= {(args.bottom_in + args.size_mm/25.4/2)*0.0254:.5f} m "
          f"(bottom at {args.bottom_in} in)")
    print(f"face pitch: {args.pitch_deg} deg from vertical")
    pedro_table(fmap)


if __name__ == "__main__":
    main()
