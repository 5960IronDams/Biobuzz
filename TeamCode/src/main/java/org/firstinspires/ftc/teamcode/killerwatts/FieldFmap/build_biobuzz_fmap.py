#!/usr/bin/env python3
"""Build ftc2026BiobuzzTest2.fmap from adjustable geometry — no hand-edited matrices.

Usage (repo root):
    py TeamCode/src/main/java/org/firstinspires/ftc/teamcode/killerwatts/build_biobuzz_fmap.py

Reads the current .fmap for cluster centroids, rebuilds every entry with exact
trig (uniform tilt, nominal spacing, orthonormal rotations), verifies, and
writes back. Re-run after every tape-measure session; the .fmap is a build
artifact of THIS file. Edit CONFIG below, not the JSON.

Conventions (must match the existing working file — MT1 solves solid on it):
  * Limelight .fmap transform is row-major 4x4; rotation columns are the tag
  * frame axes X (right, increasing ID), Y (up), Z (normal) in field meters.
  * Tag normals point UP-ish (+Z): elevation = tilt_from_horizontal_deg above
  * the horizontal plane. 30deg-from-horizontal == 60deg-from-vertical: tags
  * face away from the hive centerline, tipped 30deg up from horizontal.
  * All clusters mapped in the UP-hive rest pose: uniform tilt, ~48in height.
  * Stickers lie FLAT on the cell face (brick course): the 4-tag row runs along
  * the cell's long edge, tag "up" (Y) across it. IN_PLANE_ROLL spins the print
  * within the face — 90deg swaps the row from vertical-stack to brick-lay
  * without touching normals.
"""

import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ================= EDIT ME =================
CONFIG = {
    # File to read centroids from AND write back to (backup saved as .bak).
    "filename": "ftc2026BiobuzzTest2.fmap",

    # Normal elevation above horizontal, degrees.
    # 30 == 60deg-from-vertical: tags face away from the hive centerline,
    # tipped 30deg up from horizontal. Single value: every cluster is mapped in
    # its UP-hive rest pose. Per-cluster override, e.g. {34: 60.0}, if a hive
    # ever needs the bi-stable split back (that was stale data — see .bak).
    "tilt_from_horizontal_deg": 30.0,
    "tilt_override": {},

    # Horizontal direction each cluster's normal points ("away from centerline").
    # Flip a value between "+X"/"-X"/"+Y"/"-Y" while debugging facing.
    "facing": {30: "+Y", 34: "-Y", 38: "-Y", 42: "+Y"},

    # Field direction of INCREASING tag ID along the sticker row.
    # Must be perpendicular to that cluster's facing (script warns otherwise).
    "row_dir": {30: "+X", 34: "+X", 38: "+X", 42: "+X"},

    # In-plane print rotation, degrees. 0 = tag X (increasing ID) along row_dir.
    # 90 = print spun within the face (brick-lay vs card-stack): rotates the
    # tag X/Y axes about the normal, normals untouched. Flip while debugging
    # print orientation; per-cluster override e.g. {34: 0} if a hive differs.
    "in_plane_roll_deg": 90.0,
    "in_plane_override": {},

    # Tag centers along the row, inches (Fig 9-15: -6.5/-2.75/+2.75/+6.5).
    "offsets_in": [-6.5, -2.75, 2.75, 6.5],

    # Height handling. None = keep each cluster's current mean Z (no jump).
    # Set to 48 * 0.0254 (= 1.2192) to force the taped ~48in height on all.
    "tag_height_m": None,
    "per_cluster_z": {},  # e.g. {34: 1.16}

    # XY handling. True = keep each cluster's current centroid XY, only snap
    # the 4 tags collinear with nominal spacing. False = use centroids below.
    "keep_xy_centroids": True,
    "centroids": {  # meters; only used when keep_xy_centroids is False
        30: (-0.3256, 0.4158),
        34: (-0.3239, -0.2634),
        38: (0.3239, -0.2634),
        42: (0.3239, 0.4158),
    },

    "tag_size_mm": 82.55,
    "family": "apriltag3_36h11_classic",
}
# =============== END EDIT ME ===============

IN2M = 0.0254
DIRS = {"+X": (1.0, 0.0), "-X": (-1.0, 0.0), "+Y": (0.0, 1.0), "-Y": (0.0, -1.0)}


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm(v):
    l = math.sqrt(dot(v, v))
    assert l > 1e-9, v
    return (v[0] / l, v[1] / l, v[2] / l)


def main():
    path = os.path.join(HERE, CONFIG["filename"])
    d = json.load(open(path))
    fids = sorted(d["fiducials"], key=lambda f: f["id"])
    assert [f["id"] for f in fids] == list(range(30, 46)), "expected IDs 30-45"

    offs = [o * IN2M for o in CONFIG["offsets_in"]]
    assert len(offs) == 4

    out = []
    for base in (30, 34, 38, 42):
        cl = [f for f in fids if base <= f["id"] < base + 4]
        el = math.radians(CONFIG["tilt_override"].get(base,
                                                      CONFIG["tilt_from_horizontal_deg"]))
        fx, fy = DIRS[CONFIG["facing"][base]]
        rx, ry = DIRS[CONFIG["row_dir"][base]]
        if abs(fx * rx + fy * ry) > 1e-6:
            print(f"WARNING base{base}: row_dir not perpendicular to facing "
                  f"({CONFIG['row_dir'][base]} vs {CONFIG['facing'][base]}) — "
                  f"orthogonalizing, check sticker orientation!")
        # Normal: horizontal facing tilted up by el.
        n = (fx * math.cos(el), fy * math.cos(el), math.sin(el))
        # In-plane X along the row, orthogonalized; then spin the print about
        # the normal by the in-plane roll (brick-lay vs card-stack).
        x0 = norm((rx - dot((rx, ry, 0.0), n) * n[0],
                   ry - dot((rx, ry, 0.0), n) * n[1],
                   0.0 - dot((rx, ry, 0.0), n) * n[2]))
        y0 = cross(n, x0)
        roll = math.radians(CONFIG["in_plane_override"].get(
            base, CONFIG["in_plane_roll_deg"]))
        x = (x0[0] * math.cos(roll) + y0[0] * math.sin(roll),
             x0[1] * math.cos(roll) + y0[1] * math.sin(roll),
             x0[2] * math.cos(roll) + y0[2] * math.sin(roll))
        y = cross(n, x)
        if CONFIG["keep_xy_centroids"]:
            cx = sum(f["transform"][3] for f in cl) / 4
            cy = sum(f["transform"][7] for f in cl) / 4
        else:
            cx, cy = CONFIG["centroids"][base]
        cz = CONFIG["per_cluster_z"].get(base,
                                        CONFIG["tag_height_m"]
                                        if CONFIG["tag_height_m"] is not None
                                        else sum(f["transform"][11] for f in cl) / 4)
        for k, f in enumerate(sorted(cl, key=lambda f: f["id"])):
            tx, ty, tz = cx + offs[k] * x[0], cy + offs[k] * x[1], cz
            # NOTE: row stays horizontal (tilt axis == row), so all 4 share tz.
            t = [x[0], y[0], n[0], tx,
                 x[1], y[1], n[1], ty,
                 x[2], y[2], n[2], tz,
                 0, 0, 0, 1]
            out.append({"transform": t, "size": CONFIG["tag_size_mm"],
                        "id": f["id"], "family": CONFIG["family"], "unique": 1})

    out = sorted(out, key=lambda f: f["id"])

    # ---- verify ----
    print("=== rebuilt ===")
    worst_el = 0.0
    max_shift = 0.0
    for base in (30, 34, 38, 42):
        cl = [f for f in out if base <= f["id"] < base + 4]
        want = CONFIG["tilt_override"].get(base, CONFIG["tilt_from_horizontal_deg"])
        xs = [f["transform"][3] for f in cl]
        ys = [f["transform"][7] for f in cl]
        gaps = [(xs[k + 1] - xs[k]) * 1000 if abs(xs[3] - xs[0]) >= abs(ys[3] - ys[0])
                else (ys[k + 1] - ys[k]) * 1000 for k in range(3)]
        els = [round(math.degrees(math.asin(max(-1, min(1, f["transform"][10])))), 3)
               for f in cl]
        worst_el = max(worst_el, max(abs(e - want) for e in els))
        print(f"base{base}: target={want} gaps={[round(g, 2) for g in gaps]} "
              f"els={els} z={[round(f['transform'][11], 5) for f in cl]}")
    for nf, of in zip(out, fids):
        t, o = nf["transform"], of["transform"]
        X, Y, N = (t[0], t[4], t[8]), (t[1], t[5], t[9]), (t[2], t[6], t[10])
        assert abs(dot(X, Y)) < 1e-9, (nf["id"], dot(X, Y))
        assert abs(dot(X, N)) < 1e-9 and abs(dot(Y, N)) < 1e-9
        assert abs(math.sqrt(dot(X, X)) - 1) < 1e-9
        assert abs(dot(X, cross(Y, N)) - 1) < 1e-9, nf["id"]
        max_shift = max(max_shift, math.hypot(t[3] - o[3], t[7] - o[7],
                                              t[11] - o[11]) * 1000)
    print(f"max |elev-target| = {worst_el:.4f} deg, "
          f"max center shift vs input = {max_shift:.1f} mm")

    os.replace(path, path + ".bak")
    json.dump({"fiducials": out, "type": d["type"],
               "fieldlength": d["fieldlength"], "fieldwidth": d["fieldwidth"]},
              open(path, "w"), separators=(",", ":"))
    print(f"wrote {path} (backup at {path}.bak)")


if __name__ == "__main__":
    main()
