# `ftc2026BiobuzzTest.fmap` — cluster tags mapped at 30° REST, off-rest solves filtered in software

> [!CAUTION]
> This map deliberately treats MOVING hive-cell tags as static, against FIRST's guidance.
> Every entry is the tag's pose with its cell at the 30° REST position (the bi-stable
> notch the hive holds until scored into). A cell knocked off rest moves its tags off
> the mapped pose and the Limelight solve degrades — expected, handled downstream
> (`TipSanityFilter` drops it, `TipCompensator` will rescue it once tested). If the robot
> localizes badly right after a cell tips nearby, suspect this map first.

## Sources (all measured, no guesses)

- Tag size 3.25" (82.55 mm) + in-face offsets (tag centers −6.5 / −2.75 / +2.75 / +6.5 in
  about the face centerline; tag center 2.75" above the sticker bottom edge) — your
  Figure 9-15 photo (AprilTag Cluster and Reference Hole layout).
- Bottom edge of sticker 9.938" (25.25 cm) above the FRONT of the cell — same figure,
  lower view. Combined tag height ≈ cell-bottom height + 9.938 + 2.75.
- Pivot axis 43.95" above TILES; hive center-to-center 25.5"; cells ~18.8" apart per
  hive — your Figure 9-10 photo + manual §9.6.1 text (pp. 69–71 of the Section 9 PDF).
- Rest tilt 30° — your Figure 9-10 lower-left view + this conversation. The tag face
  normal at rest is pitched 30° off straight-down, toward that cell's scoring side.
- Cluster-to-field placement: each cluster sits on the BOTTOM of its cell, bottom edge
  toward FIELD CENTER; red cells one hive (IDs 30–37), blue the other (38–45); SDK 12.0
  names 30 RED SCORING / 34 RED AUDIENCE / 38 BLUE AUDIENCE / 42 BLUE SCORING.
- Field 144×144 in (3.6576 m); `fieldlength/fieldwidth` are correct meters
  (the original file had FRC 2024 values).

## How the 30° rest is encoded

Per-tag rotation = face 30° off straight-down: R = Rx(180°∓30°) composed with the
facing yaw (±90° about Z per hive side), i.e. each row of the 4×4 tilts the tag normal
30° from −Z toward the cell's rest direction. The two cells of one hive rest toward
OPPOSITE sides (bi-stable pivot — one up, one down), so the red blocks differ: IDs
30–33 encode −30° (Rx(150°)), IDs 34–37 encode +30° (Rx(210°)); same split for blue
(38–41 / 42–45). Z translations differ per block accordingly (rest-high cells sit
higher: 1.18044 m vs rest-low 0.78826 m at the tag centroid).

## Frame assumption you MUST verify on-field (Fusion Tune)

Limelight FTC origin = field CORNER (meters, +X along one wall, +Y along the other,
Z up). Entries assume corner (0,0) = Red-side/Audience-side corner with +X along the
hive row. If `Fusion/visionX/Y` mirrors Pinpoint truth, the map corner is wrong — fix
the matrix translations here, not the filter. Per-tag X/Y step along the row by the
face offsets above; the REST height came from pivot 43.95" minus the drop to the
rest-tilted face (see estimates below).

## Sanity-filter tuning (the 30° part)

A GOOD solve reports the rest signature — botpose pitch ≈ ±30°, roll ≈ 0 — NOT ~0.
Park level in front of a REST cell, read `Tip/pitchDeg` + `Tip/rollDeg`, and set
`TipSanityFilter.LEVEL_PITCH/ROLL_DEG` to exactly what you see (defaults 30.0 / 0.0).
`TILT_TOL_DEG` (12°) is the deviation allowed: a REST solve fuses at full weight, a
knocked cell exceeds it and gets dropped (Stage 1) or rescued (Stage 2, disabled).

## What is still estimated (measure before competition)

1. **Tag height (Z per block)**: cell-bottom height at 30° rest is not dimensioned in
   the pages I could read (Fig 9-10 gives opening heights 53.5/65.6", not the cell
   bottom). Estimated from pivot 43.95" minus drop to the tilted face. Put a tape on
   a REST cell: `Z_m = (cellBottomIn + 9.938 + 2.75·cos30° …) × 0.0254`, update Z.
2. **Row offsets (±X per tag, ±Y per block)**: hive center-to-center 25.5" + face
   offsets to the cluster centroid. Replace with a tape from the field wall if off.
3. **SCORING vs AUDIENCE ends** (30–33 vs 34–37): SDK names don't say which physical
   end they sit on. If backwards, swap the two red blocks (and the two blue) wholesale.
4. **Rest direction per cell** (which side reads −30° vs +30°): if `Tip/pitchDeg` at a
   known REST cell reads the mirror (e.g. −30 where +30 expected), swap that block's
   rotation rows with its hive-mate's.

## Upload + pipeline checklist

1. Limelight web UI → Field Map → upload this file → select as Active Field Map.
2. Fiducial ID filter: 30–45 (all cluster tags). Nothing else exists this season.
3. `Vision.poll()` seeds gyro yaw every loop (required for MT2).
4. `TipSanityFilter` (ON): drops off-rest/underground/teleport solves — `Tip/*` telemetry.
5. `TipCompensator` (OFF): enable only after its validation plan passes.
6. `HiveCellMonitor` still handles relative drive-to-cell aiming off the same tags.
