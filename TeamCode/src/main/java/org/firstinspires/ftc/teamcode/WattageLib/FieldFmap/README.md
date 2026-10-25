# Biobuzz `.fmap` generator — how to use `fmap_generator.py`

Generates the 16-tag Limelight field map for the four Biobuzz hive cells
(AprilTag 36h11, IDs 30–45) from a handful of tape measurements. Every tag is
placed in full 6 DOF: position from the hive-centerline model, orientation from
the 30° rest-tilt model.

> [!CAUTION]
> The map treats MOVING hive-cell tags as static, mapped at the 30° REST pose
> (the bi-stable notch the cell holds until scored into). A cell knocked off
> rest moves its tags off the mapped pose and the Limelight solve degrades.

## Spec chain → config (top of the script)

The tag references derive from the drawings' spec chain, not a tape:

| Config | What it is (from the drawings) | Default |
|---|---|---|
| `TAG_SIZE_MM` | black-square edge of a tag | 82.55 (3.25") |
| `FACE_PITCH_DEG` | tilt of the hive faces from vertical | 30 (at rest) |
| `PIVOT_HEIGHT_IN` | pivot axis above the floor | 43.95 |
| `HOLES_FROM_LIP_IN` | cell's lower lip → reference holes, along the surface | 9.938 |
| `TAG_ABOVE_HOLES_IN` | tag centers above the holes, along the surface | 2.75 |
| `LIP_FROM_WALL_IN` | furthest-out bottom edge (the pivot lip) from the audience/scoring wall, horizontal | 53.5 |
| `HIVE_SEPARATION_IN` | red↔blue hive centerline distance (25.5 → ±12.75 about 0,0) | 25.5 |
| `INNER_TAG_OFFSET_IN` | inner tag centers from the hive centerline | 2.5 |
| `OUTER_TAG_OFFSET_IN` | outer tag centers from the hive centerline | 6.5 |

The chain closes on itself: it puts the tag bottom edge at 53.531", matching
the drawings' 53.5 to 1/32". Derived from it (do not tape these — re-deriving
is automatic):
`TAG_BOTTOM_IN = PIVOT_HEIGHT_IN + (HOLES + ABOVE − size/2)·cos(FACE_PITCH_DEG)`
(= 53.5308"), `CELL_X_BOTTOM_IN = (FIELD_HALF_IN − LIP_FROM_WALL_IN) −
(HOLES + ABOVE − size/2)·sin(FACE_PITCH_DEG)` (= 12.9685"),
`TAG_CENTER_IN = tag_center_in(TAG_BOTTOM_IN, FACE_PITCH_DEG)` (ladder rule,
= 54.9380"), and `CELL_X_IN = CELL_X_BOTTOM_IN − (size/2)·sin(FACE_PITCH_DEG)`
(= 12.1560"). **Tape day:** fill `TAPED_TAG_BOTTOM_IN` /
`TAPED_CELL_X_BOTTOM_IN` at the top of the config with your taped numbers —
the map then derives from YOUR tape and the chain is bypassed entirely; a
direct tape also absorbs the pivot's unmodeled drop center, because it
measures the final position. Leave them `None` to keep deriving from the
chain. `FIELD_HALF_IN` (72) fixes the
144×144" field; `TAG_FAMILY` is fixed.

## Field deployment checklist

1. Limelight web UI → Field Map → upload the `.fmap` → set as Active Field Map.
2. Fiducial ID filter: 30–45 (nothing else exists this season).
3. Verify a known parked pose solves to the expected botpose before trusting it.


## The model in one screen

Limelight/FTC frame: **center origin**

Each hive's **centerline** runs down the middle of its tag cluster, and the two
centerlines sit symmetric about field center 0,0, `HIVE_SEPARATION_IN` apart
(25.5 → each at ±12.75" along the row axis). The tags' X comes from the spec
chain: the pivot lip hangs `FIELD_HALF_IN − LIP_FROM_WALL_IN` (18.5") from the
**perpendicular centerline** (through 0,0, across the field), and the face
rises inward as it rises up, pulling the bottom edge to 12.9685" and the tag
centers to 12.1560" — all 16 tags share these.

| Cell | IDs | LL position | Lean | Row direction (+LL-Y) |
|---|---|---|---|---|
| RED AUDIENCE | 34–37 | (+CELL_X_IN, −sep/2) | away from center | ascending: 34 35 36 37 |
| BLUE AUDIENCE | 38–41 | (+CELL_X_IN, +sep/2) | away from center | ascending: 38 39 40 41 |
| RED SCORING | 30–33 | (−CELL_X_IN, −sep/2) | away from center | descending: 33 32 31 30 |
| BLUE SCORING | 42–45 | (−CELL_X_IN, +sep/2) | away from center | descending: 45 44 43 42 |

Member offsets are **measured from the hive centerline**: inner tags
`INNER_TAG_OFFSET_IN` (2.5"), outer tags `OUTER_TAG_OFFSET_IN` (6.5") →
(−6.5, −2.5, +2.5, +6.5), gaps 4.0 / 5.0 / 4.0.

**Heights are tilt-corrected.** The map stores the tag CENTER. On a face
pitched p from vertical, the center is `size/2` up the *face* from the bottom
edge but only `(size/2)·cos(p)` of that is vertical rise, so
`TAG_CENTER_IN = TAG_BOTTOM_IN + (size/2)·cos(FACE_PITCH_DEG)` = 54.9380" 

**One pitch for both sides.** Lean angle is a single number
(`FACE_PITCH_DEG`, 30° = rest); the sides differ only in lean *direction*
(always away from field center)

## Field Assumptions (correct for most use cases)

- **Static at rest.** Moving cells are mapped at the 30° rest pose only.
- **One orientation for all 16 tags.** Tags share the same pitch/zero in-face yaw/zero roll;
- **One shared height.** All 16 centers share `TAG_CENTER_IN`; the real field
  mounting heights are assumed identical (verify with the tape on your field).
- **One shared X.** `CELL_X_IN` is mirrored between audience and scoring sides;
- **Symmetric clusters.** Tags(of the clusters) offsets are always ±inner/±outer about the centerline.
- **One tag size and family** for every entry; the field is hardcoded at 144×144" (`FIELD_HALF_IN`).


## Usage

```powershell
python fmap_generator.py                    # -> field_spec.fmap + console summary + table
python fmap_generator.py --out my_map.fmap  # different destination
```

The script's interface is the CONFIG block at the top of `fmap_generator.py` —
every physical knob lives there, and derived values re-derive automatically
(change `TAG_BOTTOM_IN` or `FACE_PITCH_DEG` and `TAG_CENTER_IN` follows;
change the inner/outer offsets and the member offsets follow). There are no
other CLI flags on purpose: the standardized placement needs no knobs, and
teams on non-standard fields edit the config instead. Tape day is two lines:
fill `TAPED_TAG_BOTTOM_IN` and `TAPED_CELL_X_BOTTOM_IN` (vertical and flat
floor tapes to the black squares' bottom edge); everything downstream
re-derives. The console summary states which values were used and ends with
an LL-frame table (meters) — exactly the numbers stored in the map. Multiply
by 39.37 for tape-comparable inches.




