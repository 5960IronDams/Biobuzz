# VisionFusion Tuning Guide

How to tune the Limelight ↔ Pedro fusion, and how to read the `Fusion/*` +
`Tip/*` telemetry while doing it. Everything below is grounded in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Vision/VisionFusion.java`
(all knobs are Panels-live under **Panels → VisionFusion** unless noted).

---

## 1. What the pipeline actually does

Per loop (see `RobotMain.RobotRunPeriodic()`):

```
follower.update()        →  PREDICT   (Pinpoint dead-reckoning + fusion owner)
visionFusion.correct()   →  CORRECT   (0 or 1 Limelight frame → addMeasurement)
```

* TeleOp calls `correct()` **twice** per loop (once in `RobotRunPeriodic`,
  once in the trailing drive update). The second call re-polls the same
  ~10 Hz Limelight packet and is a same-frame **no-op** (bit-identical dedup,
  10 ms window) — each camera frame fuses exactly once.
* `correct()` never calls `setPose()`. It blends via Kalman gain
  `K = P/(P+R)` with a **back-dated timestamp** (exposure midpoint), so
  corrections interpolate through Pedro's history instead of teleporting it.
* `report()` runs once per loop from `flushTelemetry()` — the single source
  of every `Fusion/*` and `Tip/*` line.

### The split-solver rule (do not fight it)

| Axis | Source | Why |
|---|---|---|
| XY | **MT2** botpose (seeded with fused heading) | MT2's yaw is a seed *echo* — fusing it double-counts the gyro and warps heading |
| Heading | **MT1** yaw, gated hard (see §4) | independent solve, safe to fuse, but flip-ambiguous on single tags |

MT2 yaw is **never** fused (NaN-masked). `FusionLocalizer` skips NaN axes, so
each fuse call masks the axis it doesn't own.

### Three layers of defense

1. **Absolute rejects** — garbage only: no solve, NaN, stale (>350 ms),
   off-field, tip/tilt sanity, seed-mirror (>120°), diverged state.
   Rejected frames fuse *nothing*.
2. **Soft R de-weighting** — almost everything else still fuses, but its
   measurement noise `R` is multiplied up (trust down) by conditions:
   motion, range, tag size/baseline, residuals, seed disagreement.
   This is why corrections survive "tag deserts" instead of going blind.
3. **Axis gates** — MT1 yaw has its own hard gates (§4); XY always fuses
   unless `FUSE_XY=false`.

---

## 2. First calibration: the Limelight yaw offset (do this before ANY tuning)

Everything MT2-related is **seeded** with the fused heading mapped into the
Limelight frame. If the offset is wrong, MT2's XY is conditioned on a bad
seed and `Fusion/seedAgrDeg` goes high.

1. Put the robot on a known field heading `H` (Pedro, degrees), stationary.
2. Read `Fusion/mt1LlRawDeg` (raw MT1 yaw straight off the LL, no mapping).
3. `LL_YAW_OFFSET_DEG` (in `Vision.java`) = `norm180(H − mt1LlRawDeg)`.
4. Echo check: `Fusion/mt2LlRawDeg` should track `Fusion/seedSentDeg`
   (MT2 yaw is a seed echo). `Fusion/seedAgrDeg` should sit **< ~10°**.
   * Pinned near **180°** = the seed frame is mirrored → fix
     `LL_YAW_OFFSET_DEG` / `LL_YAW_SIGN`, don't tune around it.

The sign/offset pipeline (for reference, lives in the MT1-yaw comment block
in `VisionFusion.java`):

```
Measurement:  ll_yaw · LL_YAW_SIGN + LL_YAW_OFFSET_DEG  → field_yaw  →  +90° → pedro_yaw
Seeding:      fused(pedro) → −90° → field_yaw → (field − offset) · sign → ll_yaw
```

---

## 3. Baseline trust — set these two first, everything else scales them

These are **variances**, not stddevs (they feed `R` directly):

| Knob | Meaning | Default |
|---|---|---|
| `BASE_R_XY` | XY measurement variance, **in²**. 9.0 = 3 in stddev | 9.0 |
| `BASE_R_HEADING` | MT1 yaw variance, **rad²**. 0.10 ≈ 18° stddev | 0.10 |

* Robot drifts from vision between LL frames (laggy/wobbly XY)?
  → **lower** `BASE_R_XY` (trust vision more).
* Vision solve is jittery and pose twitches?
  → **raise** `BASE_R_XY` (trust Pinpoint more).
* Heading never gets drift-corrected (compare `Fusion/fusedHdeg` vs
  `pinpointHdeg` over minutes)?
  → **lower** `BASE_R_HEADING` toward 0.03 (10° std). If heading ever
  *warps/snaps*, raise it back toward 0.27+.

`Fusion/scale` shows the total R multiplier applied this loop
(1.0× = full trust, 64× = `MAX_TOTAL_SCALE` clamp).

---

## 4. The MT1-yaw gates (why `Fusion/yaw` says what it says)

`Fusion/yaw` = `fused` or the skip reason. MT1 yaw fuses only when ALL of:

| Gate | Knob | Default | Skip reason shown |
|---|---|---|---|
| Multi-tag | `YAW_MIN_TAGS` | 2 | `tags` |
| Slow | `YAW_MAX_SPEED_IPS` | 20 in/s | `speed` |
| Not turning | `YAW_MAX_OMEGA_RPS` | 0.5 rad/s | `turn` |
| Close | `YAW_MAX_DIST_M` | 2.5 m | `dist` |
| Small innovation | (hardcoded 90°) | — | `innov` |
| Feature on | `MT1_YAW_FUSING` | true | `yaw-off` |

Single-tag yaw is flip-ambiguous on down-facing clusters — these gates are
the armor. `Fusion/yawInnov` shows the signed innovation (flips show as ±180).
Normal driving shows `speed`/`turn` skips constantly; that's by design —
yaw drift correction happens while you pause/creep.

---

## 5. Soft de-weighting knobs (when to touch each)

All multiply XY's `R`. Defaults are sane; touch ONE at a time.

| Symptom | Knob | Direction |
|---|---|---|
| Vision lags while driving fast | `SPEED_REF_IPS` (40) | raise |
| Vision dies while spinning | `TURN_REF_RPS` (2.0) | raise |
| Long-range shots lose vision | `DIST_SLOPE_PER_M` (0.6) / `CLOSE_DIST_M` (1.5) | lower slope / raise close range |
| Far/tiny tags rejected-ish | `AREA_REF_PCT` (0.5) | raise (less penalty) |
| Two tags close together → jumpy XY | `SPAN_REF_M` (1.0) | raise (more penalty) |
| Only 1 tag visible → too jumpy | `SINGLE_TAG_PENALTY` (3.0) | raise |
| No MT2, MT1-position fallback noisy | `MT1_XY_PENALTY` (4.0) | raise |
| MT2 XY collapses when seed sketchy | `YAW_DISAGREE_PENALTY` (4.0), ramps between `YAW_AGR_FULL_DEG` (15°) and `YAW_AGR_MAX_DEG` (60°) | raise |
| LL self-reported stddev trusted too much | `STDDEV_TRUST_CAP_IN` (6.0) | lower |
| Big jumps still nudge the pose | `RESIDUAL_FULL_IN` (6) / `RESIDUAL_MAX_IN` (24) / `RESIDUAL_MAX_PENALTY` (9) | lower FULL / raise PENALTY |
| Any single reading too influential | `MAX_TOTAL_SCALE` (64) | lower |

The residual ramp is **soft and unbounded-by-design**: a 36 in jump fuses at
~23× R (heavily ignored, still fusing); only >288 in residual is hard-rejected
(`reject-residual-huge`), and stationary (<10 in/s) + >36 in jump rejects too
(`reject-residual-stationary`) — those two are hardcoded backstops against
filter divergence, not tunables.

---

## 6. Reading `Fusion/status`

| Status | Meaning | Action |
|---|---|---|
| `fused-xy` | XY fused (yaw gated) | healthy |
| `fused-xy-yaw` | XY + MT1 yaw fused | healthy |
| `fused-xy-bigjump` | fused, but residual > `MAX_JUMP_XY_IN` (36) | watch: usually a lock hop after blindness |
| `no-solve` | LL sees nothing | check exposure/streams |
| `reject-stale` | frame > 350 ms old | USB/network lag; raise `MAX_STALENESS_MS` only if chronic |
| `reject-tilt` / Tip `REJECT_*` | tipped/off-rest cluster solve dropped | Stage 1 doing its job; see `Tip/verdict` |
| `reject-seed` | seed echoed back wrong (>120°) | yaw-offset/mirror problem — recalibrate §2 |
| `reject-residual-stationary` | stationary + huge jump | filter state diverged; let Pinpoint recover |
| `skip-fused-diverged` | fused pose outside 2× field | Pinpoint problem, vision stood down |
| `fuse-off` / `disabled` | kill switches (`FUSE_XY` / `ENABLED`) | intentional |
| `no-limelight-configured` | no LL in RC config | Pinpoint-only by design |

`Fusion/pinpointX/Y/Hdeg` (raw Pinpoint) vs `Fusion/fusedX/Y/Hdeg` is the
fastest "is vision helping or hurting" comparison — log both on the field
map and watch which one tracks reality.

---

## 7. Recommended tuning order (one change per test)

1. **§2 yaw-offset calibration** (stationary, 4 headings). Everything depends on it.
2. **Stationary test**: `fused-xy`, `residualIn` < ~3 in, `scale` ~1×, `seedAgrDeg` < 10°.
3. **Heading sweep** (spin in place): residual must stay FLAT (if it rises with
   heading, the camera lever-arm is double-compensated — see the note at
   `USE_MT1_XY` in the code). Expect `yaw=skip(turn)`.
4. **Slow drive**: fused pose should track the field layout; vision corrections
   visible as small `Fusion/dxy` nudges.
5. **Fast drive / spins**: no heading snaps, no NaNs; `yaw=skip(speed)`.
6. Only now touch §3/§5 knobs — trust baselines first, de-weighting second,
   reject gates **last** (they throw away data).

---

## 8. Deliberately disabled things (don't enable casually)

* `TipCompensator.ENABLED` (Stage 2 rescue of tipped-cluster solves) — ships
  **false** until its validation plan is done on-field.
* `USE_MT1_XY` — flips the XY source to MT1's independent solve; only after
  comparing `Fusion/mt1X/Y` vs `visionX/Y` on-field (the 02-47 depth-collapse
  investigation lives in the code comments).
* `REJECT_CLUSTER_ONLY` — conservative Pinpoint-only-when-clusters-only; you
  localize off the clusters, so it ships false.
* The inline `BOT_Z_MIN/MAX_M` height check in `correct()` is commented out —
  `TipSanityFilter` (Stage 1) owns height sanity now.
