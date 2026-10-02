# Bug report draft — ready to paste into the Pedro Pathing GitHub

> [!TIP]
> Copy everything below the horizontal rule into a new GitHub issue (Markdown is already GitHub-flavored).
> Fill the two `[TODO]` placeholders if you want (your team name is optional).

---

## Title

**`FusionLocalizer.addMeasurement` injects a one-time ~138–206° heading offset, even for XY-only measurements with NaN (masked) yaw — revhub/core 3.0.1**

## Environment

- Pedro Pathing: `com.pedropathing:revhub:3.0.1` (pulls `com.pedropathing:core:3.0.1`)
- FTC SDK: current 2026-season SDK, Robot Controller = Control Hub
- Dead reckoning: `com.pedropathing.revhub.localizers.PinpointLocalizer` (goBILDA Pinpoint odometry computer) wrapped in `com.pedropathing.localization.FusionLocalizer` (built by Pedro's bundled `Constants.create(...)`)
- Units: inches / radians (Pedro frame: CCW+, 0 = +X)
- Our loop: `follower.update()` every ~25 ms; `addMeasurement(...)` right after, ~10 Hz (Limelight), with the stamp back-dated by the solve's self-reported capture latency (30–100 ms, clamped ≤ 250 ms)

## Summary

Calling `FusionLocalizer.addMeasurement(Pose, long stamp, Pose R)` — **even when the measurement's heading is `Double.NaN`** (so only X/Y should fuse) — injects a **one-time spurious heading change of roughly +138° to +206°** into the filter state. The injection happens at the moment a measurement is applied; after it, the predict machinery behaves perfectly (the fused heading tracks the dead-reckoning heading exactly, but with the constant offset). Four events documented across four independent on-robot sessions. With `addMeasurement` never called, the filter is flawless through aggressive rotations.

We verified in decompiled bytecode that the per-axis NaN mask (`hasX/hasY/hasH`) zeroes both the innovation and the masked axis's gain rows, so an XY-only fuse should be mathematically unable to move the heading. It does anyway — which localizes the corruption to the parts of `addMeasurement` we could not fully audit from bytecode: the corrected-state insertion at an interpolated (back-dated) stamp and/or the forward history re-propagation walk.

## Reproduction (on-robot)

1. Build `FusionLocalizer(PinpointLocalizer, ...)` via `Constants.create(...)`; seed the pose (`setPose`) so the filter starts at a known pose.
2. Robot **stationary**. Log both `follower.pose()` (fused) and the raw dead-reckoning pose every loop.
3. After the filter settles (heading ≈ 90°), apply one measurement:
   - `new Pose(67.0, 26.4, Double.NaN)` (inches, Pedro frame — a ~17" XY innovation from the true pose)
   - `stamp = System.nanoTime() − ~100 ms` (back-dated, as a real vision latency would be)
   - `R = new Pose(9, 9, 0.27)` (in², in², rad²)
4. Observe: fused heading jumps ~90° → ~256.7° (+166.7°) while the raw dead-reckoning heading stays 90.0 exactly.

We reproduced this four times (see Evidence). It also triggers with a real vision pipeline feeding plausible solves; the innovation does not need to be large.

## Evidence

### A/B control — fusing off: filter is perfect

With `addMeasurement` never called (Pinpoint-only predict), the fused pose tracked the raw Pinpoint pose **to 0.1° / 0.1 in** through a full drive sequence including a 90° CW turn and a 180° CCW turn (~47 s of motion). Zero anomalies.

### Event log (all robot-stationary unless noted)

| Run | Measurement at injection | Fused H before → after | Dead-reckoning H | Notes |
|---|---|---|---|---|
| A (session 1) | finite "neutral" yaw = current fused heading | 90° → 189.9° → 283.7° (two fuses, steps +99.9°/+93.8°) | 90.0 | steps ≈ the measurement's own yaw value |
| B (session 2) | finite yaw (real MT1 solve, innovation −12.9°) | 90° → 255.6° (+165.6°) | 90.0 | ONE sane yaw fuse flipped it |
| C (session 3) | NaN yaw, XY-only | 90° → 146° (+56°) → 296° (+150°) | 90.0 | two XY-only fuses |
| D (session 4) | NaN yaw, XY-only (67.0, 26.4) | 90° → 256.7° (+166.7°) | **90.0 exact** | the decisive trace below |

### Session D — the decisive trace

At the injection moment (t=2499 ms), logged every loop:

```
t=2356 ms   fused = (55.8, 11.2,  90.0)   raw pinpoint = (55.4, 10.7,  90.0)
t=2499 ms   fused = (55.8, 11.2, 256.7)   raw pinpoint = (55.4, 10.7,  90.0)
```

The XY correction applied normally (barely moved — large-R frame) and the heading moved +166.7° that the gain math cannot produce (masked axis). The raw dead-reckoning heading did not move at all.

**Aftermath — the offset is constant, not chaotic.** After the injection, the robot was driven through a 90° CW turn and a 180° CCW turn. The fused heading tracked the raw Pinpoint heading perfectly at every sample, offset by exactly +166.7°:

```
fused 238.5 / pinpoint  71.8   Δ = 166.7
fused 173.9 / pinpoint   7.2   Δ = 166.7
fused 197.1 / pinpoint  30.4   Δ = 166.7
fused 349.9 / pinpoint 183.2   Δ = 166.7
fused 302.7 / pinpoint 136.0   Δ = 166.7
fused 273.1 / pinpoint 106.4   Δ = 166.7
fused 266.3 / pinpoint  99.6   Δ = 166.7
fused 255.1 / pinpoint  88.4   Δ = 166.7
```

So: one-time injection at `addMeasurement` time, then completely normal predict/correct flow through the corrupted state.

### Bytecode audit (what we verified, and where we suspect)

Disassembled `com.pedropathing.localization.FusionLocalizer` (core-3.0.1). In `addMeasurement(Pose meas, long stamp, Pose R)`:

1. Per-axis validity flags `hasX/hasY/hasH = !isNaN(...)`; innovation vector built with masked components forced to 0; the H component wrapped with `Angle.normalizeSigned`.
2. `S = P + R; Sinv = S.invert(); K = P·Sinv` (full 3×3), then **`K = M·K` and `innov = M·innov` with `M = diag(hasX, hasY, hasH)`**, then `corr = K·innov`; corrected pose = `(x+corr₀, y+corr₁, normalize(h+corr₂))`.
3. The corrected state is written into the history at the (back-dated) stamp, then a **forward walk re-propagates**: `pose = pose.compose(entry.relativeTransform)` over every history entry at/after the stamp, re-running `updateCovariance` per step.

Steps 1–2 are provably inert for a NaN yaw (the masked gain row is zeroed). Therefore the corruption is in the machinery around the insertion — our prime suspects:

- **The forward re-propagation walk** composing `entry.relativeTransform` onto the corrected pose (double-application or frame mismatch of the per-step transforms would scale with the stored transforms, and could explain a large one-time jump);
- **`interpolate(stamp)` / `interpolateTransform` / `Pose.interpolate`** (we checked: the headings at our events were ~90°, far from the ±π wrap, so plain linear-interpolation wrap error seems unlikely — but the interpolation + walk interaction is untested);
- The SE(2) helpers the path uses (`Pose.compose`, `Pose.invert`, `Twist.riemannianLog`, `Twist.exp`).

The predict path (`update()`: `currentRelativeTransform = currentRawPose⁻¹ ∘ deadReckoning.pose(); newPose = filtered ∘ currentRelativeTransform`) is verified correct — excluded by the A/B control.

We did **not** A/B the stamp: all our calls used back-dated stamps. If the injection disappears with `stamp = now` (no interpolation, no re-propagation), that would pin the bug to the interpolate/walk path.

## Suggested unit-level repro (JVM, no hardware)

Sketch — adapt to the `Localizer` interface:

```java
// Dead-reckoning localizer that never moves (stationary robot).
Localizer stationary = new Localizer() {
    private MotionState ms = MotionState.ofVelocity(
            new Pose(55.4, 10.7, Math.toRadians(90)), Velocity.zero());
    @Override public MotionState state() { return ms; }
    @Override public void update() { /* never moves */ }
    @Override public void setPose(Pose p) { ms = MotionState.ofVelocity(p, Velocity.zero()); }
    @Override public void reset() { }
};

FusionLocalizer fusion = new FusionLocalizer(
        stationary,
        new Pose(1, 1, 1),           // P0
        new Pose(0.01, 0.01, 0.01),  // Q
        new Pose(9, 9, 0.27),        // R0 (in², in², rad²)
        100);                        // bufferSize

for (int i = 0; i < 50; i++) fusion.update();   // prime history

double h0 = fusion.state().pose().heading();    // 90°
long stamp = System.nanoTime() - 100_000_000L;  // back-dated 100 ms
fusion.addMeasurement(new Pose(67.0, 26.4, Double.NaN), stamp, new Pose(9, 9, 0.27));
double h1 = fusion.state().pose().heading();

// EXPECTED: h1 ≈ h0 (yaw NaN-masked; XY innovation is only ~17 in).
// OBSERVED ON-ROBOT: h1 − h0 ≈ +2.4 to +3.6 rad (~138°–206°, one-time).
System.out.println("heading delta = " + Math.toDegrees(h1 - h0));
```

If the JVM repro doesn't fire, the on-robot recipe above is the fallback — our four events all occurred with real timing (40 Hz update, ~10 Hz measurements, back-dated stamps, a large populated history).

## Diagnostic experiments that would pin the mechanism

1. `stamp = now` instead of back-dated — does the injection persist? (isolates interpolate + re-propagation)
2. A measurement with **zero** XY innovation (x/y = current pose) — still injects? (isolates innovation magnitude)
3. Sweep `bufferSize` (1 vs 100) — does a single-entry history (no walk) fix it?
4. Log `P` before/after the call — watch whether the heading row/column changes unexpectedly.
5. Compare `Pose.compose` of stored `relativeTransform`s against re-summing raw dead-reckoning deltas over the same window.

## Classes involved

**Used by our integration (all 3.0.1):**

- `com.pedropathing.localization.FusionLocalizer` (+ `FusionLocalizer$KalmanState`) — the corruption site
- `com.pedropathing.localization.Localizer`, `MotionState`
- `com.pedropathing.math.Pose`, `Velocity`, `Matrix`, `Vector`, `Twist`, `com.pedropathing.utils.Angle`
- `com.pedropathing.revhub.localizers.PinpointLocalizer` (+ `PinpointConfig`)
- `com.pedropathing.follower.Follower` — `update()`, `pose()`, `setPose()`, the public `localizer` field

**Needing review/correction:**

- `FusionLocalizer.addMeasurement(Pose, long, Pose)` — the corrected-state insertion at an interpolated stamp and/or the forward history re-propagation walk (prime suspects)
- Possibly the SE(2) helpers it leans on: `Pose.interpolate`, `FusionLocalizer.interpolateTransform`, `Twist.riemannianLog`, `Twist.exp`, `Pose.compose`, `Pose.invert`
- `FusionLocalizer.update()` — verified correct (excluded by control)

## JVM replay — both `main` AND the `core-3.0.1` jar are clean

We compiled the actual Pedro sources (your `main` clone) into a standalone JVM harness and replayed session D exactly: stationary fake dead-reckoning at (55.4, 10.7, 90°), 60 prime updates at 25 ms, then 27 XY-only fuses (NaN yaw, (67.0, 26.4), R = (9, 9, 0.27), 100 ms back-dated stamps at ~10 Hz), a 28th fuse, and a finite-yaw fuse.

**Result: zero heading deviation on BOTH the `main` sources AND the published `core-3.0.1.jar`** (the same harness compiled against each). The correction math — mask, gain, Joseph update, stamp interpolation, `remainder` split, forward walk — is provably inert for a masked yaw on a stationary robot in both versions.

Combined with the full-resolution on-robot trace (the dead-reckoning heading logged **every loop**: 90.00 on all ~200 lines through the injection — never glitched), the injection happened inside `addMeasurement` on the robot, in an environment state the replay does not capture. The candidates we cannot yet rule out: real Pinpoint velocity/heading micro-jitter feeding `updateCovariance`, real variable latencies (30–250 ms) interacting with the buffer-eviction window, or a difference between the published 3.0.1 artifacts and `main` in a class we have not byte-diffed. We are proceeding with a shadow-instrumented copy of `FusionLocalizer` (per-call state dumps) to catch the injection red-handed, and will attach that log.

## Source review (main) — three real bugs found

### 1. `Angle.smallestDifference` is unsigned — heading interpolation can swing the wrong way around the circle

```java
public static double smallestDifference(double one, double two) {
    return Math.min(normalize(one - two), normalize(two - one));
}
```

Both `min` arguments are in [0, 2π), so this returns a **non-negative magnitude**, never a signed shortest difference. `Pose.interpolate` uses it as a *signed* delta:

```java
double headingDiff = Angle.smallestDifference(upperPose.heading(), lowerPose.heading());
double heading = Angle.normalize(lowerPose.heading() + ratio * headingDiff);
```

When the true motion between two history entries is **clockwise** (or crosses the 0/2π wrap), the interpolated heading rotates **counter-clockwise the long way — off by up to ~2π − |true Δ|**. Example: lower = 0.02 rad, upper = 6.27 rad (true motion −0.03 rad through the wrap): interpolated headings sweep 0.02 → 0.05 rad (CCW) instead of 0.02 → 6.285 (CW) — every intermediate pose is wrong by up to ~358°.

This matters for any consumer interpolating across a CW move or a wrap crossing — including `FusionLocalizer.interpolate()`'s `interpolPose` when a back-dated vision stamp brackets entries on opposite sides of a rotation. Correct fix: `return normalizeSigned(one - two);`

### 2. `Matrix.inverse3x3` returns the transpose of the inverse

The cofactors are not transposed: `inv[0][1]` is set from `-(get(0,1)·m22 − get(0,2)·m21)` (cofactor C₀₁) instead of the adjugate entry C₁₀ = `-(get(1,0)·m22 − get(1,2)·m20)`. The result is **M⁻ᵀ**, not M⁻¹. Invisible for symmetric matrices (which is why our S = P + R survived), but wrong for any asymmetric 3×3 — e.g., a state-transition or cross-covariance inverse.

### 3. `FusionLocalizer.setPose` breaks the history chain invariant

```java
if (!history.isEmpty()) history.lastEntry().getValue().pose = setPose;
```

Only the last entry's `pose` is patched; its `relativeTransform` still describes the *old* pose transition. The invariant `poseₖ = poseₖ₋₁ ∘ relativeTransformₖ` (maintained by `update()`, relied on by `interpolate()` and the forward walk) is violated for every subsequent back-dated interpolation, and the next forward walk silently rebuilds downstream poses from the now-stale deltas, discarding the `setPose` correction for anything but the current pose. Benign if `setPose` is only called before the first `update()`; corrupting if called mid-run (as vision-correction workflows do).

### Minor

- `interpolate()` stores `Pose.interpolate(...)` (linear lerp of filtered poses) as the pseudo-state's pose but `interpolateTransform(Pose.zero(), upper.relativeTransform, ratio)` (an SE(2)-geodesic partial of the *raw* delta) as its `relativeTransform` — two different interpolation schemes for one state; the inserted state violates the chain identity unless both schemes agree.
- The pseudo-state inherits `lower.covariance` un-interpolated.
- The constructor's sentinel `history.put(0L, KalmanState(Pose.zero(), ...))` means a back-dated stamp arriving before the first real `update()` interpolates against Pose.zero() (huge spurious innovation). Our stamps never landed there, but it is fragile.

## What we need / next steps

1. **Diff `v3.0.1` vs `main`** for `FusionLocalizer`, `Pose`, `Angle`, `Twist`, `Matrix`, `MotionState`, `Velocity` — our replay proves `main` is clean for this scenario, and the on-robot 3.0.1 injected four times, so the injected state difference must exist somewhere in that set (or in the real-sensor timing interactions).
2. **Or**: we will attach a shadow-instrumented `FusionLocalizer` log (per-call pastPose, gain rows, per-entry walk headings) from the next on-robot event.

## Our workaround (context)

We bypassed `addMeasurement` entirely: accepted vision measurements are applied as a direct exponential blend via `follower.setPose(...)` (XY only; heading left to the dead-reckoning, which is excellent). We'd happily return to the proper Kalman — it does stamp interpolation and proper covariance bookkeeping our blend doesn't — as soon as the injection is fixed.

Happy to provide raw telemetry logs (~2000 lines each, CSV-parseable) for any of the four sessions if useful.

`[TODO: optional — team name / contact]`

---

## Hunt status & plan (2026-09-30)

- **Static prong exhausted**: the `PedroPathing-main` folder we audited is a ZIP extract of upstream **main**, not the v3.0.1 release source (it sits inside our FTC SDK fork checkout; no Pedro git history available locally). Byte-diffs of `addMeasurement`, `Pose.interpolate`, and `Angle` (3.0.1 binary vs main) matched, and a JVM replay built from main **and** against the 3.0.1 jar is clean for our scenario.
- **On-robot hunt mode shipped**: `VisionFusion.USE_PEDRO_FUSE` (Panels-live, default false) routes accepted corrections through Pedro's `addMeasurement` instead of our direct blend. `RobotMain` swaps in `InstrumentedFusionLocalizer` � a subclass with identical math that snapshots the history around every call and dumps any heading change > 2� (logcat tag `IronLog-Fuse`, Dashboard keys `Fuse/injDh` / `injBefore` / `injAfter`, which also land in our telemetry files). Statuses are prefixed `pedro-` while hunting.
- **Procedure**: deploy ? Panels: `USE_PEDRO_FUSE = true`, `FUSE_XY = true`, `MT1_YAW_FUSING = false` ? stationary test (the injection reproduced 4/4 in this configuration) ? `adb logcat -d -s IronLog-Fuse` + pull telemetry.
- **The before/after diff disambiguates the final three candidates**: (1) only the inserted stamp entry's heading changed ? the gain/mask path (contradicts bytecode; implies a 3.0.1-vs-main delta); (2) a walk-rewritten downstream entry changed ? the forward re-propagation; (3) the `upperKey` entry's `relativeTransform` (remainder) went large ? the interpolation split.
- **Interim note**: with the direct blend (default), our pipeline is stable � 734 corrections over two long runs with zero heading anomalies (see the diagnosis artifact �11).

---

## ROOT CAUSE CONFIRMED (on-robot instrumented evidence, 2026-09-30 02:20)

Hunt run: direct blend first (~24 s, 54 corrections, clean), then `USE_PEDRO_FUSE = true`. Within 4 Pedro-path corrections, two injections were caught by the instrumented subclass:

- **Injection #1: dh = +119.2�.** BEFORE: all history entries at 102.3�, all `relativeTransform` headings 0.0. AFTER: all entries at 221.6�, with the **inserted stamp entry** holding pose 102.3� but `rtH = -119.2`, and the **upper entry** at 221.6� with `rtH = +119.2`.
- **Injection #2: dh = +16.1�.** Identical signature: a new split with `rtH = -16.1`, everything downstream pushed +16.1�.

Raw odometry was healthy on every loop (pinpoint heading 102.35� constant), the measurement yaw was NaN (masked), the robot was stationary.

### Mechanism

Every `addMeasurement` call splits the history at the back-dated stamp (`interpolate()` + the `upperKey` `remainder` rewrite + the inserted pseudo-entry) and then forward-propagates. **The split path writes `relativeTransform` values that do not connect the poses they sit between** � the inserted entry's own pose is unchanged while its stored transform carries the full injected angle (-119.2�), and the walk composes that junk (+119.2�) onto every downstream pose. The chain invariant `pose? = pose??1 � relativeTransform?` � which `update()` maintains � is violated by the split/insert path. Injections compound across fuses (each new split adds another jump), matching the observed "warps a little more each fuse" behavior.

The gain/mask path is exonerated: with `M = diag(1,1,0)`, `Ky_heading = 0` in both main and the 3.0.1 binary (bytecode + JVM replay). The injection enters through the **split/insert transform bookkeeping**, which is frame-inconsistent with the deltas `update()` stores.

### Suggested upstream fix

After the forward walk recomputes each downstream pose, **re-derive each entry's `relativeTransform` from the walked poses** (`relativeTransform? = pose??1?� � pose?`) instead of reusing the pre-split stored transforms; and/or make `interpolate()`'s `toMeasurement`/`remainder` split use the same frame convention as `update()`'s deltas. A regression test that fuses N measurements with back-dated stamps on a stationary fake localizer and asserts heading invariance would lock this in (harness available on request).

### Our interim mitigation

We bypass `addMeasurement` with a direct exponential blend (`follower.setPose`), which ran 734 corrections across two long sessions with zero heading anomalies.

---

## FINAL ROOT CAUSE (definitive, arithmetic-verified): `Pose.log()` 2p-wrap hole

The on-robot logcat dumps contain the exact numbers, and both injections decode to **fractions of a full circle**:

| Event | Stamp ratio in bracket | ratio � 360� | Observed injected rtH |
|---|---|---|---|
| #1 | 0.6691 | 240.9� = **-119.1�** | -119.2 (inserted), +119.2 (upper) |
| #2 | 0.9554 | 343.9� = **-16.1�** | -16.1 (inserted), +16.1 (upper) |

Both match `exp(ratio � 2p)` to 0.1�. The split treated the bracketing transform as a **full-circle rotation**.

### The chain

1. Stationary, the Pinpoint's X/Y reads are quantized-identical, but its heading jitters ~1e-5 rad.
2. A tiny **negative** jitter, normalized into [0, 2p) by the `Pose` constructor, becomes heading � **6.2832 rad** in the stored `relativeTransform`.
3. `Pose.log()`:
   ```java
   if (Math.abs(heading) < 1e-6) return new Twist(x, y, heading);
   double A = Math.sin(heading) / heading;
   double B = (1.0 - Math.cos(heading)) / heading;
   double denom = A * A + B * B;   // ? ~1e-12 at heading � 2p (sin � 0, 1-cos � 0)
   ```
   The guard fails for 6.2832, and the general branch runs at a near-singular point: the twist carries **? � 2p** (a phantom full circle). The translation term explodes as `~1/denom` but evaluates to ~0 in our case only because the quantized stationary X/Y made `A�x + B�y � 0` � on a *moving* robot the same hole also corrupts the translation (potential large XY teleport).
4. `FusionLocalizer.interpolate()` splits that transform: `toMeasurement = exp(ratio � 2p)`, `remainder = exp((1-ratio) � 2p)` � phantom rotations of ratio�360� / (1-ratio)�360�.
5. `addMeasurement`'s forward walk composes the junk onto every downstream history entry ? the one-time per-fuse heading injection, compounding across fuses.

This also explains why a JVM replay with a bit-identical fake localizer never fired: deltas were exactly 0, so the wrap never occurred. Real sensor jitter is the trigger.

### The fix (one line + a guard)

```java
public Twist log() {
    double h = Angle.normalizeSigned(heading);   // -1e-5 stays -1e-5, never becomes 6.2832
    if (Math.abs(h) < 1e-6) return new Twist(x, y, h);
    double A = Math.sin(h) / h;
    double B = (1.0 - Math.cos(h)) / h;
    double denom = A * A + B * B;
    if (denom < 1e-12) return new Twist(x, y, h);  // near-singular fallback
    ...
}
```

The same pattern should be audited in any other consumer of `Pose.heading()` that assumes a signed representation.

### Regression test

A stationary fake localizer whose heading jitters �1e-5 rad (sign random per update), N fuses with back-dated stamps; assert the fused heading never deviates > 1� from the dead-reckoning heading. Our InstrumentedFusionLocalizer + harness can be shared on request.

### Consumer status

`USE_PEDRO_FUSE` returns to false (default): our direct blend bypasses `addMeasurement` and ran 734+ corrections with zero anomalies. We can switch back to Pedro's fuse the day this ships in a release.
