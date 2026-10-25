# TurretPivot — FRC → FTC Porting Notes (session log)

Port of the 2026-IndianaJoe FRC turret rotation subsystem to the Biobuzz FTC robot.
Source of truth: `C:\projects\2026-IndianaJoe\src\main\java\frc\robot\subsystems\Turret\TurretPivot.java` + `TurretPivotConstants.java`.
This file is the running session log — append to it in future sessions.
Session 1 = sections 1–8 (initial port, hub-quadrature boot). **Session 2 = section 9 (OctoQuad absolute feedback, 1:1 gearing, field-relative heading) — section 9 supersedes the gearing/boot guidance in sections 2–3.**
**Session 6 = section 13 (shoot-readiness triggers, auto-transfer, left-trigger toggle).** It supersedes the shooter keybinding table in §12.3 (left trigger is no longer hold-to-shoot; left bumper is the unlock/panic).

---

## 1. What was added

| New file | Purpose |
|---|---|
| `TeamCode/.../teamcode/Subsystems/TurretPivot.java` | The turret rotation subsystem: Through Bore quadrature tracking, boot zero logic, wrap-aware soft limits, PID → continuous servo, sanity watchdogs, FRC-parity commands (`C_GotoPositon`, `C_Stop`, `GotoSafeAngle`, `BRAKE`). |
| `TeamCode/.../teamcode/TeleOp/TurretPivotDebug.java` | Standalone bench OpMode ("Turret Debug", group *Debug*) — boots the turret alone (no drivetrain/vision), has the hand-spin gearing verifier. |
| `IronConstants.java` (edit) | `TurretServoName = "turret_servo"`, `TurretEncoderName = "turret_encoder"`. |
| `RobotMain.java` (edit) | `public TurretPivot turret` built via `TurretPivot.tryCreate(opmode)` — null-safe like `Vision`, so every existing OpMode keeps working when the turret isn't configured. |

The subsystem auto-registers with the SolversLib `CommandScheduler` (team convention), so its `periodic()` runs from `RobotMain.RobotRunPeriodic()` in all robot OpModes once the hardware is configured.

---

## 2. IMPORTANT FINDING — the Control Hub cannot read the Through Bore's *absolute* output

The REV Through Bore encoder (REV-11-1271) outputs two things:

1. **Quadrature (A/B wires)** — relative counts. ✅ The REV/Control Hub reads this off a motor port's encoder pins (this is what our port-3 "dummy motor" config gives us).
2. **Absolute angle as a pulse-width signal on the Index wire** — ❌ **the REV Hub has no way to decode this.** Verified directly against this project's FTC SDK 12.0.0: the complete Lynx command set (`com.qualcomm.hardware.lynx.commands.core.*` in `Hardware-12.0.0.aar`) contains only `LynxGetMotorEncoderPositionCommand` etc. — there is no absolute-encoder/pulse-width-input command, and no Through Bore device type anywhere in the SDK. The only SDK code that reads that PWM is the **OctoQuad** sample (`SensorOctoQuadAdv.java`, `DEGREES_PER_US = 360.0/1024.0`) because the OctoQuad has real PWM-input channels.

**What this means for the ported design:** today the boot is *"quadrature is zeroed at Init — humans point the turret generally forward first"*, and multi-turn position is tracked from the quadrature counter (which is already multi-turn on the hub — no per-rev wrap to unwrap until a real absolute read exists). The `TurretPivot.AbsoluteAngleSource` seam inside the subsystem is already wired through the boot-window picker and a per-loop quadrature-vs-absolute consistency check, so adding a true absolute path later **needs no rewrites** — call `turret.setAbsoluteSource(...)` before driving and the boot upgrades to a real absolute solve (the `chooseForwardBootWindow` code is the one-encoder EasyCRT analog).

**Options when you want a true absolute boot:**
- **OctoQuad** (ACME Robotics, I2C): wire the Through Bore's A/B/I into an OctoQuad channel in PWM mode, read absolute angle over I2C, feed `setAbsoluteSource()`. Best option, ~$40.
- **Slow-side gearing instead**: if the encoder were geared *slower* than the turret (see §3), one encoder revolution would cover the whole ±180° and the absolute read would cover the entire range with no boot assumption at all — worth considering before the final mechanism is locked in.
- Keep the current human-forward-pointing boot (works fine, it's what we shipped).

---

## 3. Gearing math (the part to double-check at the bench)

```
ENCODER_REVS_PER_TURRET_REV = TURRET_GEAR_TEETH / ENCODER_GEAR_TEETH = 187 / 24 = 7.7917   (configurable)
counts per turret revolution = 4096 (Through Bore, 1024 PPR ×4 decode) × 7.7917 = 31,906
turret degrees per count     = 360 / 31,906 = 0.0113°
one encoder revolution       = 360 / 7.7917 = 46.2° of turret  ← the absolute "window"
soft limits                  = ±180° → ±15,950 counts
```

- Per your description ("the encoder going around 8-10 times for a full rotation of the turret") the **encoder is on the FAST side**: the turret turns the 187T gear, the 24T pinion is on the encoder shaft. The constants are written that way (`TurretPivot.TURRET_GEAR_TEETH = 187`, `ENCODER_GEAR_TEETH = 24`).
- **If bench bring-up shows the opposite** (encoder on the slow side), swap the two teeth constants — everything else is written against `ENCODER_REVS_PER_TURRET_REV`.
- **Bench check (built into Turret Debug):** right-stick-click starts the hand-spin verifier — hand-rotate the *turret* one full revolution. Expected raw-count delta ≈ **31,906** (or ≈ **525** if the gearing is backwards). Set `ENCODER_COUNTS_PER_REV` to whatever the true value turns out to be.
- `ENCODER_REVERSED` (CCW must read positive) and `SERVO_REVERSED` (positive power must rotate CCW) are the two direction switches — FRC analogs: `TurretClockwiseDir` = `CounterClockwise_Positive`.

---

## 4. RC (Control Hub) configuration to do on the phone

1. **Servo port** (any): device type *Continuous Rotation Servo*, name `turret_servo` — the SWYFT torque continuous servo.
2. **Motor port 3**: device type *Motor*, any motor type (the dummy), name `turret_encoder`. The Through Bore's A/B wires land on port 3's encoder pins. The code reads counts from it and **never drives it** (power is forced to 0 and the watchdogs catch it if something else does).
3. No other config needed. Everything else (gains, limits, gearing) is Panels-tunable under **Panels → TurretPivot**.

---

## 5. Boot procedure (for the humans)

1. Point the turret generally **forward** (within `FORWARD_BOOT_WINDOW_DEG` = ±25°).
2. Select **Turret Debug** (or any OpMode) and press **INIT**.
3. In init, hand-move the turret slightly → boot check flips to `VERIFIED` (proves the encoder counts).
4. PLAY. The turret holds at 0° (forward) once you command a goto.

Recovery for "booted with the turret half-turned": hand-point it forward and hit **left bumper** (`reZeroAtForward()`) — soft re-zero, no restart needed.

---

## 6. FRC → FTC parity map

| FRC (2026-IndianaJoe) | FTC (Biobuzz) | Notes |
|---|---|---|
| TalonFX + MotionMagicVoltage | software PID → `CRServo` power | CRServo has no closed-loop torque; power = speed command |
| `SensorToMechanismRatio` (gearRatio 37.5) | `degPerCount()` from gear teeth | all positions in degrees now |
| `SoftwareLimitSwitch` forward/reverse | `MIN/MAX_ANGLE_DEG` + output-stage guard (refuses to push outward, always allows turning back) | defaults ±180° per this build (FRC was ±198°) |
| `chooseTurretSetpointDeg` + `angleErrorDeg` (turn-back-around logic) | ported **verbatim** | wraps shortest-path error ±180°, tries the ±360° equivalent before clamping |
| Dual CANcoder + EasyCRT boot solve | one Through Bore + `bootZero()` / `chooseForwardBootWindow()` (dormant until an absolute source exists) | see §2 |
| `OutofScopeAlert` | `OUT_OF_SCOPE` latched fault + log | fires when the tracked angle escapes the limits |
| CRT-connected checks | `ENCODER_OK`, encoder-read exception → immediate BRAKE | no feedback = no motion |
| — (new, CRServo-specific) | `STUCK` watchdog: pushing power with no motion > 0.75 s → BRAKE | catches jams / dead encoder |
| — (new, CRServo-specific) | wrap/glitch absorption with `wrapEvents` counter | absorbs counter resets/glitches > ½ rev; expect ~0 on the hub |
| `PositionalTolerance = 0` | `POSITION_TOLERANCE_DEG = 2.0` | FRC value was degenerate (never true exactly); 2° is a real number — tune |
| `GotoSafeAngle(angle, velocityDps)` velocity FF (kV) | same signature (degrees) | shoot-on-the-move support |
| `SubSystemState` enum, `atSetpoint()`, `BRAKE()`, `C_GotoPositon()`, `C_Stop()`, `getPosition()`, `getRawAngle()` | same names | `getPosition()`/`getRawAngle()` now return **degrees** (FRC's getPosition returned rotations) |
| kP=110, kD=0.15, kS=0.25, kV=0.07 (volts, TalonFX) | kP=0.030, kI=0, kD=0.002, kS=0.06, kV=0.004 (power, CRServo) | **starting points only** — not transferable units, tune on the bench |

**Deliberate changes / suggestions (flagging for review):**
- **Edge slow zone** (`EDGE_SLOW_ZONE_DEG` 20°, cap 0.35): the TalonFX's MotionMagic cruise limit has no CRServo equivalent, so the power cap shrinks near the soft limits. Cheapest way to keep the servo from slamming the ends.
- **Deadband parking** (`SERVO_DEADBAND` 0.08, same idea as your `ContinuousServo`): a CRServo creeps at tiny powers; parking below the threshold also sets the practical holding accuracy (≈ kP⁻¹ × deadband ≈ ±2.7° with the starting kP). Raise kP (and let kI hold) if you need tighter.
- **Self-computed velocity** (count delta / dt) instead of `getVelocity()` — immune to the dummy-motor config type's ticks-per-rev scaling, and works even when parked.
- The FRC `RPM`/temperature/current telemetry has no CRServo equivalent — intentionally omitted.

---

## 7. Tuning quickstart (Panels → TurretPivot)

1. `MAX_POWER = 0.6`, `EDGE_MAX_POWER = 0.35` — keep while first testing.
2. Raise `kP` until the turret reaches a 90° goto crisply without buzzing.
3. Add `kD` if it overshoots (power per deg/s — start 0.002).
4. Add `kI` only if it parks short of the target (deadband-vs-hold fight); keep `I_MAX` small.
5. `kS` = the power that just barely moves the turret from rest.
6. `kV` = power per deg/s of *desired* velocity — matters only once shots are on the move.
7. `MAX_POWER` is also your turret speed limit — set it from match requirements, not servo max.

---

## 8. Next steps (queue for future sessions)

- [ ] Bench bring-up: verify OctoQuad channel 0 reads (§9), calibrate `ENCODER_ZERO_OFFSET_DEG`, verify field-relative sign (§9.4).
- [ ] Wire `turret` into `KeyBindings` + `CommandFactory` (TeleOp driver control).
- [ ] Port the aiming path: robot-relative → turret-relative angle (IndianaJoe `C_Align` / ballistics) — the turret angle target comes from `visionHeading - robotHeading` + ballistics offset; feed through `GotoSafeAngle(angle, velocityDps)` with the chassis yaw rate as the feedforward (shoot-on-the-move).
- [x] ~~Optional: OctoQuad → `AbsoluteAngleSource` for a true absolute boot~~ — DONE in session 2 (§9).
- [ ] Revisit limits: if the turnstyle is mechanically continuous, consider ±200°+ limits so the wrap-aware setpoint logic gets through-the-back travel (FRC behavior) instead of long-way-around.

---

## 9. Session 2 — OctoQuad MK1 absolute feedback, 1:1 gearing, field-relative heading

### 9.1 Hardware now in play

- **OctoQuad MK1** on **Control Hub I2C port 3**, RC-config name `octoquad` (`IronConstants.TurretOctoQuadName`).
- **REV Through Bore** on **OctoQuad channel 0** *(session 3: moved to **channel 7** — see §10.5)*, pulse-width (absolute) mode. Remember the cable mod from the SDK sample: at the 6-pin REV connector end, the yellow wire moves from pin 3 to the ABS pin-5 position.
- Gearing re-done **1:1**: one encoder revolution = one turret revolution. `TURRET_GEAR_TEETH = ENCODER_GEAR_TEETH = 1`.
- The old hub-port arrangement (`turret_encoder` dummy motor on port 3) is now a **fallback only** — the subsystem auto-uses the OctoQuad when it's configured and falls back to hub quadrature when it isn't (handy while both configs exist on the phone).

RC config to add: I2C port 3 → device type **OctoQuad**, name `octoquad`. Nothing else — the code configures the channel at init: `resetEverything()` → bank **ALL_PULSE_WIDTH** (the OctoQuad only banks whole-chip or bank1-quad/bank2-PWM; channel 0 must be PWM, so all-PWM it is) → pulse range `1–1024 µs` (Through Bore: 1–1024 µs = 0–360°) → `saveParametersToFlash()`. **Caution:** `resetEverything()` wipes the settings of ALL channels — the turret must remain the only user of this OctoQuad, or future users must re-apply their config after ours (the nested `OctoQuadAbsoluteSource` javadoc says this too). If you later add quadrature encoders on the same board, move the turret wire to a channel 4–7 and switch the bank to `BANK1_QUADRATURE_BANK2_PULSE_WIDTH`.

### 9.2 Boot + calibration (the new zero story)

At 1:1 the absolute reading covers the entire ±180° range, so the boot no longer needs multi-turn guessing — it needs a **magnet zero calibration** (the FRC CRT `MagnetOffset` analog):

1. First boot(s): leave `ABS_ZERO_CALIBRATED = false`. Boot behaves like before — "the turret points forward right now = 0°". The dashboard shows `Turret abs raw deg` and the **suggested offset** live, and `reZeroAtForward()` (left bumper in Turret Debug) logs the exact value to enter.
2. Bench: point the turret forward, read `abs raw deg`, set `ENCODER_ZERO_OFFSET_DEG` to that number in Panels, flip `ABS_ZERO_CALIBRATED = true` (Panels values do NOT persist across deploys — put the final numbers into the source constants once they're dialed in).
3. From then on, every boot solves the true angle straight from the OctoQuad — no human pointing required. Pointing forward is now only a **sanity check**: if the calibrated boot angle lands more than `FORWARD_BOOT_WINDOW_DEG` (25°) from forward, the subsystem logs a loud `ZERO_OFFSET_MISMATCH`, raises the fault, and *falls back to assumed-forward* (graceful, matches your pointed-forward procedure) instead of trusting stale calibration. Re-point + re-zero (or recalibrate) and go.

### 9.3 The wrap-seam / overshoot question you asked about

With 1:1 gearing and soft limits at exactly ±180°, the total travel is **exactly one revolution** — the absolute encoder's 0/360 seam sits inside the working range, so *any* overshoot past a limit crosses it. This is handled, not lost:

- The tracker integrates **wrapped deltas** (`angleDiff1`, shortest path in ±0.5 rev). A seam crossing is just an ordinary step: the continuous angle reads e.g. −181°, the soft-limit guard refuses to push outward, and the PID pulls it back into range. No wrap-around glitch, no crash, no loss.
- Each crossing increments `Turret wrap/glitch events` — expect ~1 per limit excursion; a fast-growing count means wiring/CRC trouble, not motion.
- The unwrap can only fail if the turret rotates **more than half a revolution between two loop reads** (>~9000°/s at 50 Hz) — mechanically impossible for this turret. There is no speed at which it can "get lost."
- Startup pointing "within a few degrees of forward" keeps the seam far from the boot position anyway; and since the boot read is absolute, boot is unambiguous regardless.
- Feedback-loss tolerance: one bad I2C/CRC read is ignored; `FEEDBACK_FAIL_LIMIT` (10) consecutive failures brake the servo. The stuck watchdog (power with no motion) covers a dead OctoQuad too.

### 9.4 Dashboard additions (the heading readouts)

- **Panels → TurretPivot telemetry**: `Turret heading (robot rel)` (+CCW from chassis-forward), state, velocity, power, feedback path, `abs raw deg`, calibration status + suggested offset, wrap events, faults.
- **Panels main dashboard (RobotMain)**: `Turret heading (robot)` and `Turret heading (field)` — field-relative = chassis heading (Pedro, CCW+, radians) + turret robot-relative, normalized ±180. Reported every flush, including init_loop.
- **Panels Field widget**: an orange **turret ray** drawn from the robot body along the turret's field heading (`PanelsFieldRenderer.drawTurretRay`, queued into the same single canvas flush as the robot; toggle/color/length under `PanelsFieldRenderer`).
- **Verify the sign once at the bench**: hold the turret still and rotate the chassis CCW — the field-relative value should stay put. If it swings, the Pedro heading sign convention differs from what's assumed and needs a minus sign in `RobotMain.reportTurret()`.

### 9.5 Resolution note (PWM quantization)

The Through Bore's absolute PWM has 1024 steps = 0.35° at the turret (1:1). Position accuracy is therefore ~±0.2° and the per-loop velocity derivative steps ~17°/s per PWM step — `VELOCITY_EMA_ALPHA` (0.4) smooths the D term. If you ever want finer *velocity* resolution again, the fallback path (hub quadrature at 4096 counts/rev, 0.088°) can be re-wired alongside the OctoQuad (A/B to port 3, ABS wire to the OctoQuad) — the subsystem auto-detects both.

### 9.6 Compile status

`:TeamCode:compileDebugJavaWithJavac` passes (gradle exit 0) after the session-2 changes.

---

## 10. Session 3 — goBILDA 5203 drive motor (replaces the SWYFT servo)

### 10.1 What changed

The PID output stage is now a pluggable **drive stage** (`TurretPivot.DriveStage`) with two implementations taking the same -1..1 power, so the PID, soft limits, edge slow-zone, watchdogs, and all Panels tuning carry over unchanged:

- **`MotorDriveStage`** — goBILDA 5203 (`DcMotorEx`, name `turret_motor`): `RUN_WITHOUT_ENCODER` (the OctoQuad is the position truth — the motor's own encoder is not needed), `BRAKE` zero-power behavior (active position holding — a big upgrade over the servo's freewheel), `SERVO_DEADBAND` intentionally NOT applied (the motor holds via the full PID output, which is what lets kI hold position tightly), Lynx current alert armed at `CURRENT_ALERT_AMPS` with current + `isOverCurrent` telemetry and a latched `OVER_CURRENT` fault (alert only, no auto-brake — the soft limits keep it out of the stops; FRC analogs: stator current telemetry, `maxStatorCurrent`).
- **`ServoDriveStage`** — the legacy SWYFT CRServo, unchanged behavior (deadband parking, freewheel on brake).

**Selection**: motor if configured, else servo; if BOTH are configured, `PREFER_MOTOR_DRIVE` (Panels-tunable, default true) decides — flip it to false while the motor is in the RC config but not yet installed and you want to keep running the servo. The active drive shows in telemetry (`Turret drive`, `Turret drive` in Turret Debug too).

### 10.2 The gear-ratio config you asked for

- `TurretPivot.MOTOR_GEAR_RATIO` — motor output-shaft revolutions per ONE turret revolution. **Currently a placeholder (1.0) — set it when the new gearbox is chosen.**
- Important subtlety: this ratio does **not** affect position feedback at all — the OctoQuad reads the turret directly at 1:1, so `MOTOR_GEAR_RATIO` only sets the *speed budget*: `maxTurretSpeedDps()` ≈ `MOTOR_NO_LOAD_RPM × 360/60 / MOTOR_GEAR_RATIO` (shown in telemetry as `Turret est max speed` when the motor drive is active). E.g. a 312 RPM 5203 at 50:1 → ~37 deg/s no-load estimate. Use that sanity number when picking the final ratio — you want the full ±180° sweep to complete in about a second or two: ratio ≈ `312×360/60 / desiredDps`.
- `MOTOR_NO_LOAD_RPM` = the 5203 variant's no-load speed (312 for the 19.2:1 variant — same family the team already uses on the intake at 537.7 counts/rev; 223 for the 25.9:1 variant). Only used for the speed estimate.
- `MOTOR_REVERSED` — software direction flip so positive power = +CCW (same contract as `SERVO_REVERSED`).

### 10.3 RC config + bench bring-up (when the motor arrives)

1. RC config: motor port → device type goBILDA 5203 (whichever variant matches), name `turret_motor`. Direction is handled in software — don't worry about the port's "Reversed" checkbox (leave it off and use `MOTOR_REVERSED` so it's Panels-tunable).
2. Wire it up; the encoder cable from the 5203 to the hub is optional (unused by this code).
3. Turret Debug: confirm `Turret drive` shows the 5203; jog slowly (MAX_POWER still 0.6, EDGE_MAX_POWER 0.35); verify `MOTOR_REVERSED` so +stick = CCW.
4. Re-tune: the power-united gains are roughly in the right ballpark but the 5203 has much more authority than the servo — expect to REDUCE `kP` or `MAX_POWER` if it buzzes/oscillates, and raise `MAX_POWER` for full-range sweeps once trustworthy. `kS` will be smaller for the motor (less breakaway stiction than a servo). The stuck watchdog now also protects against a jammed gearbox.
5. The servo can stay configured as an automatic backup drive — remove `PREFER_MOTOR_DRIVE`/`turret_servo` only when the transition is done.

### 10.4 Compile status

`:TeamCode:compileDebugJavaWithJavac` passes (gradle exit 0) after the session-3 changes.

### 10.5 Session 3 addendum — Through Bore moved to OctoQuad channel 7

The turret's ABS wire now sits on **OctoQuad channel 7** (was channel 0), keeping channels 0–3 free for future quadrature encoders. What actually matters about that move:

- **Channel + bank config must match.** The OctoQuad banks channels as ALL-quadrature, ALL-pulse-width, or bank1(0–3)-quadrature / bank2(4–7)-pulse-width. With the encoder on 7, the code now applies **`BANK1_QUADRATURE_BANK2_PULSE_WIDTH`** (`TurretPivot.OCTOQUAD_BANK_CONFIG`) instead of the old `ALL_PULSE_WIDTH` — that keeps the quadrature half of the chip usable while channel 7 stays pulse-width. (`TURRET_OCTOQUAD_CHANNEL = 7` likewise.) If you rewire again, change both together; bank config re-applies on OpMode restart.
- With nothing wired on 0–3, the bank split behaves identically for the turret — the telemetry, calibration and wrap handling are unchanged. Quadrature channels reading zeros is normal.
- If you DO wire quadrature encoders onto channels 0–3 later, they'll need their own small reader (the turret subsystem only touches channel 7) — and note the optional best-of-both setup: the Through Bore's **A/B wires on a channel 0–3 quadrature input** plus its ABS wire on 7 would give the turret high-res quadrature *and* absolute on the same OctoQuad; the subsystem's transport seam (`countsSource` + `absoluteSource`) is already built to accept exactly that if it ever gets wired.

---

## 11. Session 4 — the "wild heading" capture: analysis + glitch defenses + wiring checklist

Capture: `panels-capture-20261016-025502.json` (TeleOp, turret hand-moved 0 → −90 → 0 → +90).

### 11.1 What the data shows

| Phase | abs raw deg | behavior |
|---|---|---|
| t=2.5→9.3s (untouched) | **35.5** stable | healthy at rest |
| t=9.4→21.2s (hand motion) | jumps **every loop**: −72.8, −40.7, +155, −175, −177.9°… | 91 events >10°/loop, up to ±178° per 37 ms loop — physically impossible by hand |
| rest stops mid-run | 103.0 (1.2 s), 181.4 (1.4 s) | stable-but-arbitrary |
| final rest | 272.8 | displayed +237.3° instead of the expected +90° |

Conclusions:
1. **The tracking math is fine; the RAW OctoQuad reading is garbage during motion** — the tracker faithfully integrated garbage (that's the "massive and wild").
2. **Even the rest readings don't reconstruct the motion** (rest-to-rest deltas +67.5°, +78.4°, +91.4° — the story needs a −90 in there), so the reading is not just transiently glitching — the channel is not reliably receiving the ABS pulse train at all.
3. Field-relative was just the ±180-normalized robot-relative — chassis/Pedro heading was innocent.
4. The old `mod1()` inside `readRevs()` masked out-of-window values into plausible-looking angles — making diagnosis harder (fixed, see below).

### 11.2 Leading physical suspect — the cable mod

The REV Through Bore's absolute PWM is NOT on the stock 4-pin cable pinout. Per the SDK's `SensorOctoQuadAdv` sample, you must **move the yellow wire from pin 3 to the ABS pin-5 position on the 6-pin REV connector end** (lift the white tab with a small flathead, pull the crimp, reinsert). If the channel instead sees the A/B quadrature wires, PWM mode measures garbage between random edges — **stable when static, wild when toggling, arbitrary rest values: exactly the capture signature.**

Checklist:
1. Was the yellow-wire mod done? Does ONLY the ABS signal (+5V/GND) reach the ch7 header's signal pin?
2. Connector fully seated on ch7?
3. Run the new **90° bench test** (below) — a healthy ABS path shows ~±256 µs per exact 90° hand rotation.
4. After the wiring is confirmed: **recalibrate the zero** — every rest reading in this capture is untrustworthy, so any previously noted offset suggestion (181.4 etc.) is meaningless.

### 11.3 Code defenses added (all compiled, gradle exit 0)

- **Range validation in the source**: raw µs outside `[1−slack, 1024+slack]` (slack 16 µs, tunable) returns an empty read + `OOR` counter instead of being mod-1'd. `Turret abs raw us` now on the dashboard.
- **Median-of-3 filter** (`ABS_MEDIAN_WINDOW`) on the raw stream before unwrapping: single-sample garbage between good reads is dropped entirely (one-tick lag on real motion).
- **Implausibility gate** (`ABS_MAX_PLAUSIBLE_DPS` = 2000°/s × dt, tunable): steps beyond the mechanism's real capability (motor no-load at MAX_POWER ≈ 1100°/s + hand margin) are NOT integrated — counted as `glitch`; after 8 consecutive rejects the tracker re-anchors to the raw reading so it can't freeze forever.
- **Boot stability check**: at init the source is sampled 8× (12 ms apart); all must be valid with <3° spread, else `ABS_BOOT_UNSTABLE` fault + loud log (boot proceeds on the pointed-forward fallback). Also verifies the channel is alive BEFORE trusting a boot angle.
- **Config echo**: the code now reads back what the chip actually has (bank config, pulse window, tracksWrap) at init and logs it — a silently-failed flash write is instantly visible.
- Diagnostic counters `OOR / CRC / glitch` on the dashboard and in Turret Debug; Back clears them.
- Turret Debug gained the **90° ABS bench test**: dpad-up marks the raw µs, rotate exactly 90°, dpad-down prints the µs delta (healthy ≈ ±256 µs/90°; near-zero, ~4×, or out-of-window = wiring/config problem).

### 11.4 What to run next session

1. Turret Debug → watch `OQ raw us` while hand-rotating slowly across the full sweep: it should sweep 1→1024 µs smoothly and monotonically (twice: once CW, once CCW), and sit still at rest.
2. The 90° test (§11.3) in both directions.
3. If the µs stream is healthy: calibrate `ENCODER_ZERO_OFFSET_DEG` (forward, read `abs raw deg`, set, flip `ABS_ZERO_CALIBRATED`), then repeat this exact capture test — the heading should track 0 → −90 → 0 → +90 within a degree or two.

---

## 12. Session 5 — CRT dual-absolute-encoder boot + turret-first keybindings

The turret mechanism was reworked to the new design: a **176-tooth ring gear** that spins a **64-tooth idler**, which drives **two** REV Through Bore absolute encoders — a **31T pinion (encoder A)** and a **32T pinion (encoder B)**. Because both pinions mesh the same idler, the idler teeth cancel: each encoder turns `176 / pinionTeeth` revolutions per turret revolution (A ≈ 5.677, B = 5.5 exactly). The hub cannot read the Through Bore PWM directly, so both ABS wires land on the OctoQuad: **B (tracking, 32T) on channel 6, A (31T) on channel 7** (bank2 pulse-width — session 5c corrected this doc; the code constants and the working wiring are B=6/A=7).

### 12.1 CRT boot solve (Chinese Remainder Theorem)

Each encoder reading alone is ambiguous — encoder B repeats every `360/5.5 ≈ 65.5°` of turret travel. But the *pair* (A, B) is unique over `lcm(31, 32)/176 ≈ 5.64` turret revolutions, far beyond the soft-limit travel, so the boot is unambiguous anywhere:

1. At init, one bulk OctoQuad read captures BOTH channels (no extra I2C traffic; the partner value is served from the cached transaction).
2. The solver removes the calibrated zero offsets, then enumerates the candidate turret turns that satisfy encoder B (`t = (bFrac + k)/R_B` for integer k within the soft limits).
3. For each candidate it predicts encoder A's fraction (`frac(R_A × t)`) and keeps the candidate whose prediction matches the measured A within `CRT_MATCH_TOL_ENC_REVS` (default 0.03 enc-revs ≈ 2 turret deg — EasyCRT-style quarter-spacing guard is much tighter; 0.03 tolerates chain backlash).
4. The winning candidate becomes the boot angle. A mismatch beyond tolerance = **solve failed** → fall back to the legacy single-encoder boot (assumed-forward) + loud log. This is the FRC IndianaJoe `EasyCRT` algorithm ported by hand (`crtSolveTurretTurns()` in `TurretPivot`).

The CRT solve replaces the "point the turret forward" boot entirely **once calibrated** — the turret can be parked at any angle at init and the solved heading is exact.

### 12.2 Calibration (one-time, at the bench)

1. Park the turret at **forward (0°)**.
2. Read the two raw dashboards in Turret Debug: `Turret CRT A raw us` and `Turret CRT B raw us` (µs) — convert to revolutions by dividing by 1024 (or read the `abs raw` values shown in the panel).
3. Enter those raw revolutions into `CRT_ZERO_OFFSET_A_REV` / `CRT_ZERO_OFFSET_B_REV` (Panels → TurretPivot) and flip `CRT_ZERO_CALIBRATED = true`.
4. Hand-rotate the turret a known amount and verify the solved heading tracks. If the solve comes up empty with a large A-vs-prediction mismatch, try flipping `ENCODER_A_REVERSED` (the two magnets are independent).

**SESSION 5b — the zero offsets are RAW-frame.** The offsets are subtracted from the raw pulse-width reading *before* the `ENCODER_REVERSED` / `ENCODER_A_REVERSED` direction flips are applied (`solveFrameRevs()` in `TurretPivot`), so they are a property of the magnet's physical position only:

- Enter exactly what the dashboards show (raw µs / 1024) — no sign juggling.
- **Flipping a reversal flag never invalidates the calibration.** If the dashboard turret rotates the wrong way, flip the flags and redeploy — the boot position stays put. (Before 5b the offsets were consumed *after* the flip: flipping moved the software zero by `2×offset mod 1` on each encoder — with A at 750/1024 that is ≈ 0.465 enc-revs ≈ 29° of turret — and desynced A from B, so the CRT solve came out in the wrong spot even though the raw readings were identical.)
- If the solve comes up empty after a flip, that flag now points the wrong way relative to the physical rotation (positive turret CCW must read positive) — flip it back; do not touch the offsets.
- The same raw-frame rule covers the legacy single-encoder zero (`ENCODER_ZERO_OFFSET_DEG`) and the `reZeroAtForward()` suggested value.

After boot, the match tracks on **encoder B only** (the 32T pinion — the slower of the two, easing the per-loop unwrap) with the session-4 glitch defenses unchanged. **Encoder A becomes a passive health monitor**: every loop the tracked position predicts A's reading, and a persistent disagreement while stationary (beyond `CRT_CROSSCHECK_TOL_DEG`, 5 consecutive loops) latches the `CRT_CROSSCHECK` fault — catching a slipped magnet or a drifting chain mid-match.

### 12.3 Keybindings — turret-first (the big behavioral change)

The chassis is **never aimed anymore**. `AimAtGoal` was rewritten: it now requires the **turret subsystem only** (the follower is read for the target bearing but never driven), and each loop it computes the closest alliance target's robot-relative heading and commands `TurretPivot.GotoSafeAngle` with a chassis-yaw-rate feedforward (so the turret holds the field aim while the chassis turns). The old follower turn-PID, integral state, and `shootCmd`-suppresses-drive logic are gone — `applyDrive()` is now unconditional manual field-centric.

| Control | Binding | Notes |
|---|---|---|
| Drive translation | Left stick (as before) | Always active, even while aiming |
| **Chassis rotation** | **Right stick X** | Standard layout (SESSION 5 REV 2: reverted to the stick; dpad no longer turns the chassis) |
| **Manual turret jog** | **Dpad left / right** | Left = CCW (+), right = CW (−), via `NudgeDeg` so soft limits + PID stay active. **GATED on `!isShooting()`** — ignored while the shoot compound runs |
| Shoot (aim + LUT RPM + gate) | Left trigger hold | `ShootAtGoal()` takes no stick suppliers; the turret aims, the chassis is free |
| Intake | Right trigger | Unchanged |
| Transfer | LB forward / RB reverse | Unchanged |
| Servo presets/nudges | X / dpad up/down | Unchanged |

**Fully automatic while shooting (the IndianaJoe behavior):** holding the trigger hands aiming over entirely — `AimAtGoal` re-picks the closest point on the driver's alliance from `AimPoints.pp` every loop and tracks it with the turret's own PID + soft limits + a chassis-yaw-rate feedforward (the turret holds the field aim while the chassis turns). The dpad jog is gated on `!isShooting()`, so the driver cannot interrupt the automatic aim mid-shot — the dpad only spins the turret when no shot is active (mostly the turret is auto; the dpad exists for debugging and manual rotation). The driver keeps full chassis control throughout: left stick translation, right stick rotation.

### 12.3.1 Shooter pivot offset (where shots come from)

`AimAtGoal` now aims from the **shooter**, not the robot center — matching IndianaJoe's `TargetSelector` (`robotPose.plus(ShooterPivotRobotRelative2d)`). The offset lives in the already-imported `NoFireZones` constants (`SHOOTER_OFFSET_X_IN` / `SHOOTER_OFFSET_Y_IN`, robot-relative, shared with the no-fire-zone math — a single source of truth; set them in Panels to the turret pivot's position on the robot, defaults 0). `AimAtGoal.getShooterPose()` rotates the robot pose through its heading to apply the offset, and all three consumers use it: the **bearing** to the target, the **distance** (feeds the RPM LUT), and **closest-target selection**.

Not ported (optional later): IndianaJoe also adds tangential velocity at the offset pivot from chassis rotation (`-ω·py, ω·px`) to the shot-solution velocity — relevant only when firing while spinning the chassis fast.

### 12.4 Compile status

`:TeamCode:compileDebugJavaWithJavac` passes (gradle `BUILD SUCCESSFUL`) after the session-5, 5b (raw-frame zero offsets), 5c (gate scale, corrected `MOTOR_REVERSED = false` after bench ground truth, boot direction guard), 5d (drive ratio 2.75:1, kS taper, settled-hold), 5d-rev2 (real ~10 Hz PWM refresh ceiling → `MAX_POWER` 0.40, nudge-from-actual-position, refresh-interval velocity, CRT re-solve at PLAY) and 5e (§12.11 hybrid tracking: motor built-in quadrature per-loop + PWM absolute anchor, staged `MAX_POWER` 0.50) changes. The `FtcRobotControllerActivity` deprecated-API note is pre-existing and unrelated.

### 12.5 What to run next session (bench bring-up order)

1. Wiring (matches the code constants): tracking encoder **B (32T pinion) ABS → channel 6**, encoder **A (31T pinion) ABS → channel 7**; confirm both `Turret abs raw us` (B) and `Turret CRT A raw us` (A) dashboards read stable µs at rest.
2. Calibrate the CRT zero offsets (§12.2), flip `CRT_ZERO_CALIBRATED`, power-cycle, and confirm the boot log prints "CRT boot solve: turret ±X deg" with the turret parked at a random angle.
3. Hand-rotate the turret across the full travel and verify the solved heading tracks 0 → ±90 → ±180 within a degree or two (same test as §11.4).
4. Verify the cross-check: hold the turret still and confirm `Turret CRT crosscheck` stays near 0°; then simulate a failure by flipping `CRT_MATCH_TOL_ENC_REVS` way up and confirming the boot falls back loudly.
5. Driver test in TeleOp: right stick rotates the chassis (standard drive), dpad left/right jogs the turret when no shot is active, left-trigger aim converges the turret with the chassis free and fully driver-controlled.

### 12.6 Session 5c — runaway incident (capture 2026-10-23) and fixes

Capture `panels-capture-20261023-003251.json` (20 s @ 50 ms): dpad nudges drove the turret continuously in one direction, and left-trigger auto-aim zipped left/right, jerked, and never settled. Three independent causes, all visible in the one capture:

1. **The tracked stream ran UN-FLIPPED at runtime (the runaway — corrected diagnosis).** Bench ground truth: positive power rotates the turret **CCW** in real life, and CCW is the positive display convention. Both encoders read raw-inverted vs physical (raw *falls* with CCW — first seen as the mirrored dashboard when the flags were false), so the stream must apply the flip to be stable. The capture shows the tracked heading going **negative** during the +0.6 power phases while the raw deltas (taken at face value — no aliasing correction needed) go negative too: the stream behaved as `tracked = +raw`, i.e. **`ENCODER_REVERSED` was effectively FALSE at runtime even though the code file says TRUE**. With the stream mirrored, positive power moved tracked position AWAY from the command → positive feedback → runaway. The earlier 5c conclusion ("drive inverted", `MOTOR_REVERSED = true`) was based on an aliasing re-interpretation of the saturated raw deltas and is **retracted**; `MOTOR_REVERSED` is back to `false` and matches the verified drive direction. Encoder A's flag, by contrast, verifiably WAS honored at runtime (the `+28.73 deg` rest crosscheck is the exact fingerprint of 5b's offset-then-flip with `ENCODER_A_REVERSED = true`), so the deployed build was current — the anomaly is isolated to the B stream's runtime value. Leading suspect: a **stale saved override in the Panels config page** (a `@Configurable` value saved there re-applies at init and beats the compiled default). **Action: open the Panels Config → TurretPivot and check `ENCODER_REVERSED` — if it shows a stale `false`, set it `true` (or clear the override). The new "Turret enc B flip (runtime)" telemetry line prints what the robot is actually using.**
   **Rule (unchanged): "positive power must produce positive tracked motion" — but when the loop runs away, suspect the ENCODER flip first, not `MOTOR_REVERSED`.** New guard: the boot hand-move check now cross-checks direction — `pollBootDirectionGuard()` accumulates the FLIPPED-B motion and encoder A's solve-frame motion over the same hand motion and latches `B_DIR_MISMATCH` (+ telemetry "Turret boot direction") if their signs disagree after ~15° on both channels. Residual limitation: it compares the encoders against each other; if BOTH flips were wrong they'd still agree, so also confirm absolute direction by hand (CCW must RAISE "Turret heading").
2. **Plausibility gate mis-scaled (+ the PWM aliasing ceiling).** The gate compared ENCODER-rev steps against a TURRET-rev threshold (5.5× too tight): motion above ~364 deg/s was rejected wholesale, the tracker froze, and the 8-reject re-anchor teleported the tracked position — the phantom ±175 deg/s velocity spikes then kicked kD into ±0.6 power flips (the jerking). Fixes: gate threshold converted to encoder revs and clamped at 0.5 enc-rev/loop, plus a `MAX_POWER` cap to keep closed-loop speeds under the PWM aliasing ceiling (encoder B repeats every 65.5 deg of turret travel — above ~620 deg/s at the observed ~53 ms loop a true ≥1-rev step aliases into a small wrong-direction step no threshold can unwrap). The cap was initially sized at 0.3 against the then-believed 1872 deg/s no-load speed; session 5d set the real drive ratio (2.75:1) and re-sized the cap to 0.8 (~545 deg/s) — see §12.8.
3. **Encoder A's zero shifted ~28.7°.** At rest (turret physically at forward per encoder B: raw 287 µs ≈ its 283 offset) encoder A read 286 µs vs its calibrated 750 — a 0.4531 enc-rev shift ≈ 28.73° of turret, exactly the crosscheck error shown (`+28.73 deg`, ~199 fails). The CRT boot solve correctly refused to run on disagreeing encoders and fell back to assumed-forward. Most likely the A magnet/pinion coupling slipped during the violent slamming. Re-calibrate both zero offsets at forward (§12.2 — raw dashboards ÷ 1024) and inspect the A coupling; if the at-rest crosscheck drifts again with no motion, the magnet is slipping and needs a mechanical fix.

Also fixed: `faultString()` now reports the `CRT_CROSSCHECK` fault (the capture showed 199 crosscheck fails next to "faults: none"), and the debug OpMode wiring doc now matches the code (B → ch6 primary, A → ch7 cross).

**Corrected bench procedure (supersedes the earlier 5c list):**
1. Check the Panels Config page for TurretPivot: `ENCODER_REVERSED` and `ENCODER_A_REVERSED` must both read `true` at runtime — clear any stale saved override. The "Turret enc B flip (runtime)" telemetry line shows what the robot actually has.
2. Re-calibrate BOTH CRT zero offsets at forward from fresh raw dashboards ÷ 1024 (encoder A will read ≈ 286/1024 = 0.2793, not 750/1024, unless the magnet re-seated itself); inspect the A magnet/pinion coupling for slip.
3. At init, hand-move the turret ~20°: "Turret encoder live (boot)" → VERIFIED and "Turret boot direction" → OK (B and A agree). If it says MISMATCH, the stream's flip is not applied — fix the runtime flag before anything else.
4. Dashboard absolute-direction check: hand-rotate the turret **CCW** — "Turret heading" must go **positive**. If it goes negative with the guard saying OK, both flips are wrong (not seen so far).
5. Then: dpad nudge ≈ 4° and settles; auto-aim converges and stops. The gate fix stays in place regardless of the sign resolution (`MAX_POWER` re-sized in §12.8).

### 12.7 Session 5c validation — capture 20261023-005534 (CRT system verified end-to-end)

After re-calibrating both CRT offsets and clearing the stale runtime flag, the capture confirms every piece: powered off, turret hand-placed at ~45° CCW, robot booted — **`Turret CRT boot: SOLVED (diff 0.0007 enc-revs, 6 candidates)`**, dashboard read **+41.0°** instantly (user-confirmed correct). Then hand-rotated CW to the rear (heading 41.0 → **−181.7**, 223° of CW tracked as *decreasing* — correct sign), rested ~2 s, then hand-rotated CCW with stops at ~−91° (left side) and ~0° (forward), ending at **+49.9°** (~231° of CCW tracked as *increasing* — correct sign, endpoint matching the physical ~45° left-front). Through ~455° of multi-wrap travel: **wrap/glitch events 0**, OOR/CRC 0/0, crosscheck transient ≤ ±2.5° during motion (PWM frame phase skew between the channels — expected) settling to ≤ 0.5° at every rest, faults `none`, `Turret enc B flip (runtime): true`. Back-solving the offsets from the trace gives `CRT_ZERO_OFFSET_B_REV ≈ 248 µs`, `CRT_ZERO_OFFSET_A_REV ≈ 649 µs` — consistent at both ends of the session (no drift ⇒ the A magnet/coupling is stable after re-calibration). Remaining: exercise the closed loop (dpad nudges, auto-aim) — power stayed 0.000 during this hand test. Cosmetic fix: removed the stale duplicate `Turret boot` telemetry key that was overwriting the CRT-aware line (dashboard showed "absolute (calibrated)" even when the CRT solve ran).

### 12.8 Session 5d — real drive ratio + the arrival-bounce fix

**Drive ratio confirmed (user, on the robot):** the 5203 19.2:1 (312 RPM) output shaft carries the **64T gear**, which directly meshes the 176T turret ring; both encoder pinions (31T → encoder A, 32T → encoder B) also mesh that 64T. So:
- `MOTOR_GEAR_RATIO = 176/64 = 2.75` (was the 1.0 TODO default, which made `Turret est max speed` claim 1872 deg/s).
- Real turret no-load at full power ≈ **680 deg/s**. The PWM aliasing ceiling at the observed ~53 ms loop is ~620 deg/s, so full power would sit just over the line: `MAX_POWER = 0.3 → 0.8` (~545 deg/s no-load, ~12% under the ceiling; the 0.3 cap had been sized against the phantom 1872 deg/s and made aiming needlessly slow). The encoder CRT ratios are unchanged — the 64T's count cancels out of them (64/32·176/64 = 176/32), which is why the crosscheck stayed valid all along.

**Arrival bounce (dpad nudge overshot and "bounced left and right" around the target) — two structural causes, both fixed:**
1. **kS zero-crossing chatter.** The old code added the full ±0.060 static-friction feedforward at *every* nonzero output — including each sign reversal while hunting the target. Each zero-crossing injected a ±2·kS = 0.12 power step, re-energizing the cycle forever. Fix: `KS_RAMP_DEG = 4.0` — full kS beyond 4° of error, fading linearly to 0 at the target (0 disables the taper).
2. **No settled-hold.** The PID kept driving inside the 2° position tolerance, where the only thing left to fight was the loop's own lag (~19 Hz loop + median-3 + velocity EMA) — the classic bounce. Fix: `IN_BAND_HOLD = true` with `SETTLE_VEL_DPS = 15` — inside `POSITION_TOLERANCE_DEG` AND slower than 15 deg/s the output goes to 0 and the 5203's BRAKE zero-power behavior holds; leave the band or speed up and the PID re-engages (the kS ramp supplies breakaway torque for the correction).

kP/kD unchanged (0.030 / 0.002) — the bounce was structural, not gain-stiffness. If a small residual wobble remains on the bench, tune live in Panels in this order: kD 0.002 → 0.004 (more damping), then kP 0.030 → 0.024 (softer approach); a fresh capture of a nudge gives the numbers to do it precisely.

### 12.9 Session 5d rev 2 — the "spins wildly fast" event (capture 20261023-012245) and the real PWM refresh ceiling

Event: boot (CRT solved cleanly at +88°), first dpad tap → turret spun wildly CW for several physical wraps; power-off. The capture shows power pinned at −0.800, the tracked heading thrashing around ~85° (±20° jumps), crosscheck errors ±30° latching `CRT_CROSSCHECK`, and the soft limits never firing (the tracked position never left ~85°) — while the turret physically wrapped. Two root causes stacked:

1. **The OctoQuad PWM width refreshes at ~10 Hz, not per loop.** Run-length analysis of BOTH channels: 13 of 14 motion runs are exactly 2 loops — one fresh reading every ~106 ms at the ~19 Hz loop. The real unwrappable ceiling is therefore **half an encoder-B wrap per FRESH reading** = 32.7° turret per ~106 ms ≈ **310°/s** (allowing loop drift to 120 ms → ~273°/s). The §12.8 ceiling (~620°/s) assumed per-loop refreshes and was wrong. `MAX_POWER = 0.8` (~545°/s) put every fresh step at ~0.99 encoder-B revs — past the wrap boundary — so every step folded to the wrong direction and the tracked position thrashed instead of tracking.
2. **The first dpad tap jogged from a stale default setpoint.** `NudgeDeg` jogged from `setPointPosition`, which after boot is the default **0** — while the turret sat at +88°. The tap aimed **92° away** at full cap: sustained saturated power plus cause 1 = endless CW spin (the loop never saw convergence because the tracked position never approached the target).

Fixes (session 5d rev 2):
- `MAX_POWER = 0.40` (~272°/s no-load — under the real ceiling with margin for loop drift).
- `NudgeDeg` now jogs from `getPosition()` when position mode is not engaged (a stale setpoint can never command a surprise move again).
- **Velocity measured over the true refresh interval**: the abs path now divides each fresh delta by the time since the raw width actually *changed* (was: by the single-loop dt → velocity was a 0↔2v square wave into kD — a large alternating kick at any speed). Between changes the last measurement is held; after `ABS_VELOCITY_STALE_S = 0.30` (~3 refreshes) with no width change it relaxes to 0 (stopped).
- **CRT re-solve at PLAY**: `rezeroAtStart()` re-runs the boot anchor (and clears the crosscheck latch) — wired into `JowByTeleOp.start()`, `AutoOpModeBase.start()` and `TurretPivotDebug.start()`. Answers "should we do the CRT on start?": YES — the turret can be hand-spun during the init→start gap with the tracker not running, and the CRT solve re-derives the absolute position from raw encoders in ~one read, so re-anchoring at start is free insurance.

Bench retest after this: boot anywhere, tap dpad once (≈4° move from the *actual* position, settles, `INPOSITION`, power 0), then auto-aim. The `CRT_CROSSCHECK` fault from this event stays latched until the next init/start (by design).

### 12.10 Advisory — motor built-in encoder for turret speed (no code change)

Question: would the turret motor's built-in encoder (goBILDA 5202/5203 series, 19.2:1, 312 RPM) buy more max turret speed, and at what resolution cost?

Spec (goBILDA spec sheet, user-confirmed): relative quadrature Hall encoder on the *motor* shaft — "Encoder Resolution 537.7 PPR at the Output Shaft", formula `(((1+(46/17)))*(1+(46/11))) * 28` = 19.2 × 28 = 537.6 countable events per gearbox-output rev (28 = 7 Ch-A cycles × 4 edges per motor-shaft rev; goBILDA rounds to 537.7). Read free via the hub's motor-encoder port (`getCurrentPosition()`), hardware counter integrates edges between loops — no aliasing at any speed we can reach.

Math:
- Turret counts: 537.6 × 2.75 (64T→176T) = **1478.4 counts/turret-rev = 0.244°/count** (vs PWM absolute 1024 µs × 5.5 = **0.064°/step** — ~3.8× coarser standalone, but both ≪ the 2.0° tolerance).
- Speed: removes the ~10 Hz PWM-refresh ceiling entirely (the ceiling was a property of the PWM *source*, not the motor). Ceiling becomes the motor: **~680°/s at full power** (~2.5× the current 272°/s cap), ~500–600°/s realistic under load.

Verdict: the motor encoder is only optimal **as a hybrid** — quadrature for per-loop deltas + Through-Bore PWM kept for boot zero / CRT / periodic absolute anchor (10 Hz crosscheck keeps quadrature drift and backlash honest). Standalone replacement is strictly worse (loses absolute position + CRT, coarser, sees gearbox+mesh backlash). The hybrid also makes the 62/64T pinion swap unnecessary (that swap only raised the PWM ceiling) and supplies an independent direction ground truth that closes the boot-guard's both-flips-wrong blind spot, plus a clean per-loop velocity for kD/kV. Implementation cost: fuse the two sources, widen the PWM plausibility gate for >36°/loop turret steps (current 0.5 enc-rev clamp would trip), and make the CRT crosscheck speed-aware. → Implemented in §12.11.

### 12.11 Session 5e — hybrid tracking implemented (motor quadrature + PWM absolute anchor)

User approved the hybrid. Design goals: lift the ~10 Hz PWM-refresh ceiling without mechanical changes, keep every absolute duty on the OctoQuad, and never let a wrong motor-encoder assumption become a runaway.

**Source split** (`TurretPivot.java`):
- **Motor built-in encoder (goBILDA 5202/5203: 28 counts/motor-shaft-rev × 19.2032 planetary = 537.7 PPR at output × 2.75 drive = ~1478.7 ticks/turret-rev = 0.2435°/tick)** — per-loop MOTION owner in ACTIVE mode: delta integrates into `unwrappedCounts` every loop and feeds `computeVelocity` (clean per-loop velocity replaces the refresh-interval estimate; no 0↔2v square wave into kD). The hub counts edges in hardware between loops — exact at any speed.
- **OctoQuad PWM absolutes (unchanged roles)** — boot CRT solve, `rezeroAtStart()`, the every-loop encoder-A crosscheck, and:
  - **At-rest hard anchor** (`anchorAtRest`): when motor-derived velocity ≤ `MOTOR_ENC_REST_ANCHOR_VEL_DPS` (8), the tracked position is hard-snapped to the wrap-resolved PWM reading every loop. At rest the PWM width is not stale (nothing moved since the chip latched it), so this is exact — it erases backlash + integration drift right before the shot.
  - **Moving monitor** (`anchorWhileMoving`): while moving, a fresh PWM reading is up to one ~106 ms refresh interval STALE — correcting with it would rewind real motion, so it is wrap-resolved (candidate = reading + k·full-rev nearest the motor-integrated prediction), recorded in "Turret anchor corr", and NOT applied. This wrap-resolution is what makes the PWM usable at any speed (the old half-wrap aliasing ceiling becomes irrelevant to *tracking*).

**Safety gates (the part that makes it trustworthy):**
1. **Watch mode before trust**: after boot/re-zero the hybrid starts in WATCH — the legacy PWM-only tracker owns integration at `PWM_ONLY_MAX_POWER` (0.40). The first powered motion ≥ `MOTOR_ENC_CONFIRM_PWM_DEG` (6°) must show the motor integrating the same direction (≥ `MOTOR_ENC_CONFIRM_MOT_DEG` = 3°) → ACTIVE. Wrong direction → `MOTOR_ENC_DIR` fault → OFF; no motion from the motor while the turret moved → `MOTOR_ENC` fault after 2 evidences → OFF.
2. **Runtime dead/direction checks in ACTIVE**: per fresh-interval, wrap-resolved PWM interval ≥ 5° with < 1° motor integration (dead) or opposite sign (polarity wrong) → 2 consecutive evidences latch the fault and demote to PWM-only, hard-anchoring to the absolute reading that won the argument. Read failures ≥ 3 → demote.
3. **Power cap follows the mode**: `effectiveMaxPower()` = `MAX_POWER` (0.50, staged) only in ACTIVE; otherwise `min(MAX_POWER, PWM_ONLY_MAX_POWER)` — a fallback can never resurrect the 5d aliasing hazard.
4. **Anchor sanity**: |correction| > `MOTOR_ENC_ANCHOR_REJECT_DEG` (25°, inside the ±32.7° wrap window) → rejected; 8 in a row → hard re-anchor (recovery, same philosophy as the legacy gate).
5. **Boot direction guard third vote**: `pollBootDirectionGuard` now also accumulates the motor encoder's turret-frame motion (`MOTOR_ENC_POLARITY`); B and A agreeing with each other while BOTH disagree with the motor (the both-flips-wrong blind spot) latches `B_DIR_MISMATCH` with a log naming both suspects. Telemetry line "Turret boot dir (motor)".

**Known limitations (accepted)**: mid-motion slip of a few degrees is invisible until the next at-rest anchor (timing staleness ±1 loop makes tighter thresholds unusable — and shots happen at rest); a motor-spins-but-turret-jammed failure defeats the stuck watchdog in ACTIVE mode (motor velocity looks alive) — the crosscheck/at-rest anchor still flag it; a sustained PWM outage in ACTIVE keeps tracking on the motor encoder alone (advisory, position frozen otherwise — chosen over braking).

**Constants added** (all Panels-tunable): `MOTOR_ENC_TICKS_PER_MOTOR_REV=28`, `MOTOR_PLANETARY_RATIO=3591/187`, `MOTOR_ENC_HYBRID=true`, `MOTOR_ENC_POLARITY=+1` (BENCH-VERIFY: +0.10 power jog must INCREASE "Turret mot-enc ticks"), `PWM_ONLY_MAX_POWER=0.40`, `MOTOR_ENC_ANCHOR_REJECT_DEG=25`, `MOTOR_ENC_REST_ANCHOR_VEL_DPS=8`, `MOTOR_ENC_CONFIRM_PWM_DEG=6`, `MOTOR_ENC_CONFIRM_MOT_DEG=3`, `MOTOR_ENC_DEAD_PWM_DEG=5`, `MOTOR_ENC_DEAD_MOT_DEG=1`, `MOTOR_ENC_STREAK_LIMIT=2`, `MOTOR_ENC_READ_FAIL_LIMIT=3`. Changed: `MAX_POWER 0.40 → 0.50` (staged; raise toward full power after the hybrid proves out). Telemetry: "Turret mot-enc" (ACTIVE/WATCH/OFF+reason), "Turret mot-enc ticks", "Turret anchor corr", "Turret boot dir (motor)". Faults: `MOTOR_ENC`, `MOTOR_ENC_DIR` (cleared by `clearFaults()` and `rezeroAtStart()`).

**Prerequisite check before bench**: the 5203's 4-pin JST-XH encoder cable must be seated in its hub motor-port header (it feeds `getCurrentPosition()` in RUN_WITHOUT_ENCODER). If it's unplugged, watch mode latches MOTOR_ENC after two evidences and the robot stays on the PWM-only tracker — which §12.12 shows is NOT safe at speed (strap the cable so it cannot pop out mid-match).

**Bench bring-up order (first hybrid validation)**:
1. Init anywhere; check "Turret mot-enc: WATCH…" and that ticks read; hand-spin ~20° at init (TurretPivotDebug) to exercise the three-way direction guard.
2. `MOTOR_ENC_POLARITY` check: +0.10 power jog — ticks must rise with CCW heading; flip to −1 if not (Panels, then re-init).
3. One dpad tap (~4°): still WATCH (below the 6° confirm threshold) — should behave exactly like 5d rev 2.
4. A larger move (e.g. GotoSafeAngle ±30°): log shows "hybrid ACTIVE: motor encoder confirmed"; then a dpad tap must jog ~4° from the actual position and settle.
5. Auto-aim (left trigger): converges and holds; expect max speeds ~340°/s now; at arrival the at-rest anchor correction ("Turret anchor corr") should read ~±0.1° (backlash-sized), not degrees.
6. Watch "Turret mot-enc" stay ACTIVE and no MOTOR_ENC faults; capture a Panels trace for the notes.
7. After it proves out: raise `MAX_POWER` in steps (0.65 → 0.8 → 1.0), re-checking arrival behavior each step.

### 12.12 Session 5e incident — capture 20261023-020255 ("CRT solved then erratic"): root cause + resolution

Event: boot clean — CRT SOLVED (diff 0.0041 enc-revs, 5 candidates), tracked heading −10.3°, hybrid in WATCH. At i=191 an auto-aim setpoint (−18.3°, then −22.3°) commanded −0.30 power and the turret thrashed violently to the end of the capture (power bang-bang ±0.400, tracked heading swinging ±55° around ~0°, i=198 latched `MOTOR_ENC`).

Analysis (scripted shortest-path replay of BOTH PWM channels; dashboard rows ~50 ms):
- **"Turret mot-enc ticks" read 0 for the entire capture** — the 5203's 4-pin JST-XH encoder cable was unseated. The hybrid could never leave WATCH (confirmation needs motor motion), so the legacy PWM-only fallback owned tracking at the 0.40 cap. Dead detection worked exactly as designed (2 evidences → `MOTOR_ENC` → OFF).
- **The physical motion was real; the tracked heading was fantasy.** Independent unwrapping of B and A (offsets back-solved from the capture: 776/1024 and ≈657/1024 — matching code defaults) agree to **±0.9° cumulative** through the whole thrash. Since A (31T) and B (32T) fold at different rates (a 1-rev fold error = 63.4° vs 65.5° of turret), coincidental folding on both is impossible — the reconstruction is the physical truth: the turret really swung from −10° to a net **−277°** in repeated ±20–26° per-fresh-step oscillations (peak ~250°/s, under the ~310°/s fold ceiling — no PWM aliasing) while the tracker believed it stayed within ±55°. Divergence peaked ≈120° mid-thrash and ≈273° at the end; the ±180° soft limits were blind because they act on tracked position.
- **Corruption mechanism (bounded at code level)**: the legacy filter medians the *wrapped* fractional revs in [0,1) — `median3(absHist0, absHist1, revs)` in `updateFromAbsolute` — which is not wrap-aware. PWM refreshes at ~10 Hz against the 42 ms loop (capture: 23.9 Hz), so a 3-loop median window often contains two fresh readings; during fast motion across the 0/1024 seam the median can emit the wrong branch, producing a wrong-sign shortest-path delta up to ±0.5 enc-rev (±32.7°) that passes the 0.5 gate *by construction* and integrates as fake motion. The thrash crossed the seam constantly (raw 953→125→318→27→735…); 16 >0.25-rev steps passed the gate and were integrated. Exact per-sample reconstruction is impossible (the dashboard samples ~50 ms, skipping some loops), but this mechanism class bounds the observed divergence — and the fake position drove the controller, whose ±0.4 bang-bang drove the real turret harder: a corruption↔thrash feedback loop.

Resolution (user, on the robot): **seating the motor-encoder cable** removed the failure mode — the turret moves the correct amount with little overshoot at normal speeds. In hybrid ACTIVE the motor encoder owns per-loop motion and the PWM only anchors, so the wrap-blind median no longer touches tracking. Remaining: high-speed wobble → §12.13.

**Open hardening for the PWM-only fallback** (it runs if a cable pops mid-match — this incident shows it is not safe at speed):
1. Wrap-aware median: unwrap each incoming reading against `lastAbsRevs` *before* the median-3 (median the unwrapped candidates or the deltas) — kills wrong-branch emission without losing the glitch defense.
2. Fold-guard speed cap in non-ACTIVE modes: clamp power when the last fresh interval implies > ~0.6× the fold ceiling (~185°/s).
3. At-rest snap in non-ACTIVE modes: hard-anchor to the fresh PWM reading after ~0.3 s at rest (bounds fallback divergence to the last motion episode).
4. Soft limits: consider a physical-evidence heading (A-derived) when tracked and physical disagree persistently.

### 12.13 Session 5e — full-speed wobble (hybrid ACTIVE): the D-path loop gain, and tuning order

With the motor encoder seated, tracking is hybrid-ACTIVE and the residual wobble at full staged power (`MAX_POWER` 0.50 ≈ ~340°/s no-load) is a control-loop property, not a tracking fault. The exact control law (`driveOutput`): `out = kP·error − kD·velocity + kI·∫error + kS·sign·ramp`, clamped to cap; the kV feedforward is **dormant** in position mode (`desiredVelocityDps = 0` there — the hook exists for a future profile).

**The suspect: derivative-on-measurement through the lagging velocity EMA.** `VELOCITY_EMA_ALPHA = 0.4` → reported velocity lags truth by ~1.5 loops (~60 ms at 24 Hz). The D path's loop gain ≈ kD × plant velocity gain = 0.002 × ~680 (deg/s per unit power) ≈ **1.36** — negative feedback with >unity gain and ~1.5 loops of delay oscillates (pumps energy at each reversal) instead of damping. At the old 0.40 cap the same gain existed, but the PWM-derived velocity was intrinsically sparse (~every 2–3 loops) and effectively low-passed, which hid the problem; the motor encoder's sharp per-loop velocity exposes it. Signature match: wobble grows with speed and is worst around reversals.

**Tuning order (Panels, one change at a time, capture each run):**
1. `kD` 0.002 → **0.001** (D loop gain → ≈0.68 < 1). Expect the wobble to collapse; arrival may feel slightly softer.
2. If light dither remains near arrival: `VELOCITY_EMA_ALPHA` 0.4 → **0.7–1.0** (less D delay; at speed the motor-encoder quantization of 0.24°/tick is negligible, and `IN_BAND_HOLD` masks low-speed noise).
3. If long moves now undershoot or feel sluggish: that is saturation + coast physics (no profiler), not gains — the structural fix is a trapezoidal setpoint profile driving `desiredVelocityDps` (FRC MotionMagic analog, kV hook already present). Prefer adding that before raising `MAX_POWER` past 0.5: at 0.5 the turret slews ~14°/loop, at 0.8 ~23°/loop, which worsens any residual overshoot.
4. Differential check: if the wobble correlates with "Turret anchor corr" spikes, it is the PWM anchor, not PID (moving-anchor corrections are recorded, not applied).
5. `kI` stays 0 unless there is a steady-state offset at hold; kS taper (§12.8) already handles zero-crossing bounce.

**Diagnostic capture 20261023-022708 (user gain experiment: kP=1e-5, kD=0, kI=0, kS=0.4, kV=0, EMA 0.7)** — proves the hybrid and exposes a controller quirk worth knowing:
- Hybrid verdict: **ACTIVE (pol +1)** the entire capture — the polarity bench-check passed itself on the first powered move; motor encoder counted every hop; **wrap/glitch events 0 for all 514 rows**; anchor corr ≈0 at rests, ≤ ±10.6° recorded during motion, 0 rejects; faults none. The tracking path is validated; the §12.11 bring-up items 1–2 are done.
- With kP≈0 the controller degenerated to FROZEN-or-BANG, and the mechanism is the kS gating arithmetic: kS is only added `if (|out| > 1e-4)` where out = kP·error, so engagement requires **|error| > 10°** at kP=1e-5. Captured exactly: 33 rows (~1.4 s) frozen at 4.8° error with power "−0.000" (below the gate — the turret will never approach on its own inside 10°); then a tap pushed the error past 10° and power snapped to exactly −0.400 until the position swept back inside 10°, where power zeroed and the 5203 BRAKE stopped it hard (−165 → −13 → −1°/s in ~200 ms) wherever it landed. Landing spots = engagement threshold + brake distance ≈ deterministic → the user's "jerks to predetermined points ~15° apart". Telemetry power takes only {−0.400, −0.000, +0.000, +0.400} — the binary signature.
- kS 0.4 vs 0.6 in this config: with kP≈0, kS IS the throttle (its real job is breakaway compensation at 0.060) — 0.6 = harder bang = overshoot wobble. Expected.
- DPAD nudges measured at 8° per step (some 4° steps interleaved); held dpad repeats the nudge every ~2–4 rows (~40–80°/s setpoint sprint).
- Restore real gains: kP 0.030, kD 0.001, kI 0, kS 0.060, kV 0, EMA 0.7 — at kP=0.030 the kS gate engages from |error| > 0.0033°, so the approach is proportional all the way and this artifact vanishes.

### 12.14 Session 5f — kS engagement gate decoupled from kP (the "frozen below 10°" fix)

The §12.13 diagnostic capture exposed a real coupling flaw: kS was gated on the *magnitude of the PID output* (`|out| > 1e-4`, evaluated before kS was added), so the effective engagement threshold was `1e-4 / kP` degrees and silently scaled with kP — 10° at the kP=1e-5 experiment, 0.0033° at the real kP=0.030. Any future downward kP retune would have silently disabled breakaway compensation.

Change (`driveOutput` + new constant, compiled `BUILD SUCCESSFUL`):
- New Panels-tunable **`KS_ENGAGE_DEG = 0.25`**: kS engages whenever `|error| > 0.25°`; direction comes from the PID output, falling back to the sign of the error if the output is exactly 0 (degenerate all-zero-gains case).
- The `KS_RAMP_DEG` taper still fades kS to 0 at the target, so no arrival chatter is reintroduced; with normal gains the behavior is identical to before (the old gate passed from |error| > 0.0033°, far inside the epsilon).

Answering the standing question "is the 10° gate needed for the PID terms to work together": **no** — it was purely the kP=1e-5 artifact. What IS worth keeping is an engagement epsilon near zero error (anti-chatter at the target), which `KS_ENGAGE_DEG` now provides, decoupled from gain scale.

### 12.15 Session 5f bench — captures 20261023-024811 / 025055: weakpoint ranking after the gate fix

**024811 (real gains restored: kP 0.030, kD 0.001, kS 0.060, EMA 0.7):** kS-gate fix verified live (telemetry power now takes many intermediate values, not just ±0.400/0). Hybrid WATCH→ACTIVE (pol +1 confirmed by powered motion), faults none, wrap/glitch 0–1, at-rest anchor snaps ≤ ~1°, moving-monitor corr ±2–7° recorded-not-applied. Long saturated sprints behave (246° sweep at ~300°/s, ~2° arrival overshoot, clean brake). The "jitter": **71 rapid power sign flips**, clustered in dpad-hold sequences — per 8° tap the turret overshoots 1.5–3°, reverses, and seesaws ±3–5° at ±20–60°/s for 0.3–0.5 s before IN_BAND_HOLD catches it; state flaps MOVINGTOPOSITION↔INPOSITION every few rows.

**025055 (auto-aim):** two distinct problems.
1. **Aim-setpoint thrash** (i=480–522): setpoint alternates +15..+33 ↔ −43..−5 about every 1–2 rows → turret bangs ±0.500 chasing it, physically swinging ±15° at ±180°/s. **Initial "selection flips" diagnosis was WRONG** — the raw JSON proves `Aim target = RedFront` on every aim loop, robot X/Y static, heading wobbling only ±0.6°. The true cause is the chassis-yaw feedforward units bug in `AimAtGoal` — see §12.16 (bearing + yaw = constant −2.8° all segment; every degree of setpoint motion was the ff term).
2. **Loop limit cycle around a static target** (i=596–610): aim setpoint steady at −5.21° and the turret still rocks −0.8 ↔ −10.5 (±5°, ~0.4 s period) — the 30° approach ran at ~137°/s into an 11.1° first-arrival overshoot (i=592–594), then P reversal + weak kD + backlash dither. Braking is reactive (only after crossing).

**`CRT_CROSSCHECK` latched at i=252** — spurious: encoder A's ~10 Hz PWM is stale during the dither (±25–65°/s), so it disagreed ±3–4° with the fresh motor-integrated position for 5 consecutive checks → latch (persists until re-zero by design). The crosscheck needs speed-awareness (known §12.10 item).

**Weakpoint ranking (the chain: setpoint → PID → cap 0.5 → 5203 → 2.75:1 → 64T→176T mesh → ring):**
1. **Aim ff units bug (§12.16)** — the auto-aim setpoint itself thrashed; fixed session 5g.
2. **No motion profile.** The position loop commands speed ∝ error up to the 0.5 cap, so every arrival happens at full commanded speed; braking (P reversal + kD 0.001) only acts *after* crossing. Explains the low-speed jitter and the i=596–610 limit cycle. Profile-lite cap added session 5g (fix A).
3. **Backlash + stiction** in the 64T→176T mesh converts each overshoot into a rocking limit cycle; the kS kick at each reversal re-punches it. Physical, but never excited if arrivals are slow.
4. Crosscheck not speed-aware (cosmetic latch noise) — fixed session 5g (fix D).
5. Aim-point selection without hysteresis — selection was proven stable while stationary; hysteresis added anyway as driving hardening (fix B).

**Not the weakpoint:** hybrid tracking — clean everywhere in both captures (ACTIVE, zero wrap/glitch events, fresh per-loop position, sane anchors). The feedback chain below the controller is now the strongest part; the controller's *shaping* of the command is what's missing.

**Proposed fixes:** (A) error-scaled power cap in `driveOutput` — `profCap = clamp(kProfile·|error|, min, effectiveMaxPower())` — the profile-lite that makes approach speed ∝ remaining error (trapezoid via `desiredVelocityDps`/kV later); (B) aim-point hysteresis (stick to current point unless another is better by a margin); (C) after A, raise kP for small-error stiffness (0.030 → 0.04) since saturation-driven overshoot disappears; (D) speed-aware crosscheck tolerance.

### 12.16 Session 5g — the auto-aim thrash root cause: chassis-yaw feedforward units bug (deg/s added as deg)

The user challenged the §12.15 "selection flips" diagnosis ("only 4 aim points, we were stationary, vastly closer to 1 hive") — and the raw capture proved them right. `AimAtGoal.execute()` computed:

```java
yawRateDps   = Math.toDegrees(dHeading / 0.02);            // deg/s estimate
double ffDeg = |yawRateDps| < 2 ? 0 : -yawRateDps * 1.0;   // YAW_FF_SCALE = 1.0
turretSetpointDeg = desiredBearingDeg + ffDeg;             // VELOCITY added as POSITION
```

The bearing term already tracks the chassis 1:1 (`bearing = fieldBearing − heading`); the ff's job is only a ONE-LOOP lead, i.e. `−yawRate·dt` ≈ −0.9° at 44 deg/s. Adding `−yawRate·1.0` instead amplified every per-loop heading change by 1/dt ≈ 25–50×. The Pinpoint heading jitters ±0.5–0.9° per loop under turret-slam vibration (and the turret's own reaction torque rocks the chassis), so the ff injected ±25–45° of setpoint noise per loop — the turret slammed, the slam shook the heading, the ff flipped sign: a self-excited loop through the ff path.

**Proof from capture 20261023-025055:** across i=470–525, `Aim target` = RedFront (58.0, 56.5) on every row (selection NEVER flipped), robot X ≈ 55.3–55.4 static, heading only 89.0–90.2°, `Aim chassis yaw` alternating ±3..±44 dps, and `Aim turret set + Aim chassis yaw` = **−2.5..−3.3 constant** — the true bearing was −2.8° and 100% of the ±30–60° setpoint motion was the ff term. Segment i=589–618 shows the same law at small scale (yaw +3.0 → set −5.8; yaw −3.3 → set +0.5; B ≈ −2.8).

**Session 5g changes (all compiled `BUILD SUCCESSFUL`):**
1. `AimAtGoal` (root cause): ff is now `−yawRateDps · loopDtSec · YAW_FF_SCALE` (deg, one loop of lead; `YAW_FF_SCALE` redefined as "loops of lead"); the loop period is MEASURED (`lastExecNs`, clamped 2–200 ms) instead of hard-coded 0.02 s; the yaw rate is EMA-smoothed (`YAW_RATE_EMA_ALPHA = 0.3`); `YAW_FF_DEADBAND_DPS` 2 → 5; `GotoSafeAngle` velocity arg now passes the filtered true rate (kV hook stays inert in position mode). New telemetry: `Aim dt (ms)`.
2. `AimAtGoal` fix B: aim hysteresis `AIM_SWITCH_MARGIN_IN = 6.0` — keep the locked point unless the challenger is >6 in closer, within the same candidate set (alliance vs fallback).
3. `TurretPivot` fix A: profile-lite cap in `driveOutput` — `PROFILE_LITE_ENABLED`, `PROFILE_GAIN_PER_DEG = 0.020`, `PROFILE_MIN_POWER = 0.060` (= kS, so breakaway survives); cap = min(profCap, edge/mode cap), applied to the final output including kS.
4. `TurretPivot` fix D: crosscheck tolerance is now speed-aware — `tol = CRT_CROSSCHECK_TOL_DEG + CRT_CROSSCHECK_STALE_S·|vel|` (`CRT_CROSSCHECK_STALE_S = 0.10` s stale-age model matching the ~10 Hz refresh), full cutoff moved `CRT_CHECK_MAX_SPEED_DPS` 20 → 120 deg/s; the i=252-style spurious latch at ±65°/s (expected stale disagreement ±6.5° vs old fixed 3°) now passes; a persistent slip still latches.
5. Fix C deliberately NOT applied yet: kP stays 0.030 until a capture proves the shaped approach; it is a one-value Panels change when ready.

**Bench expectations for the next auto-aim capture:** `Aim turret set` should sit within ~±1° of −2.8° (plus real lead only while the chassis actually turns); `Aim chassis yaw` should read ~0 at rest (EMA + deadband); approach swings capped at 0.02·|err| (e.g. ≤0.10 by 5° out); no `CRT_CROSSCHECK` latch during dither.

### 12.17 Session 5g bench — captures 20261023-030728 / 031455: profile-lite + aim fixes validated; residual stall fixed by Panels values; wrap seam behavior confirmed intended

**030728 (first run of the 5g build, PROFILE_MIN_POWER 0.060, kP 0.030):** every §12.16 expectation met. Three aim episodes (62.6° / 152.6° / 141.8° approaches) locked in 0.6–0.95 s each, final errors 0.86–2.73°, max arrival overshoot 1.6–2.2° (from ~260°/s crossings), zero profile-cap violations, aim setpoint span ~±0.2°, `Aim chassis yaw` ≤ 1 dps, 15 power sign flips total (vs 71 in 024811), wrap/glitch 0, faults none, crosscheck max 2.38°. `Aim dt (ms)` median 49 — the real command loop is ~50 ms, so the old hard-coded 0.02 s made the ff ~2.5× worse on top of the units bug (~50× total). One artifact: episode 1 settled 2.73° past target and STALLED pushing `−0.060` — `PROFILE_MIN_POWER = 0.060` equals kS and this mesh's static friction from rest needs slightly more.

**031455 (user set PROFILE_MIN_POWER 0.10 + kP 0.04 via Panels):** stall eliminated — err 2.34° pushed through at `+0.097` (the floor) to 0.39°. Three aim episodes (109.2° / 79.3° / 107.3° approaches): locked in 0.4–0.8 s, final errors 0.45° / 1.48° / 0.31°, max overshoot 2.3–5.7° from ~270°/s crossings, aim setpoint span 0.20° over 99 aim rows, yaw ≤ 0.9 dps, deactivations hold at 0 power. Cap exactness visible in the data (pw −0.423 at err 21.16 = 0.02·err; nudge cruises at cap ~0.16–0.22). 8° dpad taps: 1-row overshoot bounces, immediate settle. Health: faults none, wrap/glitch 0, crosscheck max 6.47° only during the boot WATCH→ACTIVE transition at speed (covered by the speed-aware tol; 2 consecutive rows marginally above it, far under the 5-fail latch), max 2.27° at rest. Anchor corr ≤ ±16.9° recorded-not-applied during 300°/s slews (0 rej).

**Wrap-seam behavior (rows 146–194) — confirmed INTENDED by the user.** Nudging across the ±180 seam twin-flips the setpoint representation (+176.29), so the next jog is >180° of naive error and the turret takes the long way at full power — which is the only path that stays inside the ±180 soft limits, i.e. exactly the FRC chooseTurretSetpointDeg "turn back around" behavior. The user confirms ±180 travel is the DESIGNED policy (not a limitation to remove) and deliberately uses setpoint edits across the seam to regression-test the wrap logic. Do not "fix" this; the nudge cap shaping (0.02·|err|, floor 0.10) already bounds these test sweeps.

### 12.18 Session 5g — modular refactor: turret package, CRT extracted, servo + hub fallback + legacy zero removed

User directives: servo code gone (5203 settled), only the hybrid mode + a way back to encoder-B-only tracking kept, CRT math in its own class, OctoQuadAbsoluteSource out of the TurretPivot file, all turret items in a turret package, and the pre-CRT zero constants removed if dead.

**New layout — `org.firstinspires.ftc.teamcode.Subsystems.turret`:**
- `TurretPivot.java` — the subsystem (PID, hybrid state machine, boot orchestration, telemetry). FRC names kept.
- `TurretCrt.java` — ALL CRT math, static and stateless: `solve()` (the EasyCRT boot solve, returns a `SolveResult`), `solveFrameRevs` (RAW-frame offset-then-flip), `crosscheckDiffDeg`, `mod1/fracDiff`, plus every CRT constant (zero offsets, tolerance, crosscheck tol/stale-s/cutoff). @Configurable.
- `OctoQuadAbsoluteSource.java` — the OctoQuad PWM source (was nested in TurretPivot), with its channel/bank/PWM-window constants and `ENCODER_REVERSED` (the encoder-B flip).
- `AbsoluteAngleSource.java` — the small source interface (readRevs / readCrossRevs / describe).
- `TurretPivotDebug.java` — the bench OpMode, moved from TeleOp (hand-spin verifier now reads degrees directly).

**Removed (all verified dead before deletion):**
1. Servo drive: `DriveStage` interface, `ServoDriveStage`, `SERVO_REVERSED`, `SERVO_DEADBAND`, `PREFER_MOTOR_DRIVE`, `IronConstants.TurretServoName`. The 5203 motor is now held directly (the old `MotorDriveStage` behavior inlined: RUN_WITHOUT_ENCODER, BRAKE hold, current alert, change-gated direction writes).
2. Hub-quadrature fallback: `countsPort` / "turret_encoder" dummy-motor path (`updateFromHubCounts`, `signedRawCounts`, `checkAbsoluteConsistency`, `absVsCountsPhaseOffset`, `getRawCounts`, the counts branch of the boot check) and `IronConstants.TurretEncoderName`. The fallback story is now exactly what the user asked for: hybrid ACTIVE → (any fault or `MOTOR_ENC_HYBRID=false`) → encoder-B PWM-only tracking with `PWM_ONLY_MAX_POWER`.
3. Pre-CRT zero: `ENCODER_ZERO_OFFSET_DEG`, `ABS_ZERO_CALIBRATED`, `faultZeroMismatch`, and bootZero's single-encoder branch — dead since the CRT solve took over (it only ran when CRT wasn't even attempted). `reZeroAtForward` now suggests both RAW CRT offsets instead. `FORWARD_BOOT_WINDOW_DEG` went with it.
4. The internal "counts" layer: `ENCODER_COUNTS_PER_REV` and the `unwrappedCounts`/`degPerCount` machinery — the tracked stream is now plain TURRET DEGREES (`unwrappedDeg`), which every conversion site uses directly (×360/R_B for encoder-B revs, motor ticks already converted). Behavior-identical by construction; `getUnwrappedCounts()` → `getUnwrappedDeg()`.
5. Dead constants `MANUAL_SPIN_MAX_DPS` / `MANUAL_SPIN_DEADBAND` (never referenced).

**Kept identical:** every live telemetry key (minus the four dead counts/zero lines), the control law, all watchdogs, the boot direction guard (three votes), watch/active promotion and demotion, wrap-aware setpoint choice.

**⚠️ PANELS PERSISTENCE: moving TurretPivot to a new package changes its Panels storage key, so persisted @Configurable values reset to compiled defaults on first flash.** The user's tuned values were therefore BAKED into the compiled defaults: kP = 0.040, PROFILE_MIN_POWER = 0.100, VELOCITY_EMA_ALPHA = 0.7, kV = 0.000, CRT_CHECK_MAX_SPEED_DPS = 120 (now in TurretCrt). After flashing, glance at the Panels Config page to confirm these read correctly; CRT zero offsets (657/1024, 776/1024) moved to TurretCrt with correct defaults.

### 12.19 Session 5g — refactor slice 2: `TurretTracker` (sense vs. act split)

User ask: "the turret pivot still feels heavy, much of the turret class is about encoder and position management. if that is the next biggest slice we should move that into a file for turret position management." — improvement on the idea: the moved half is not passive "position management" but a *tracker* — it fuses three encoders into one position estimate. Naming it `TurretTracker` makes the split read as sense-vs-act: **the tracker figures out WHERE the turret is; the pivot decides what to DO.**

**New file `TurretTracker.java` (~1100 lines) — the SENSING half.** Everything that used to answer "where are we" moved here:
- the tracked stream (`unwrappedDeg` / `zeroOffsetDeg`), velocity EMA, the median-of-3 + implausibility gate;
- the full hybrid state machine (OFF / WATCH / ACTIVE) with every streak counter, promotion, demotion and anchor (at-rest hard snap + wrap-resolved moving monitor);
- the every-loop encoder-A cross-check (speed-aware tolerance);
- boot verification (pollBootEncoderCheck, the 3-vote boot direction guard incl. the motor vote, the ABS stability check, `anchorBoot` / `zeroHere`);
- ALL of its tuning constants: `ABS_MEDIAN_WINDOW`, `ABS_MAX_PLAUSIBLE_DPS`, `ABS_REJECT_REANCHOR`, `ABS_BOOT_READS`, `ABS_BOOT_SPREAD_DEG`, `VELOCITY_EMA_ALPHA`, `ABS_VELOCITY_STALE_S`, `MOTOR_ENC_TICKS_PER_MOTOR_REV`, `MOTOR_PLANETARY_RATIO`, `MOTOR_ENC_HYBRID`, `MOTOR_ENC_POLARITY`, `MOTOR_ENC_ANCHOR_REJECT_DEG`, `MOTOR_ENC_REST_ANCHOR_VEL_DPS`, `MOTOR_ENC_CONFIRM_*`, `MOTOR_ENC_DEAD_*`, `MOTOR_ENC_STREAK_LIMIT`, `MOTOR_ENC_READ_FAIL_LIMIT`, `FEEDBACK_FAIL_LIMIT`.
- It NEVER drives: BRAKE-on-feedback-loss, power caps and all reactions stay in the pivot.

**`TurretPivot.java` shrunk 1990 → ~994 lines — the ACTING half:** constructor wiring (builds the tracker with the three ratios), bootZero CRT orchestration + logging, `rezeroAtStart` / `reZeroAtForward`, setpoint management (GotoSafeAngle / Nudge / chooseTurretSetpointDeg wrap logic), the PID → power stage (kP/kI/kD/kS ramp/profile-lite/edge zone/soft limits/settled-hold), stuck watchdog, over-current monitor, BRAKE-on-feedback-loss decision, telemetry. Reads the tracker via `position()/velocityDegPerSec()/hybridActive()/feedbackLost()/modeText()` etc. **Every telemetry key name is unchanged.**

**Behavior parity notes (deliberate, not drift):**
- `TurretPivot.clearFaults()` clears its own four faults + `tracker.clearFaults()` (crosscheck, MOTOR_ENC*, glitch/failure counters, re-arm). The boot-wiring faults (`ENCODER_DEAD_AT_BOOT`, `B_DIR_MISMATCH`) are NOT clearable — re-zeroing can't fix wiring (same as before).
- `rezeroAtStart()` now calls `tracker.resetForRezero()`: clears the crosscheck latch + hybrid faults (a fresh absolute anchor re-arms the hybrid) but keeps the session diagnostic counters — exactly what the old inline clearing did.
- `effectiveMaxPower()` = `tracker.hybridActive() ? MAX_POWER : min(MAX_POWER, PWM_ONLY_MAX_POWER)` (was the mode enum; same semantics).
- The boot log line prints `tracker.modeText()` instead of the raw enum+reason (same information).

**⚠️ PANELS PERSISTENCE (again):** the moved constants' @Configurable keys moved from `TurretPivot` to `TurretTracker`, so any *persisted* edits to those specific values reset to defaults on next flash. Defaults already equal the tuned values (VELOCITY_EMA_ALPHA 0.7 etc.); re-check the Config page once after flashing if any ABS_*/MOTOR_ENC_* value had been hand-tuned in Panels.

**Bench re-verify after flash (internal stream unchanged, but confirm):** boot log shows the CRT solve, one nudge responds, one aim hold locks in; `Turret mot-enc` still reaches ACTIVE after ~6° of motion.

### 12.20 Session 5g — BUGFIX: tracked stream ran BACKWARDS (capture 20261023-034539) — fracDiff argument-order regression from the §12.18/§12.19 extraction

**Symptom:** first bench run of the refactored build. CRT boot solves correct at init (and again at a second init after moving), but between them the live tracking is REVERSED — hand-CCW shows CW on the field visual. Capture 034539 shows, in BOTH sessions: WATCH latched `MOTOR_ENC_DIR` ("direction mismatch — check MOTOR_ENC_POLARITY") at the first motion and demoted to PWM-only, and `CRT_CROSSCHECK` latched with the error sweeping continuously through A-encoder wraps at ≈ **2× the measured velocity** — the exact signature of the tracked stream running OPPOSITE to encoder A (at rest they agreed to 0.15°, because the boot ANCHOR was correct).

**Root cause:** the pre-refactor TurretPivot had TWO wrap-difference helpers with OPPOSITE argument orders — `angleDiff1(a, b) = b − a` (used for motion accumulation: the per-loop PWM delta and the boot-guard B sum) and `fracDiff(x, y) = x − y` (used for measured-minus-predicted comparisons). The §12.18 extraction collapsed both into `TurretCrt.fracDiff(x, y) = x − y` and translated `angleDiff1(lastAbsRevs, revs)` → `fracDiff(lastAbsRevs, revs)` = **last − revs = the NEGATED delta**. Every per-loop PWM step was integrated backwards; the boot solve/anchor (which never touch that helper) stayed correct — hence "CRT solves correctly, tracking reversed". Proof from the capture: heading deltas matched the RAW (unflipped) B-frame per-step (+2.75 ≈ +2.57 etc.) while the boot angles reproduced z_B = 776 under flip-true to 0.06°; a mirrored module or a flipped ENCODER_REVERSED would have broken the CRT solve instead.

**Fixed (2 sites, both motion accumulations):**
1. `TurretTracker.updateFromAbsolute`: `fracDiff(lastAbsRevs, revs)` → `TurretCrt.forwardDiffRevs(lastAbsRevs, revs)` (= revs − last, the forward motion).
2. `TurretTracker.pollBootDirectionGuard`: B-sum `fracDiff(bootDirLastB, b)` → `forwardDiffRevs(bootDirLastB, b)` (was negated — would false-latch B_DIR_MISMATCH on real hand motion); A-sum rewritten to the same `forwardDiffRevs(bootDirLastA, a)` form (value identical to before).

**Prevention:** new `TurretCrt.forwardDiffRevs(from, to)` = "to − from, seam-safe" for motion accumulation; `fracDiff` javadoc now states the x−y convention and warns that swapping them reverses the stream. Remaining `fracDiff` uses are all measured-minus-predicted (solve's is abs()) — audited.

**The safety nets worked as designed:** MOTOR_ENC_DIR and CRT_CROSSCHECK both caught the reversed stream within seconds. After re-flashing this fix, re-init: both latches should stay clear, `Turret mot-enc` should reach ACTIVE after ~6° of motion, hand-CCW should read CCW, crosscheck should stay ≤ ~3° while moving.

### 12.21 Session 5g bench — capture 20261023-035831: direction fix validated, FIRST full-field run of the refactored build (drive + fire hold)

816 rows, one session. **Zero faults the whole capture** (`Turret OK` true throughout): no MOTOR_ENC_DIR, no CRT_CROSSCHECK, wrap/glitch 0, abs/CRT OOR+CRC 0. CRT boot solve diff 0.0021 (6 candidates).

- **Direction fixed:** heading↔motor-tick sign agreement 82:1 (the 1 = a sample edge); heading vs the PRE-flip raw display runs opposite-sign 84:2, which is CORRECT for ENCODER_REVERSED=true. User confirms the field visual matches reality and hand-CCW reads CCW.
- **Hybrid:** ACTIVE at capture start and never demoted — the runtime direction confirmation (WATCH→ACTIVE) is the strongest evidence the stream and the motor encoder now agree.
- **Crosscheck:** max 2.37° (at 76 deg/s) and **0 accumulated fails all capture** — even at 287 deg/s slews it read ±0.2–0.7°.
- **Aim under fire + driving:** aim err settles to ±0.23° once locked (approach 27.2 → 8.9 → lock), on-target cycled repeatedly while the chassis moved (yaw ff handled spins to ±275 deg/s), setpoints spanned 92° (−31.8..+60.2), turret power stayed within ±0.21, velocity max 287 deg/s, motor current max 1.26 A, `Aim dt` ≈ 49–54 ms.
- **Anchors:** at-rest corrections ±0.00° (no integration drift); while moving max 14.35° recorded-not-applied (PWM staleness at speed, 0 rejects).
- Boot direction guard stayed "pending" (no 15° hand-move this run) — unexercised, not a concern: its B-sum sign bug was fixed in §12.20 in the same build.

Verdict: the refactored turret package (TurretPivot/TurretTracker/TurretCrt/OctoQuadAbsoluteSource) is field-validated. Remaining queue: §12.11 staged MAX_POWER raise (0.50 → 0.65 → 0.8 → 1.0).

### 12.22 Session 5g bench — capture 20261023-041229: MAX_POWER raised 0.50 → 1.00 (intermediate stages skipped at user request), validated

1268 rows including an init. **Zero faults all capture** (`Turret OK` true): no MOTOR_ENC_DIR/CRT_CROSSCHECK, OOR/CRC 0 both channels, CRT solve diff 0.0017 (6 candidates). Wrap/glitch counter 1 benign event (a >0.25-enc-rev PWM step counted during the WATCH confirming motion at ~264 deg/s — telemetry only).

- **Performance:** measured turret velocity up to **639 deg/s** (was 287 at MAX_POWER 0.5; ~680 deg/s is the 312-RPM no-load estimate) and commanded power actually reached 1.000. Motor current max 4.36 A (alert 20) — the mesh moves freely at full power.
- **Long wraps at speed:** the continuous stream swept through the ±180 seam at ±557–639 deg/s with the crosscheck holding ±0.4–1.8° and zero jumps — the wrap-aware stream + hybrid anchor handled full-power seam crossings without event.
- **Direction:** heading↔motor-tick sign agreement 71:0.
- **Arrivals at full power:** two measurable stable-setpoint arrivals — a 96.8° move overshooting ≤6.9° and a 90.1° move at 1.15°, no bounce/limit-cycle. The profile cap shaped the approach (0.65 commanded at ~32° error = 0.02/deg as designed); kD (0.001 × ~600 deg/s ≈ 0.6) did the braking.
- **One transient worth knowing:** at the WATCH→ACTIVE handoff the crosscheck read +14.32° for ~2–3 loops (2 of 5 fails, no latch, self-cleared ~100 ms): the first powered motion accelerated so fast that encoder A's PWM refreshed before encoder B's, so A briefly compared against B's not-yet-updated position. Same class as the 031455 boot transition; the 5-fail latch + speed cutoff contained it by design. Anchor corr while moving peaked at 22.09° recorded-not-applied (under the 25° reject gate, 0 rejects).
- Aim-under-fire: 556 at-speed rows, median aim err 0.89°, p95 1.68° (max 149° was a mid-spin transient before re-lock).

Verdict: MAX_POWER = 1.0 validated on this mechanism; keep 1.0 unless a future capture shows current/thermal or control problems.

### 12.23 Session 5h — driver-station failure board (HardwareHealth): hardware faults surface on the DS, detail stays on Panels

**Requirement (user):** "if there is an encoder failure on the turret, either CRT or the motor encoder then it should post to the control hub dashboard as well. in fact we should have any hardware failures show on the drivers hub. such as the octoquad or a motor encoder not showing/coming alive. or the limelight not giving pictures or status etc. the general idea is that the drivers hub also gets a warning of failure, because on gameday im betting the driver will only have this and will not have the panels dashboard. though we will have the laptop in the pits for debugging after a match, so only very critical information needs to be in the drivers hub."

**Design — one aggregator, two channels, a deliberately short critical list.** New `HardwareHealth` (teamcode root package, singleton via `get()`, all calls on the RC loop thread, state reset at every `RobotMain` construction):

1. **Red DS banner** — `RobotLog.setGlobalErrorMsg(...)` / `clearGlobalErrorMsg()` (verified by javap on RobotCore-12.0.0: `com.qualcomm.robotcore.util.RobotLog` — note `getGlobalWarningMessage()` returns a `GlobalWarningMessage` OBJECT; the string API is `getGlobalErrorMsg()`). Edge-triggered: set only on change, cleared once healthy — no per-tick log spam. Every message we write starts with `"IRON HW:"`; before set/clear we read the current global error and NEVER touch a message that isn't ours (the SDK posts its own init-failure/I2C errors there — clobbering them would hide real problems). We also do NOT use `addGlobalWarningMessage`/`registerGlobalWarningSource` (those would fight the SDK's own warning sources). The banner intentionally survives OpMode stop() so the driver walks back still seeing what broke; the next OpMode's `reset()` clears our own.
2. **DS telemetry line** — `"HW": "OK"` or the first fault + `(+N more)`. Ownership rule learned the hard way: `telemetry.update()` CLEARS the buffer and the DS shows only the last update, and `AutoOpModeBase.UpdateTelemetry()` already updates `opmode.telemetry` per tick (alliance/X/Y/Heading) — so the board itself never calls `update()`. Each OpMode's `UpdateTelemetry()` pulls `HardwareHealth.get().dsLine()` into its own flow (JowByTeleOp + AutoOpModeBase done; FusionTune is Panels-only and gets the banner anyway). JowByTeleOp's DS screen now actually lives during a match (previously it was one static init frame).
3. **Panels summary** — one `"HW health"` line in `flushTelemetry()` (added before `DashTelemetry.update()`), so the pit laptop sees the same board plus the full per-subsystem detail around it.

**Who reports what (the critical set):**
- **TURRET** — `TurretPivot.periodic()` reports `faultString()` every loop (`null` = healthy heartbeat): ENCODER_DEAD_AT_BOOT, STUCK, OUT_OF_SCOPE, OVER_CURRENT, ABS_BOOT_UNSTABLE, CRT_CROSSCHECK, B_DIR_MISMATCH, MOTOR_ENC, MOTOR_ENC_DIR, and the new ABS_STREAM_LOST. `tryCreate`'s catch reports `"absent at init (<exception>)"` — a missing turret/OctoQuad on the comp bot is a match-critical condition the driver sees BEFORE play.
- **LIMELIGHT** — new `Vision.healthPing()`, called from BOTH `RobotRunPeriodic()` and `InitRunPeriodic()` (independent of the VisionFusion gates: `!ENABLED`, no-fusion-localizer, and `skip-fused-diverged` all return BEFORE `vision.poll()`, so a heartbeat inside poll() would starve and false-alarm). Liveness signal = frame staleness: any frame ≤ `Vision.LIMELIGHT_FRESH_MAX_MS` (1000 ms) proves the camera publishes (even one with no tags — "no solve" is gameplay, not hardware failure, and never alarms); a dead/unplugged LL keeps serving its LAST result forever, so staleness growing past 1 s IS the death signal; a null result reports nothing and the heartbeat goes silent instead. Constant chosen above MT-pipeline FPS (staleness normally < 300 ms).

**Heartbeat/staleness semantics:** `expect(name)` at RobotMain construction marks a system REQUIRED (TURRET when created, LIMELIGHT when `vision != null`; a comp-bot-absent LL/turret is a direct fault instead — bench rigs silence both via `IronConstants.CompBot`). A required system with no `ok()` for `HEARTBEAT_STALE_MS` (2000 ms) while RUNNING flags `"no data Xs"`. Silence windows count ONLY after PLAY and restart at the PLAY edge — init-time reporting is sporadic by design (the turret's scheduler only runs post-PLAY), so a long init can never false-alarm. The PLAY signal is `RobotMain.runPeriodicSeen`, set by `RobotRunPeriodic()` (which is ONLY called from OpMode `loop()` — `OpMode` has no public isStarted(); the flag lives package-private on `OpModeInternal`, so a reflection hack was rejected in favor of this call-path fact, which AutoOpModeBase/FusionTune/JowByTeleOp all satisfy). Direct faults (e.g. "absent at init") show immediately, even during init.

**New tracker fault — ABS_STREAM_LOST (fills a real gap):** `updateFromAbsolute()` returns false ONLY when the OctoQuad read is empty (dead device / unplugged ABS wire). In hybrid ACTIVE the motor encoder keeps tracking, so this was NOT `feedbackLost()` (no brake, no latch) — the turret would keep aiming on a dead absolute path with no driver-visible trace. Now `TurretTracker` counts consecutive empty-read loops (`ABS_STREAM_LOST_LOOPS = 50`, ≈ 2.5–3 s at the measured 49–54 ms loop) and latches `faultAbsStreamLost` (getter `isAbsStreamLost()`, in `faultFragment()` + `TurretPivot.faultString()`), auto-clearing after 50 consecutive GOOD reads (a re-seated wire recovers without an OpMode restart; `clearFaults()` also clears it). Glitch-gate rejections do NOT count (the stream is alive; those stay in `absGlitchEvents`). Motor-encoder death was already covered (MOTOR_ENC / MOTOR_ENC_DIR / STUCK latches).

**Compile:** `:TeamCode:compileDebugJavaWithJavac` green. **Not yet field-verified.** Bench checklist for next session: (1) on init with everything healthy the DS shows `HW: OK` and NO red banner; (2) unplug the Limelight → within ~2–3 s of PLAY: `HW: LIMELIGHT no data Xs` + red banner, Panels `HW health` matches; reconnect → both clear automatically; (3) unplug encoder B's ABS wire (or the OctoQuad) → ABS_STREAM_LOST latches in `Turret faults` and reaches the DS banner; re-plug → auto-clears after ~50 good loops; (4) remove `octoquad` from the RC config → at init the DS shows the banner `IRON HW: TURRET absent at init (...)` before PLAY; (5) verify a long init_loop never flags "no data" for TURRET (silence windows gated to PLAY); (6) verify the SDK path: force an init hardware failure and confirm our banner logic does not clobber or wipe the SDK's own global error.

---

## 13. Session 6 — shoot-readiness triggers + auto-transfer + left-trigger latch

The user asked for three things, ported from the 2026-IndianaJoe FRC `SystemControls/SystemController.java`:

1. **The readiness triggers** (`isReadyToShoot` and its members).
2. **The transfer auto-running when "ready to shoot" is true** (RPM, turret position, alignment, chassis stability, no-fire zones).
3. **The left trigger becoming a TOGGLE** instead of IndianaJoe's push-to-hold-with-a-separate-unlock-button.

### 13.1 What the FRC code actually did (read before porting)

`SystemController` exposed a set of `Trigger`s and one AND of them:

```java
isReadyToShoot = isFlywheelAtSpeed()   // RPM near target, 0.25s falling debounce
                && isHoodAtAngle()     // hood position
                && isTurretOnTarget()  // turret within a distance-scaled tolerance
                && isDrivetrainStable()// hypot(vx,vy) under StableTranslationSpeedMaxMps
                && isRotationStable()  // |omega| under StablerotationSpeedMaxDegreespersecond
                && isShooterClear();   // !NoFireZones.isInNoFireZone(pose)
```

and `RobotContainer` bound it as `leftTriggerToggle.and(isReadyToShoot()).whileTrue(FeedAndKick())` while the left trigger ran `TestTargetAcquire()` — i.e. **the aim was latch-driven and the feed was readiness-driven, with the two as separate commands**. `leftTrigger.onTrue(state = true)` plus a separate park button was the "push to activate, another button to unlock" the user described.

### 13.2 The FTC port — `CommandFactory` (new readiness section)

| FRC (`SystemController`) | Biobuzz (`CommandFactory`) | Notes |
|---|---|---|
| `isFlywheelAtSpeedRAW()` | `isFlywheelAtSpeed()` | `Flywheel.isHoldingTarget() && isAtTargetRpm()` — the `isHoldingTarget` half matters: a COAST/STOP otherwise reads "at 0 RPM" and looks ready |
| `isFlywheelAtSpeed()` (debounced) | folded into `isReadyToShoot()` | one debounce on the whole AND instead of one on the wheel |
| `isHoodAtAngle()` | `isTurretAtSetpoint()` | this bot has no hood — the LUT RPM is its variable; the slot is "the pointing mechanism obeyed its setpoint" (`TurretPivot.atSetpoint()`, false while BRAKED/IDLE so it can't pass before an aim exists) |
| `isTurretOnTarget()` | `isTurretOnTarget()` | `AimAtGoal.isLiveOnTarget()` — **works with no command scheduled** |
| distance-scaled `currentTurretTolerance` | `AimAtGoal.aimToleranceDeg()` | MIN 2.0° near, +0.05°/in past 24 in, MAX 8.0°, and a speed term: full band at `TOL_RPM_REF` 2000 RPM ramping linearly to MIN at 25 % of it |
| `isDrivetrainStable()` | `isDrivetrainStable()` | Pedro has no `ChassisSpeeds`, so speed comes from the pose delta in `CommandFactory.readinessPeriodic()` |
| `isRotationStable()` | `isRotationStable()` | same pose-delta source |
| `isShooterClear()` | `isShooterClear()` | `!NoFireZones.isInNoFireZone(pose)` |
| `isReadyToShoot()` | `isReadyToShoot()` | the AND, falling-edge debounced by `READY_DEBOUNCE_S` (0.15 s) |

Three deliberate **improvements** over the FRC original:

- **The gate servo is an explicit member** (`isGateSettled()`): FRC's gate was opened from the "at speed" edge, so the servo's own travel was unmodeled. Now the shot can't count itself ready while the gate is still slewing.
- **A motion warm-up window** (`MOTION_WARMUP_S`, 0.5 s): the first pose sample has no predecessor, so its "speed" is undefined; without the grace a noisy first tick could pin the feed off. `REQUIRE_STABLE_TRANSLATION`/`REQUIRE_STABLE_ROTATION` are Panels switches (FRC always enabled both; FRC also skipped them while passing/auton).
- **The debounce is edge-asymmetric on purpose:** rising (ready) is instant, falling (not ready) waits the window. That is FRC's `kFalling` policy generalized from the wheel to the whole AND, so the transfer re-engages the moment the shot is good and only drops after a real (not sample-noise) loss.

### 13.3 The shot is now two halves (this is what makes the transfer auto-fire)

`ShootAtGoal()` used to be one hold-the-trigger compound (aim + RPM + gate). It is now split:

- **`TrackTarget()`** — the latched AIM half: the shared `AimAtGoal` instance + the distance→LUT→slew RPM manager. Runs whenever the latch is on, ready or not. No gate, no transfer.
- **`FireAtGoal()`** — the FIRE half: requires **gate servo + transfer only**, and every loop opens the gate and runs the transfer iff `isReadyToShoot()`, else closes the gate and stops the transfer. **This is the "auto run the transfer when ready" behavior.**
- **`AutoShootAtGoal()`** — both in parallel; **this single instance is what the left trigger toggles**. `ShootAtGoal()` keeps its name and returns it.

**Why the split (and not `leftTriggerToggle.and(isReadyToShoot()).whileTrue(FeedAndKick())` verbatim):** `ParallelCommandGroup` throws `"Multiple commands in a parallel group cannot require the same subsystems"` if two children share a requirement (`ParallelCommandGroup.addCommands`, verified in the SolversLib 0.3.6 sources). Binding `isReadyToShoot().whileTrue(...)` alongside the aim would have to own the wheel/turret again, which is exactly the fight the split avoids: the fire half requires only the two ball-path mechanisms, so it can gate on aim state it merely READS.

**Why readiness protects the BALL, not the wheel:** the ball only leaves through the transfer roller. Feeding while the wheel is off its commanded RPM jams the ball in the barrel or dribbles it; feeding while the turret is off target fires wherever it happens to point (and inside a no-fire zone, at something we must not hit). So `isReadyToShoot` is the policy for the gate+transfer pair, and the gate remains the hardware interlock.

**Latched state lives on the COMMAND:** `toggleWhenActive` schedules/cancels one command instance, so "is the shot on" is `cmd.isScheduled()`. `CommandFactory` therefore builds that instance ONCE in its constructor (`autoShootCmd`); handing out a fresh instance per call would desync the button's edge bookkeeping from the robot.

### 13.4 Controls — left trigger is the toggle, left bumper is the unlock

| Control | New behavior |
|---|---|
| **Left trigger** | **Toggle.** `toggleWhenActive(autoShoot)` — press once to start aiming + auto-firing, press again to stop. No hold, no second unlock press. |
| **Left bumper** | **Unlock / panic.** `CommandFactory.getStopShootCommand()`: cancels that same latched instance (runs both halves' `end()` → gate closed, transfer stopped, flywheel coasts to idle, turret BRAKED), then re-asserts gate closed / transfer stopped / wheel idle / turret NEUTRAL. Takes **no subsystem requirements** and finishes immediately, so it can never fight the compound it just cancelled. Idempotent. |
| Turret jog (dpad L/R) | unchanged, still gated on `!isAutoAimOn()` (renamed from `!isShooting()`) |
| Intake (right trigger) | unchanged binding; the fire half now parks it softly while feeding and hands it back (see 13.5) |
| Transfer manual test | LB moved to the panic, so both manual directions are on the right cluster: **RB = forward hold**, **R3 (right stick click) = reverse hold**; either release PID-brakes. A manual transfer press also overrides the auto-fire for `FIRE_OVERRIDE_HOLD_S` (0.75 s) so a jam can be cleared without being instantly overridden. |

### 13.5 Intake hand-off (no requirement fight)

The intake is deliberately **not** a requirement of `FireAtGoal`. Instead: while feeding, if the intake is actually spinning (`> INTAKE_RUNNING_RPM`) it gets one `stop()`, once per firing episode; after firing, `restoreManualIntakeWhenFree()` re-asserts `runIntake()` **only if the driver still holds the right trigger** — which is why `KeyBindings.update()` reports `setManualIntakeHeld(...)` every loop (same `INTAKE_TRIGGER_THRESHOLD` the binding uses). Runtime state alone could not answer it: once the fire half stops the roller, "it is stopped" reads exactly like "the driver let go".

### 13.6 Aiming math made always-live (`AimAtGoal`)

The readiness checks need a target and a distance BEFORE any aim command has ever run. `AimAtGoal` now exposes, with the turret control law (`commandTurretTo`) unchanged — verified line-by-line against the pre-session file:

- `alliancePoints()` — the current alliance's points, cached per alliance (falls back to all non-"Robot" points).
- `liveTarget()` / `liveTargetName()` — the closest point to the SHOOTER pose, computed on demand; prefers the hysteresis-locked target when this instance IS the running command, so the LUT and the "on target" check can never disagree.
- `distanceTo(Pose)` / `liveDistanceInches()`, `isLiveOnTarget()`, `liveHeadingErrorDeg()` — the same bearing/distance math as `execute()`, with no state writes.
- New constructor overload with a `DoubleSupplier targetRpm` (the LUT's live command) for the speed-scaled tolerance; the old constructors still delegate with `null`.

`getLastDistanceInches()` now returns NaN when the command is NOT aiming, and `end()` clears the per-run shot state — otherwise the RPM manager would read a stale distance from a previous target.

### 13.7 Wiring / call-order requirements

`CommandFactory.readinessPeriodic()` advances the debounce + warm-up timers and samples chassis motion; it **must be called once per loop before the scheduler polls triggers**. It is wired in both places:

- `KeyBindings.update()` (TeleOp) — also sets `manualIntakeHeld`.
- `AutoOpModeBase.loop()` (autos) — this was a real gap: autos had no call at all, so the debouncers would never tick and the motion checks would read a permanent "stable".

### 13.8 New Panels tuning (CommandFactory / AimAtGoal)

`MAX_TRANSLATION_SPEED_IN_S` (2.0), `MAX_ROTATION_SPEED_DPS` (60), `READY_DEBOUNCE_S` (0.15), `REQUIRE_STABLE_TRANSLATION`, `REQUIRE_STABLE_ROTATION`, `MOTION_WARMUP_S` (0.5), `FIRE_OVERRIDE_HOLD_S` (0.75), `restoreIntakeRpmOnRelease` (true), `INTAKE_RUNNING_RPM` (50); `AIM_TOL_MIN_DEG` (2.0), `AIM_TOL_NEAR_DEG` (24), `AIM_TOL_SLOPE_DEG_PER_IN` (0.05), `AIM_TOL_MAX_DEG` (8.0), `TOL_RPM_REF` (2000), `TOL_RPM_FLOOR_FRAC` (0.25), `ON_TARGET_DEBOUNCE_S` (0.15).

### 13.9 Validation done / still to do

**Done:** `:TeamCode:compileDebugJavaWithJavac` and `:TeamCode:assembleDebug` both green; the falling-edge debounce unit-tested standalone (rising instant, short-false held, sustained-false drops, re-arm, never-true, zero window, pause-does-not-catch-up — all pass); the distance/RPM tolerance curve unit-tested for clamping and monotonicity; `KeyBindings` bytecode checked for `toggleWhenActive(Command,boolean)` on the left trigger, `getFireCommand`→panic, and `readinessPeriodic()` in `update()`.

**NOT yet field-verified — bench checklist:**
1. Left trigger press → `Shoot latch (LT toggle): ON`, turret tracks, wheel spins to the LUT RPM; press again → latch OFF, gate closes, transfer stops, wheel coasts to idle, turret released.
2. Left bumper while latched → same OFF state even if the latch was somehow still on; pressing it twice must not throw.
3. Panels `Fire policy` line reads `WAIT (not ready)` until the wheel is at RPM + turret on target + chassis still, then `FIRE (ready)` with the transfer visibly running; grab the turret by hand → `WAIT` within ~0.15 s.
4. Drive the robot into a no-fire zone while latched → gate closes, transfer stops; drive out → resumes (no button press).
5. Hold right trigger (intake) while the shot fires → intake is parked by the fire half, then resumes on its own because the trigger is still held; release the trigger mid-fire → intake stays stopped.
6. Dpad jog while the latch is ON does nothing; with the latch OFF it jogs ~4°.

---

## 14. Session 6b — transfer on/off chatter (capture 20261025-005022) + controller revert

Two user reports, both confirmed in the log. Source capture:
`panels-capture-20261025-005022.json` (304 samples @ 50 ms, 15.3 s).

### 14.1 UNPROMPTED CHANGE (corrected): the drive controller is a Logitech, not an Xbox

Session 6 switched `KeyBindings` to `new CommandXboxController(gamepad1)` and added a `refreshPad()` branch that merged the Panels browser pad. That was not asked for. The team runs a **Logitech F310** for now. Reverted:

- `DriveController = new CommandGamepad(opMode.gamepad1)` (plain SDK axes, no remap).
- **Deleted** `refreshPad()` and every `CommandXboxController` / `PanelsGamepad` reference from `KeyBindings`, plus the `keys.refreshPad()` call in `JowByTeleOp.loop()`.
- `CommandXboxController` itself is untouched in `WattageLib/lib` — it is still available for a future pad swap; it is simply not used by the TeleOp path.

**Rule going forward:** do not change the input layer to obtain a feature. Bind through `CommandGamepad`, which already exposes everything this codebase needs (`leftTrigger(th)`, `dpad*`, `leftBumper`, `rightStick`, `rightTrigger(th)`, `toggleWhenActive`, …).

### 14.2 The transfer chatter — THREE independent causes, all measured

The symptom was ~16 transfer on/off transitions in 5 s. The capture separates into three distinct faults:

**(a) The at-speed test was a single threshold sitting on the encoder's noise floor — the main cause.**

`Flywheel` reads its built-in encoder at `TICKS_PER_REV = 28` (goBILDA 5203, `gobilda5203TicksPerRev(1.0)`). At the measured ~50 ms loop, **one encoder tick ≈ 43 RPM**, and the capture shows exactly that quantisation:

```
1671.4 / 1714.3 / 1757.1 / 1800.0     (43 RPM apart)
```

`Flywheel.AT_SPEED_TOLERANCE_RPM` is **75 RPM** — under two ticks. Error distribution over the 114 firing-window samples:

| |error| | samples |
|---|---|---|
| ~0–50 RPM | 61 |
| ~50–100 RPM | 41 |
| ~100–150 RPM | 7 |
| >150 RPM | 5 |

So 99 of 114 samples sat at 50–100 RPM of error, straddling a 75 RPM line. The wheel was hunting inside its own PID deadband; readiness was reading encoder quantisation, not speed. **Replaying the capture through the new logic: 14 transitions → 1.**

Fix — split the band into a Schmitt trigger (`CommandFactory`):
- `FLYWHEEL_READY_ON_RPM = 120` (≈2.8 ticks): enter "at speed" only when solidly there.
- `FLYWHEEL_READY_OFF_RPM = 220` (≈5 ticks): leave only when genuinely lost.
- Compared against `commandedRpm` (the LUT output), not against `Flywheel.TARGET_RPM` re-read, so the readiness check and the RPM manager agree by construction.

**(b) The LUT setpoint itself was wandering while the robot stood still.**

`Shoot LUT cmd RPM` crawled **1827 → 1805** during the fire window; `Aim dist (in)` moved only **45.8 → 47.6 in**. That is sub-inch pose/LUT noise re-feeding the setpoint every loop, which then re-opened the at-speed question in (a). Fix:
- `LUT_DISTANCE_DEADBAND_IN = 1.0` — inside the band the previous distance is reused.
- `LUT_RPM_FILTER_ALPHA = 0.25` — first-order low-pass on the LUT output, so a real distance change still arrives smoothly instead of in interpolation steps.

**(c) The transfer was being stopped and re-commanded every loop.**

Old code called `transfer.stop()` in the `else` branch **every loop** readiness was false, then `transfer.runForward()` when it returned — a hard stop/start pair per sample. Fix: the fire half now tracks an intended `FeedState` and writes `stop()` **once on the falling edge**. `runForward()` is still re-asserted every loop while feeding, because it re-reads `TRANSFER_RPM` (live Panels retunes must keep working) — but re-asserting a non-zero setpoint is not the same as chattering the motor.

Also added aim-alignment hysteresis (`AIM_HYST_EXIT_MARGIN_DEG = 1.5`): once aligned, the turret may drift 1.5 deg past the live tolerance before the shot is pulled, instead of a single-degree threshold flipping per sample.

### 14.3 "I have no way to know WHY it is not firing" — fixed

Two problems, and the second was actively misleading:

1. **The readiness lines only appeared while `firing` was true**, so the moments that explain a stall were often missing.
2. **`Ready to shoot` was computed in `readinessPeriodic()` (one tick AFTER the scheduler) while `Fire policy` was computed inside `FireAtGoal.execute()`.** In the capture they visibly contradict each other — `FIRE (ready)` is logged next to `Ready to shoot: false` on all 16 firing samples, and `Ready to shoot` reads true on only 3 samples in the whole 15 s. The line was stale, not meaningful.

Fix:
- New `Readiness` snapshot: **every member is evaluated exactly once per loop**, and that one snapshot drives both the transfer decision and the telemetry. This also fixes a subtle correctness bug — the hysteresis members are stateful, and the old code called them separately for policy and display, consuming extra Schmitt steps.
- `readinessPeriodic()` no longer emits a competing `Ready to shoot`.
- New per-loop dashboard lines while latched:

| Line | Shows |
|---|---|
| `Fire policy` | `FIRE (ready)` / `WAIT (not ready)` |
| `Ready to shoot` | the same snapshot's AND (can no longer disagree) |
| `Fire block` | **the single blocking member**, with its number — e.g. `FLYWHEEL 1714/1805 RPM (err 91, on<=120 off>220)`, `AIM err 3.2 / tol 2.6 deg`, `ROTATE (84 dps)`, `NO-FIRE ZONE`, `TURRET POS (MOVINGTOPOSITION 12.4/31.0)` |
| `  ok: flywheel / turret pos / aim / gate / translation / rotation / clear` | one `OK`/`NO` flag each with the live value vs the limit |
| `Fire transfer` | `OFF` / `FEEDING` |

`Fire block` is the one to glance at: it names the reason in one word.

### 14.4 Controller revert verification

`:TeamCode:assembleDebug` green. `KeyBindings` diff vs HEAD contains **no** `CommandXboxController`, `PanelsGamepad`, or `refreshPad` references; `JowByTeleOp.loop()` is back to `RobotRunPeriodic() → keys.update() → UpdateTelemetry() → flushTelemetry()`.

### 14.5 Bench checklist for the chatter fix

1. Latch on, let the wheel spool: `Fire policy` flips to `FIRE` once and **stays** there; `Fire transfer` shows `FEEDING` continuously (no strobing).
2. Watch `Fire block` during the spool: should read `FLYWHEEL …` until it converges, then `none`.
3. Force a real ready-loss (grab the turret, or spin the chassis): `Fire block` names `AIM` / `ROTATE`, and the transfer stops once, resuming when clear.
4. Confirm `Shoot LUT cmd RPM` is steady (±2 RPM) while standing still — it used to crawl ~22 RPM.
5. Re-check the transfer didn't lose its manual controls: RB forward, R3 reverse.

---

## 15. Session 6c — "GATE (moving)" blocked every shot (capture 20261025-010811)

Symptom: sitting still, the auto-fire never ran; `Fire block` read `GATE (moving)` ~37% of the time. The gate mechanism is **not built yet**.

### 15.1 Root cause — the gate check was CIRCULAR, not a hardware problem

`Pos shootgate` in the capture shows what was really happening:

```
tgt 0.150 cmd 0.150 AT       <- gate "settled" (closed), readiness passes
tgt 0.150 cmd 0.182 moving   <- fire half OPENS the gate -> gate now "moving" -> readiness FAILS
tgt 0.750 cmd 0.252 moving   <- gate closing again (because not ready)
tgt 0.150 cmd 0.150 AT       <- arrived -> readiness passes -> repeats
```

The readiness member was `PosServ.isAtPosition()`, which compares `target` against `current` — and `current` is the **slew-limited value the code generates itself** (`ShootGateServo.periodic()`, `current += sign(err)*SLEW*dt`). There is no position feedback from a servo. So "is the gate settled?" really means "has MY software ramp finished?".

Because the shot's first action is to OPEN the gate, a settled gate can never coexist with a shot in progress:

> ready → open the gate → ramp runs → gate not settled → not ready → close the gate → ramp runs → settled → ready → …

Measured: **60 of 160 samples** blocked by the gate; a 50 Hz simulated replay oscillates **200 transitions in 4 s**.

Nothing in this was a hardware fault. The mechanism being absent only made it visible, because the useless servo sweeps kept the ramp busy forever.

### 15.2 Fixes

**(a) `ShootGateServo.IMPLEMENTED = false`** (new static flag, Panels-tunable, next to the other per-mechanism constants). While false:
- the gate is dropped from the readiness AND (`gateOk()` returns true),
- `openGate()` / `closeGate()` are no-ops, so the fire half never writes the servo and never leaves it mid-slew,
- the panic/unlock and the RPM manager's `end()` also stop writing it.

`ShootGateServo` itself is untouched and fully wired — flipping the flag to true restores the readiness member, the fire-half open/close, and the servo in one edit, once the mechanism is mounted and `POS_A`/`POS_B` are calibrated.

**(b) The circularity itself, so it cannot come back when the gate EXISTS.** New `ShootGateServo.isClosed()` (target is at the closed preset) replaces `isAtPosition()` in the readiness set. "Closed" is stable through the entire firing sequence, because the fire half only opens the gate AFTER readiness has already passed:

| test | simulated 4 s | verdict |
|---|---|---|
| old `isAtPosition()` | 200 transitions | oscillator |
| new `isClosed()` | 1 transition | settles |

`isAtPosition()` is kept (it is the correct "the slew finished" query for the command classes) but its javadoc now states plainly that it is a statement about the software ramp, not the hardware, and why gating a shot on it while opening the gate is a guaranteed oscillator.

**(c) The dashboard no longer hides it.** `ok: gate` now prints `(NOT IMPLEMENTED - ignored)` when the flag is false, so the line reads as a deliberate bypass instead of a passing check. When the gate is live it prints `closed` / `OPEN`.

### 15.3 Replay of the capture after the fix

| `Fire block` | count |
|---|---|
| `none` (was `GATE (moving)`) | **60** |
| `none` | 16 |
| `FLYWHEEL 257/1818 RPM` (spool-up) | 3 |
| `FLYWHEEL 1629/1818 RPM` | 2 |
| `FLYWHEEL (no RPM commanded)` | 2 |
| `FLYWHEEL 1371/1818 RPM` | 1 |

Every gate block disappears; the only remaining blocks are legitimate flywheel spin-up, which is what the readiness set is *supposed* to wait for.

### 15.4 What is still worth checking on the bench

- The flywheel spool-up blocks above are normal, but confirm the wheel reaches `FIRE` and stays there (session 14.5 item 1).
- Manual gate controls (X toggle, dpad nudge) are deliberately NOT behind `IMPLEMENTED` — they still write the servo, so the servo can be bench-tested before the mechanism exists. Stop using them once the gate is mounted but uncalibrated.
- The autos still call `robot.PosServ.toPos(0.5 / 1.0 / 0.0)` directly (L_ShootLFlowerShootL, RightAuto, R_ShootPark). Those were pre-existing and are unrelated to the fire policy, but they will sweep an unattached servo when run — worth cleaning up when the gate lands.

**Compile:** `:TeamCode:assembleDebug` green.
