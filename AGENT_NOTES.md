# Agent Notes & Change Log

Centralized log for AI coding agents working on **Biobuzz-5960**.
Keep historical change notes, debugging post-mortems, tuning history, and multi-session context **here** instead of inside `.java` source files (see `AGENTS.md`).

---

## Format for New Entries
```markdown
## YYYY-MM-DD — Short Summary
- **Files touched:** ...
- **Context / Why:** ...
- **Notes for future sessions:** ...
```

---

## Existing Reference Logs
- `TURRET_PORTING_NOTES.md` — Detailed porting & bench-tuning log for `TurretPivot`, OctoQuad CRT boot, shoot-readiness triggers, and gate/transfer chatter fixes.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Vision/VisionFusion-Tuning.md` — Vision Kalman fusion tuning notes.

---

## 2026-11-23 — Post-Turn Settle Speedup (`st2` Follow-up)
- **Files touched:** `Subsystems/turret/TurretPivot.java`, `Subsystems/turret/TurretTracker.java`, `Commands/AimAtGoal.java`.
- **Context / Why (from `st2` capture analysis):**
  - `st2` confirmed `0` `GATE`/`HOOD` blockers and `0` readiness/telemetry mismatches, with clean post-turn rest-to-fire delays of `0.30–0.35 s`.
  - Frame-by-frame inspection showed that `0.30–0.35 s` came from two tunable sources: `~150–180 ms` of end-of-turn `±3°` turret overshoot/ringing (`t=7.59–7.94s`, `t=51.59–51.85s`) and `100–140 ms` from `POSITION_DWELL_S = 0.10 s` forcing 3–4 loops at `~39.5 ms` loop time (`t=27.05–27.25s`).
  - Reduced `TurretPivot.POSITION_DWELL_S` (`0.10 -> 0.045 s`, still 2 consecutive settled loops alongside the `SETTLE_VEL_DPS = 15 deg/s` gate), raised `TurretPivot.kD` (`0.0014 -> 0.0019`) to damp end-of-turn bounce, set `TurretTracker.VELOCITY_EMA_ALPHA = 1.0` (was `0.7`) to remove 1-loop velocity lag in hybrid motor-encoder mode, and raised `AimAtGoal.YAW_RATE_EMA_ALPHA` (`0.65 -> 0.85`) so chassis-stop feedforward decays faster.

---

## 2026-11-23 — Session-Level ShootGate, Turret/Hood Hysteresis & Same-Tick Readiness Fixes
- **Files touched:** `Commands/CommandFactory.java`, `Subsystems/turret/TurretPivot.java`, `RobotMain.java`, `TeleOp/KeyBindings.java`, `TeleOp/JowByTeleOp.java`, `Autos/AutoOpModeBase.java`.
- **Context / Why (from `lv` vs `st` capture comparison):**
  - **Session-level shoot gate (`CommandFactory`):** `FireAtGoal()` and `FireManualAtGoal()` now open the gate immediately in `initialize()` and keep it open throughout the active shooting session, only closing on `end()` (deactivation) or manual reverse (`B`). Moment-to-moment readiness pauses stop/resume `Transfer` only, eliminating the `~0.40 s` gate close/reopen penalty after every turn or brief readiness dip.
  - **Turret settle hysteresis & state preservation (`TurretPivot`):** Added `POSITION_HYST_EXIT_MARGIN_DEG = 2.0` (`3.5°` entry -> `5.5°` exit), required settled tracking velocity (`|vel - desiredVelocityDps| <= SETTLE_VEL_DPS`) before entering `INPOSITION` (prevents premature `INPOSITION` during fast overshoots), and stopped `GotoPositionDeg()` from unconditionally resetting `State = MOVINGTOPOSITION` every loop while already settled. Prevents `anchorAtRest()` `~0.6°` snaps from bouncing readiness.
  - **Hood SOTM hysteresis (`CommandFactory`):** Added `HOOD_HYST_EXIT_MARGIN_DEG = 1.5` (`1.0°` entry -> `2.5°` exit via `hoodAtAngleHyst()`) so single-tick `1.4°–1.7°` hood LUT steps during SOTM do not drop readiness for one loop before `Hood.periodic()` slews.
  - **Same-tick readiness & telemetry ordering (`RobotMain`, `CommandFactory`, `KeyBindings`, `JowByTeleOp`, `AutoOpModeBase`):** Moved `CommandFactory.updateMotionAndTimers()` before `CommandScheduler.run()` and `readinessPeriodic()` right after `CommandScheduler.run()`, evaluated readiness fresh inside `FireAtGoal()` / `FireManualAtGoal()` after `TrackTarget()` and subsystem `periodic()` run, and moved `KeyBindings.reportTelemetry()` into `JowByTeleOp.UpdateTelemetry()` (post-scheduler). Eliminates the 1-loop (~40 ms) stale readiness/latch lag.

---

## 2026-11-20 — Pre-State Championship Optimization, Gate Latch Fix & Cleanup
- **Files touched:** `RobotMain.java`, `TeleOp/JowByTeleOp.java`, `TeleOp/KeyBindings.java`, `Commands/AimAtGoal.java`, `Commands/CommandFactory.java`, `Subsystems/Flywheel.java`, `Subsystems/Intake.java`, `Subsystems/Transfer.java`, `Subsystems/Hood.java`, `Subsystems/Ramp.java`, `Subsystems/ShootGateServo.java`, `Subsystems/turret/TurretPivot.java`, `Subsystems/turret/OctoQuadAbsoluteSource.java`, `Subsystems/Vision/Vision.java`, `Subsystems/Vision/VisionFusion.java`, `Subsystems/Vision/FusionTune.java`, `Subsystems/pedro/Tuning.java`, `Autos/AutoRegistrar.java`, `WattageLib/Ballistics/NoFireZones.java`, `WattageLib/lib/CommandGamepad.java`, `AGENTS.md`.
- **Files deleted:** `HiveCellMonitor.java`, `TipCompensator.java`, `ContinuousServo.java`, `CommandXboxController.java`, 7 unused legacy autos (`RightAuto`, `TwoTipSingle`, `TwoPointFiveTipAuto`, `TwoPointFiveTipUnderneathAuto`, `ShootAndParkFlowerSide`, `L_ShootLFlowerShootL`, `R_ShootR_Garden_ShootR_Park`), and 5 unused Pedro tuners (`OTOSTuner`, `OctoQuadTuner`, `TwoWheelTuner`, `ThreeWheelTuner`, `ThreeWheelIMUTuner`).
- **Context / Why:**
  - **Loop ordering & single-evaluation readiness:** `JowByTeleOp.loop()` now runs `keys.update()` before `robot.RobotRunPeriodic()` so manual intents, `readinessPeriodic()`, and drive powers take effect in the same tick. `CommandFactory` now evaluates manual shoot readiness once per loop alongside auto readiness (`lastManualFlywheelReady`, `lastManualGateOpen`, `lastManualReady`) and replaces `READY_DEBOUNCE_S` / `FallingEdgeDebounce` with the per-loop snapshot.
  - **ShootGateServo open-wait latch:** Removed `IMPLEMENTED` flags/guards from `Hood`, `Ramp`, `ShootGateServo`, and `CommandFactory` (hardware is mounted). Added `ShootGateServo.isOpen()`, switched `gateOk()` to `PosServ.isOpen()`, separated `Readiness.shooterReady` from `gate`, moved `"GATE (opening)"` to the end of `firstBlocker()`, and updated `FireAtGoal()` / `FireManualAtGoal()` to open the gate when `shooterReady` passes and start transfer feeding once `PosServ.isOpen()` is true.
  - **Shared mutable `AimAtGoal` state:** Active `AimAtGoal` instances publish directly to `CommandFactory.aim`'s public `active*` fields (`activeAiming`, `activeTargetPose`, `activeTargetName`, `activeEffectiveTarget`, `activeDistanceIn`, `activeRawDistanceIn`, `activeHeadingErrorDeg`, `activeToleranceDeg`), and `updateIdleSnapshot()` refreshes them once per loop when idle. Replaced per-loop `Pose[]` array allocations in `alliancePoints()` with a cached `ArrayList`.
  - **I2C/CAN/CPU/GC reductions:** Gated Panels telemetry and `TelemetryFileLogger` creation behind `RobotMain.DASHBOARD` / `RobotMain.TELEMETRY_IO`; cached redundant motor/servo writes (`Intake`, `Transfer`, `Flywheel`); shared one `limelight.getLatestResult()` per loop in `Vision` (`polledThisLoop`) and cached `mt1Pedro`/`mt2Pedro`; guarded `OctoQuadAbsoluteSource` `resetEverything()` + `saveParametersToFlash()` behind channel-config equality; switched `CommandFactory` LUT filtering from fixed alpha to `dt / (LUT_FILTER_TAU_S + dt)`; fixed `Intake.java` `TICKS_PER_REV` comment to `19.2:1 = 537.6`; deduplicated `deadReckoning` reflection via `VisionFusion.getDeadReckoning()`.
- **Notes for future sessions:**
  - `RobotMain.DASHBOARD` defaults to `false` for lean competition loops; set `DASHBOARD = true` in Panels or code when debugging field/shoot telemetry.

---

## 2026-11-20 — Shoot-On-The-Move (SOTM) Virtual Target Lead
- **Files touched:** `Commands/CommandFactory.java`, `Commands/AimAtGoal.java`, `Subsystems/turret/TurretPivot.java`, `WattageLib/lib/PanelsFieldRenderer.java`, `RobotMain.java`.
- **Context / Why:**
  - Implemented virtual-target velocity lead (`correctedTarget = target - shooterVel * TOF * MOVING_LEAD_SCALAR`, clamped to `SOTM_MAX_LEAD_IN = 36.0`, two-pass TOF lookup via `CommandFactory.LUT_TOF_S` / `lutTofSec()`).
  - Applied the same SOTM-compensated target across `AimAtGoal.commandTurretTo()`, `liveDistanceInches()`, `liveHeadingErrorDeg()`, `isLiveOnTarget()`, and `liveEffectiveTarget()`.
  - Added purple aim-target dot (`PanelsFieldRenderer.drawAimTargetDot()` called from `RobotMain.drawField()`) so the Panels Field widget shows the raw goal when stationary and the shifted virtual aim point while moving.
  - Raised `CommandFactory.MAX_TRANSLATION_SPEED_IN_S` (`2.0 -> 40.0`), `MAX_ROTATION_SPEED_DPS` (`60.0 -> 80.0`), lowered `LUT_DISTANCE_DEADBAND_IN` (`1.5 -> 0.5`), and updated `readinessPeriodic()` to prefer `follower.velocity()` over raw 1-loop pose deltas so vision corrections do not falsely block shooting.
  - Updated `TurretPivot.GotoPositionDeg()` to only reset `inToleranceSinceNs` on inactive-to-active transitions or steps `> 5.0°` (was `> 0.05°`, which prevented `atSetpoint()` from holding true while tracking a moving target).
- **Notes for future sessions:**
  - `CommandFactory.LUT_TOF_S` values `{0.35, 0.45, 0.52, 0.60, 0.70, 0.85}` are initial estimates and should be calibrated from video frame counts at each LUT distance; tune `AimAtGoal.MOVING_LEAD_SCALAR` (default `1.0`) down to `0.5–0.8` if shots over-lead while strafing.

---

## 2026-11-18 — Flywheel Simplification & Snappier Speed Confirmation
- **Files touched:** `Subsystems/Flywheel.java`, `Commands/CommandFactory.java`, `TeleOp/KeyBindings.java`, `TeleOp/JowByTeleOp.java`.
- **Context / Why:**
  - Flywheel encoder (`TICKS_PER_REV = 28`) quantizes velocity in `42.9 RPM/tick` steps at 20 Hz, so `FLYWHEEL_READY_ON_RPM = 55.0` (~1.28 ticks) + `FLYWHEEL_SETTLE_DWELL_S = 0.10` caused repeated dwell-timer resets on 1–2 tick quantization bounces and delayed confirmation up to ~1 s after turning/stopping.
  - Fixed a bug in `CommandFactory.rpmManagerCommand()` where `lutRpm(d)` was evaluated *before* `d` was clamped by `LUT_DISTANCE_DEADBAND_IN`, making the deadband a no-op for flywheel RPM; raised `LUT_DISTANCE_DEADBAND_IN` (`1.0 -> 1.5` in) and `LUT_RPM_FILTER_ALPHA` (`0.25 -> 0.50`).
  - Added time-aware EMA smoothing (`Flywheel.RPM_SMOOTH_TAU_S = 0.06` s) so `getSmoothedRpm()` filters single-tick encoder jitter while tracking slow movement without dwell lag.
  - Updated `CommandFactory` defaults (`FLYWHEEL_READY_ON_RPM = 85.0`, `FLYWHEEL_SETTLE_DWELL_S = 0.0`, `FLYWHEEL_READY_OFF_RPM = 175.0`) and `Flywheel.AT_SPEED_TOLERANCE_RPM = 85.0`.
  - Simplified `Flywheel.java` by removing the unused `AutotuneState` machine and `periodicAutoTrim` (which was reset every loop by `rpmManagerCommand()` calling `setTargetRpm()`), and collapsing `Mode` to `STOPPED`, `HOLDING`, `COASTING_TO_IDLE`.

---

## 2026-11-18 — Turret Simplification, Hybrid False-Demotion Fix & Full Motion Feedforward
- **Files touched:** `Subsystems/turret/TurretTracker.java`, `Subsystems/turret/TurretPivot.java`, `Commands/AimAtGoal.java`.
- **Context / Why:**
  - **Mid-match PWM fallback bug (`TurretTracker`):** `anchorWhileMoving()` compared ~10 Hz stale OctoQuad PWM intervals against `motNetSinceFresh` (which also got zeroed whenever `anchorAtRest()` ran at low speed). Quick direction reversals or stop/start transitions within one 100 ms PWM window caused false `MOTOR_ENC` / `MOTOR_ENC_DIR` latches (`MOTOR_ENC_STREAK_LIMIT = 2`), locking the turret into `PWM_ONLY_MAX_POWER = 0.15` for the rest of the match even though all sensors were healthy.
  - **Removed `WATCH` mode & motor direction confirmation:** Hardware direction (`MOTOR_ENC_POLARITY`) is fixed by hardware setup; starting directly in `ACTIVE` eliminates the 0.15-power startup crawl and removes `watchDeadStreak` / `bootDirMot*` / `faultMotorEncDir`. `anchorAtRest()` now also requires PWM readings to be settled (`MOTOR_ENC_REST_SETTLE_S = 0.12 s` since last change) so stale ~10 Hz PWM doesn't snap position backward right as the turret decelerates into rest.
  - **`IN_BAND_HOLD` vs. feedforward (`TurretPivot`):** `IN_BAND_HOLD` previously cut `out = 0` inside `±3.5°` even while spinning/moving, discarding `kV * desiredVelocityDps` until error grew past `3.5°`. Now `IN_BAND_HOLD` only cuts power when `|desiredVelocityDps| < 2.0`.
  - **Responsiveness & full motion FF:** Raised `TurretPivot.kP` (`0.022 -> 0.042`), `kD` (`0.0009 -> 0.0014`), shrunk `EDGE_SLOW_ZONE_DEG` (`30 -> 12`), and updated `AimAtGoal` to read `follower.velocity()` (`vx`, `vy`, `omega` with `YAW_RATE_EMA_ALPHA = 0.65`) and add translational bearing-rate FF (`(dy*vx_s - dx*vy_s)/dist^2`, scaled by `TRANS_FF_SCALE = 1.0`) alongside `-yawRateDps`.
- **Notes for future sessions:**
  - Do not reintroduce mid-match PWM-vs-motor direction demotions using ~10 Hz PWM intervals unless staleness/buffering is explicitly time-aligned; hub quadrature read exceptions (`MOTOR_ENC_READ_FAIL_LIMIT`) + `STUCK` watchdog + Encoder-A cross-check already cover hardware failures.

---

## 2026-11-17 — Codebase Comment Cleanup & Centralized Technical Context
- **Files touched:** Core (`RobotMain`, `HardwareHealth`, `IronConstants`), `Autos/AutoOpModeBase`, `TeleOp/` (`JowByTeleOp`, `KeyBindings`), `Commands/` (`AimAtGoal`, `CommandFactory`), `Subsystems/` (`Flywheel`, `Ramp`, `ShootGateServo`, `Hood`), `Subsystems/turret/*`, `Subsystems/Vision/*`, and `WattageLib/` (`NoFireZones`, `ContinuousServo`, `TelemetryFileLogger`, `PanelsFieldRenderer`, `CommandXboxController`).
- **Context / Why:** Per `AGENTS.md`, removed multi-session change logs, tuning histories, and debugging post-mortems from `.java` source files so students see only concise variable-effect and method-usage comments. Preserved critical root-cause invariants below for future agent sessions.

### Key Invariants & Historical Findings (Do Not Regress)

1. **Turret (`TurretPivot`, `TurretTracker`, `OctoQuadAbsoluteSource`, `TurretCrt`):**
   - **Profile stability invariant:** `kP * |err| + kS * ramp` must stay below `cap(|err|) = PROFILE_MIN_POWER + PROFILE_GAIN_PER_DEG * |err|`, or the turret enters a ~4 Hz relay limit cycle (observed in capture `20261028-012540` with `kP=0.12`, `PROFILE_MIN_POWER=0.22`). Safe tuned baseline: `kP=0.022`, `kS=0.060`, `KS_RAMP_DEG=4.0`, `PROFILE_GAIN_PER_DEG=0.040`, `PROFILE_MIN_POWER=0.120`.
   - **PWM-only speed cap (`PWM_ONLY_MAX_POWER = 0.15`):** Encoder B PWM refreshes at ~7.9 Hz (`32.7 deg` half-wrap / `126.6 ms` ≈ `259 deg/s` max safe unwrap). Higher power (`0.40` in capture `20261028-013546`) folded wraps and caused runaway; `0.15` (~90 deg/s) keeps ~3× margin.
   - **Tracker continuity guard (`CONTINUITY_MAX_DPS = 1200`):** Replaced fixed `crosscheckDiffDeg` threshold because modulo-A wrap errors (~63.4 deg) can mask whole-wrap divergence. `OctoQuadAbsoluteSource` tracks PWM freshness age (~10 Hz refresh).
   - **Nudge & brake:** `TurretPivot.NudgeDeg` reads `getPosition()` when `!setpointActive`; `BRAKE()` switches motor to `FLOAT` + `0` power on release.

2. **Aiming, Readiness & Ballistics (`AimAtGoal`, `CommandFactory`, `ShootGateServo`, `NoFireZones`):**
   - **Yaw feedforward units (`AimAtGoal.YAW_FF_SCALE = 1.0`):** `ffDeg = -yawRateDps * loopDtSec * YAW_FF_SCALE` applies one loop of lead instead of adding velocity directly as a position offset (which thrashed ±30–60 deg in `20261023-025055`). `TOLERANCE_DEG` / `AIM_TOL_MIN_DEG` (`4.0`) >= `TurretPivot.POSITION_TOLERANCE_DEG` (`3.5`) prevents settled-hold deadlock.
   - **AimPoints mirroring:** `AimPoints.pp.json` stores both alliances explicitly; load with `PoseFactory.degrees()` (do not Blue-mirror).
   - **Gate & flywheel hysteresis:** `gateOk()` checks `isClosed()` rather than `isAtPosition()` to avoid ~5 Hz oscillation when opening (capture `20261025-010811`). Flywheel's 28-tick encoder quantizes ~42.9 RPM/tick at 20 Hz, so `CommandFactory` uses hysteresis + dwell (`FLYWHEEL_READY_ON_RPM=55`, `FLYWHEEL_SETTLE_DWELL_S=0.10`, `FLYWHEEL_READY_OFF_RPM=150`), `LUT_DISTANCE_DEADBAND_IN=1.0`, and a single-per-loop `Readiness` snapshot (fixes `20261025-005022` telemetry mismatch).
   - **Command tree reuse:** `AutoShootAtGoal()` must always build a fresh command tree; TeleOp `toggleWhenActive()` uses the single latched instance from `getAutoShootCommand()`.
   - **NoFireZones:** Ported from FRC `ballistics.NoFireZones` (original used a Java 16 `record`; FTC builds at source level 8 so `NoFireZone` is a final class).

3. **Ramp (`Ramp`):**
   - **`tryCreate(OpMode)` ordering:** Resolves servos *before* calling `super()`, because `SubsystemBase` registers with `CommandScheduler` in `super()` and throwing after registration left null/half-init state in Match-0.
   - **Servo direction (`§13.1`, `20261029-012050`):** Never call `Servo.setDirection()` inside `periodic()` alongside `output()` value mirroring (`1.0 - target`), which double-negated and overrode RC config direction.

4. **Vision & Localization (`Vision`, `VisionFusion`, `TipSanityFilter`, `TipCompensator`):**
   - **Split solver:** MT2 supplies gyro-conditioned XY (seed-echo yaw is never fused to avoid gyro feedback loops); MT1 supplies independent 6DOF yaw (plus attitude/Z when `ATTITUDE_FROM_MT1=true`; MT2 always reports 0/0/0 attitude) and optional XY (`USE_MT1_XY=true` after `02-47` MT2 depth-collapse runs).
   - **Frame calibration (`2026-09-29`, `fiducials(4).fmap`):** `LL_SWAP_XY=true`, `LL_YAW_OFFSET_DEG=180` (round-trips through `seedYaw()` and `toPedroPose()`). Camera lever-arm geometry lives in the Limelight Web UI (REST `/api/configset` does not exist); do not re-add code-side camera offsets or they will double-apply.
   - **Matrix inversion NaN safety (`FusionLocalizer 3.0.1`):** R diagonal in `VisionFusion.fuse()` MUST stay finite even when yaw is NaN-masked, otherwise `S = P + R` inversion propagates NaNs into XY.
   - **Dummy pose guards:** `Vision.poll()` rejects `tagCount <= 0` and `(0, 0, 0)` poses (which map to Pedro `(72, 72)`). `VisionFusion` computes MT2 seed agreement early so stationary-gate rejections still log `seedAgrDeg`.
   - **Tip filtering:** `.fmap` stores the 30° hive-cell rest tilt, so a good solve reports flat (`pitch ≈ 0, roll ≈ 0`). `TipSanityFilter` (Stage 1, active) drops off-rest/underground/teleport solves; `TipCompensator` (Stage 2) remains disabled pending field validation.

5. **Utilities (`CommandXboxController`, `PanelsFieldRenderer`, `TelemetryFileLogger`):**
   - **Xbox axis quirk:** On Android/FTC DS, Xbox HID/BLE swaps triggers (`-1` rest) with right-stick lanes; `CommandXboxController` remaps `XBOX_360` pads (leaving F310 untouched) and merges Panels browser gamepad input via `refreshPad()`.
   - **Panels Field widget:** Uses `PEDRO_PATHING` preset (`0..144`, `0 rad = +X`); robot nose tick is `(cos h, sin h)` (field-frame `(-sin h, cos h)` is +90° off).
   - **Android 7 FUSE workaround:** `TelemetryFileLogger` writes to `.log.open` and renames to `.log` on `close()` so `adb pull` gets fresh file sizes instead of stale FUSE attributes (observed `2026-09-29`).
