# Biobuzz-5960 Agent & Coding Guidelines

This repository is the FTC robot codebase for **Team 5960 (Biobuzz)**, maintained for **middle school students**.
Keep code straightforward, readable, and easy for students to find and edit without digging through layers of abstraction.

---

## 1. Coding Style & Philosophy (Middle-School Friendly)

1. **Simple, flat, and inline first**
   - Write routines (especially Autonomous OpModes and diagnostics) **inline** inside `autoRoutine()` using `SequentialCommandGroup`, `ParallelCommandGroup`, `WaitCommand`, `WaitUntilCommand`, and `.withTimeout(...)`.
   - Do **not** wrap simple 1–2 line subsystem actions in extra private helper methods unless the logic is genuinely complex (e.g., searching a LUT array).
   - Prefer direct subsystem calls that students can read top-to-bottom like a checklist (see `Autos/ZDiagnostics.java` and `Autos/R_Shoot_Flower_LShoot.java`).
2. **Direct access to subsystems**
   - Access subsystems directly through `robot.<subsystem>` (e.g., `robot.intake.runCmd()`, `robot.transfer.stopCmd()`, `robot.hood.toAngleCmd(...)`, `robot.turret.C_GotoPositon(...)`, `robot.flywheel.toRpm(...)`).
   - Only add wrapper helpers when truly needed across many OpModes (like `autoShoot()`, `rampUp()`, `rampDown()` in `AutoOpModeBase`).
3. **Direct shared mutable state (keep it simple)**
   - Prefer public mutable fields updated directly by the active command (for example, `CommandFactory.aim.activeTargetPose`, `activeEffectiveTarget`, `activeDistanceIn`) over extra getter abstractions or cached state arrays.
4. **No subsystem null guards or `IMPLEMENTED` flags**
   - Subsystems assume hardware is always wired and present on the robot. Do **not** add defensive null guards for subsystems or `IMPLEMENTED` feature flags/guards—if hardware is removed, update the code or disable the subsystem directly.
5. **Compile-time vs. Panels live tuning**
   - Use `@Configurable` with `public static` fields **only** on classes/values that actually need live dashboard tuning in Panels (`http://192.168.43.1:8080`).
   - One-off OpModes or fixed routines (like `ZDiagnostics`) should use plain `private static final` compile-time constants—do not clutter Panels with unnecessary `@Configurable` entries.
6. **Match existing patterns**
   - Use **SolversLib** (`Command`, `SubsystemBase`, `SequentialCommandGroup`, `ParallelCommandGroup`, `WaitCommand`, `Trigger`) and **Pedro Pathing** (`follow(robot.follower, pp.path("..."))`).
   - **Red-only autos:** Write new autos for the Red alliance and add them to `AutoRegistrar.RED_AUTOS`—`AutoRegistrar` automatically generates the mirrored `Blue_*` OpMode.

---

## 2. Commenting Rules (CRITICAL)

- **Source files are for students.** Comments in `.java` files must ONLY explain:
  - **What a variable/constant does and the effect of changing it** (e.g., units, direction, faster/slower, higher/lower).
  - **How a method or class is currently used** (e.g., what a command does, when to call it, safety limits).
- **NO change-history or session-log comments in code.**
  - Do **NOT** write comments about how a value or method changed over previous rounds/sessions (e.g., *"was 0.40, changed to 0.15 in §12.28 after capture..."*, *"replaces old hold-to-aim..."*, *"fixed bug where..."*).
  - Do **NOT** leave long multi-paragraph agent notes or debugging post-mortems inside `.java` files.
- **Centralize agent notes and history:**
  - Put all multi-session context, root-cause analyses, tuning history, and rationale for changes in **`AGENT_NOTES.md`** (or `TURRET_PORTING_NOTES.md` for turret-specific porting logs) at the repository root.
  - When modifying a file that already has old/cluttered change-history comments, keep the clean "how it works / effect of changing" parts in the file and move any historical/session notes into `AGENT_NOTES.md`.

---

## 3. Project & Subsystem Map

All team code lives under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
(`FtcRobotController/` contains the stock FTC SDK and samples—leave it untouched.)

### Core Files
| File | Role |
|---|---|
| `RobotMain.java` | Owns all subsystems, `CommandScheduler`, bulk-read caching, telemetry flush (`flushTelemetry()`), and `CommandFactory`. |
| `IronConstants.java` | Robot Controller hardware config names (`CompBot` toggles comp bot vs. bench names), motor tick helpers, field center, reset poses. |
| `HardwareHealth.java` | Driver Station failure/warning board (`HW` telemetry line). |

### Subsystems (Direct access via `robot.<field>`)
| `RobotMain` field | Class / File | What it controls |
|---|---|---|
| `robot.follower` | Pedro `Follower` (`Subsystems/pedro/Constants.java`) | Mecanum drivetrain & fused odometry. |
| `robot.intake` | `Subsystems/Intake.java` | Front intake roller motor (`runCmd()`, `holdCmd()`, `reverseCmd()`, `stopCmd()`). |
| `robot.transfer` | `Subsystems/Transfer.java` | Reservoir / hopper conveyor motor feeding balls toward the flywheel (`runCmd()`, `holdCmd()`, `reverseCmd()`, `stopCmd()`). |
| `robot.flywheel` | `Subsystems/Flywheel.java` | Shooter flywheel motor (`toRpm(rpm)`, `stopCmd()`, `coastToIdleCmd()`, `isAtTargetRpm()`). |
| `robot.PosServ` | `Subsystems/ShootGateServo.java` | Gate servo before the flywheel (`toggleCmd()`, `toPos(pos)`). |
| `robot.ramp` | `Subsystems/Ramp.java` | Dual-servo front ramp (`upCmd()`, `downCmd()`, `toPos(pos)`). |
| `robot.turret` | `Subsystems/turret/TurretPivot.java` | Turret rotation (`C_GotoPositon(deg)`, `C_Stop()`, `MIN_ANGLE_DEG` = CW min, `MAX_ANGLE_DEG` = CCW max, `0` = forward). |
| `robot.hood` | `Subsystems/Hood.java` | Adjustable launcher hood servo (`toAngleCmd(deg)`, `HOOD_DOWN_DEG`, `HOOD_UP_DEG`). |
| `robot.vision` / `robot.visionFusion` | `Subsystems/Vision/` | Limelight 3A AprilTag localization and Kalman pose fusion. |
| `robot.CommandF` | `Commands/CommandFactory.java` | Multi-subsystem shooting commands (`AutoShootAtGoal()`, `ManualShootAtGoal()`, `TrackTarget()`, `FireAtGoal()`), shooting LUTs (`LUT_DISTANCES_IN`, `LUT_RPMS`, `LUT_HOOD_DEGS`), and shoot-readiness checks. |

### TeleOp & Autonomous
| Path | Role |
|---|---|
| `TeleOp/JowByTeleOp.java` | Main competition TeleOp OpMode. |
| `TeleOp/KeyBindings.java` | All driver (`gamepad1`) and operator (`gamepad2`) button/trigger bindings. |
| `Autos/AutoOpModeBase.java` | Base class for all autos (alliance mirroring, `PoseFactory`, `pp` path loader, `autoShoot()`, `rampUp()`, `rampDown()`). |
| `Autos/AutoRegistrar.java` | Registers Red autos in `RED_AUTOS` as both `Red_*` and `Blue_*`. |
| `Autos/ZDiagnostics.java` | Inline robot-function diagnostic auto (hood, turret, flywheel LUT, reservoir/transfer, intake, ramp). |
| `WattageLib/` | Team utilities (`CommandGamepad`, `PPAssets`, `PPFile`, `PedroFollow`, `PanelsFieldRenderer`, `NoFireZones`, `TelemetryFileLogger`). |

---

## 4. Build & Verification

- After any Java edit, verify compilation with Gradle before finishing:
  `./gradlew :TeamCode:compileDebugJavaWithJavac` (or IDE `TeamCode:compileDebugJavaWithJavac`).
- Do not fabricate test results or hardware behavior—if something was only compile-checked, say so clearly.
