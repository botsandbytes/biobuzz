<!--
🤖 AI-RULEZ :: GENERATED FILE — DO NOT EDIT
Project: biobuzz
Generated: 2026-10-08 23:23:10
Source: .ai-rulez/config.toml

NEVER edit this file - modify .ai-rulez/ content instead
Use MCP server: npx -y ai-rulez@latest mcp
Regenerate: ai-rulez generate

Docs: https://github.com/Goldziher/ai-rulez
Content-Hash: blake3:c8daa0602a67fc944d9ba8c8554dbfd5530798adb8ae90d6dbb9ca4d0a3ccc3d
Source-Hash: blake3:3dfb7dd99abf50dc956318f7fc8823b4c9003173833a5851ad84ec0a4245e49b
-->

# biobuzz

AI-powered development governance for FTC biobuzz project

## Rules

### code-quality

**Priority:** high

Follow these coding and design standards in the `biobuzz` codebase:

## Coding Conventions
1. **Dynamic Configuration Over Hardcoding**:
   - Never hardcode mechanical coefficients, tolerances, PID constants, safety margins, velocity scales, or autonomous timing values in Java code.
   - Add or change values in `config.yaml`; access them in Java via the generated `config` facade (e.g. `config.turret.pidf.p`) — do not call the low-level `config.ConfigLoader.load(...)` directly from subsystem code.
   - When adding a new config key, also add its description (and `min`/`max` where applicable) to `config-docs.yaml`, then run `./gradlew generateConfig` (or `./gradlew verifyBuild`) to regenerate `config.java` and `config-schema.json`. Never hand-edit either generated file.
   - If a new key mentions an alliance (`red_*`/`blue_*`), add its mirror for the other alliance or the `checkConfigKeys` Gradle task (part of `verifyBuild`) will fail the build.

2. **Prefer Libraries Over Custom Implementations**:
   - Never write custom code for something a library can already do — even if that library is not yet installed. Identify and add the appropriate dependency first.
   - Use JTS Topology Suite (`org.locationtech.jts.geom.Coordinate`, `Envelope`, `Polygon`) for 2D spatial geometry and polygon intersections instead of custom record types.
   - Use Pedro 3's `com.pedropathing.controllers.PIDController`/`Controller` and `com.pedropathing.api.Paths`/`PoseFactory` instead of any custom PID/PIDF implementations.
   - Use FTC SDK's `org.firstinspires.ftc.robotcore.external.navigation.AngleUnit` normalization methods instead of custom modulo math.
   - Applies to PID controllers, geometry types, angle math, data structures, interpolation, collections, and anything else with a well-supported library equivalent.

3. **Command-Based Subsystems**:
   - Use the `com.pedropathing.ivy.Command` API to structure actions (e.g., `runIntakeCommand`, `shootCommand`), with `.requiring(this)` constraints to prevent resource conflicts. `Robot.update()` runs `Scheduler.execute()` once per loop after subsystem `periodic()`.

4. **Single-Writer Actuator Pattern**:
   - Every actuator has exactly one method that writes to hardware, and it is only ever called from that subsystem's `periodic()` — see `Shooter.periodic()`, `Turret.periodic()`, and `Intake.periodic()` (a documented no-op; intake is a pure pass-through actuator).
   - `Robot.update()` is the only caller of subsystem `periodic()` methods, once per loop, in a fixed order: `Intake`, `Shooter`, `ShotController`, `Turret`.
   - Commands, `ShotController`, and OpModes must only set *target* state (`setTargetPower`, `setAimMode`, `setHoldAngle`, ...) — never call a raw hardware-writing method (e.g. a motor's `setPower`/`setVelocity`) from outside a subsystem's own `periodic()`. Direct writes that bypass `periodic()` get silently overwritten on the next loop and have caused real regressions (a manual-rev button that appeared to do nothing because `periodic()` reverted it one loop later).

5. **Dependency Injection Over Static Global State**:
   - Per-match state (current alliance, goal coordinates, starting poses) is threaded through explicit constructor parameters — `Sentinel(Alliance)`, `Casablanca(Sentinel)`, `Robot(HardwareMap, Telemetry, MatchProfile)` — not mutable public static fields.
   - Do not reintroduce a static "current alliance" or "current match" holder. A prior static-global design caused an initialization-ordering bug where `Sentinel` captured the wrong alliance depending on which OpMode constructed it first; constructor injection makes that class of bug structurally impossible and makes the safety/geometry classes independently unit-testable.

6. **Safety & Collision Avoidance**:
   - Do not bypass `Sentinel` or `Casablanca` protections during automated or manual driving sequences.
   - `Sentinel` and `Casablanca` are per-match instances owned by `Robot` (see rule 5) — always get them via `robot.sentinel` / `robot.casablanca`, never construct a second instance for the same match.
   - Any modifications to the safety calculations should use JTS geometry checks, and depth (X-axis) vs. side (Y-axis) protection must stay symmetric — if you change one axis's bounds wiring in `Casablanca.adjustDriveInput`, mirror it in the other.

7. **Modern Java Features**:
   - Prefer modern Java syntax like `switch` expressions and `record` types for simple data containers (e.g. `MatchProfile`, `PIDGains` inside the `records` package).

8. **Autotuning & Calibration**:
   - For relay-based PID autotuning, extend `AutotuneOpMode` (which wraps `PIDAutotuner`) rather than reimplementing the oscillation/telemetry loop — implement its three abstract methods (`createAutotuner`, `readCurrentValue`, `applyPower`); see `ShooterAutotuneOpMode` for an example.
   - For non-PID calibration (e.g. finding a friction/kS constant via a power ramp), model the OpMode after `FrictionCalibrationOpMode` instead — it is not a `PIDAutotuner` use case.
   - Pedro 3 drivetrain, Pinpoint, and Foresight tuning lives in `pedro/Tuning.java` as `@Tuner` procedures. Copy measured values into `config.yaml`, document the keys in `config-docs.yaml`, and regenerate the facade.

9. **Mathematical & Algorithmic Unit Test Coverage**:
   - High overall global code coverage (e.g., 80% across the entire codebase) is not required.
   - However, all pure mathematical functions, geometry collision calculations (e.g., coordinate rotations, bounding box intersections in `Sentinel`/`Casablanca`), and numeric helpers (e.g., yaw angle wrapping in `Turret`) **must** have full local JVM unit test coverage to prevent logic regressions. See `MathSafetyTest` for the current baseline.

10. **Verification Task Workflow**:
    - Prefer running the custom task `./gradlew verifyBuild` to run the exact same verification checks as a standard build (Spotless auto-formatting, compiler error checks, local JVM tests, and the `checkConfigKeys` config-symmetry check) without deploying the application.
    - Always run `./gradlew` commands outside the terminal sandbox (`BypassSandbox: true`) because Gradle requires access to global cache lock files (e.g. `~/.gradle/wrapper/dists/...`) which are restricted by standard sandboxing.
    - For live tuning on a connected robot without a full APK rebuild, use `./gradlew pushConfig` to hot-deploy `config.yaml` over ADB (re-init the OpMode to pick it up) and `./gradlew resetConfig` to revert to the bundled config. This is a deployment convenience, not part of `verifyBuild`.

## Context

### architecture

This project is a First Tech Challenge (FTC) robot controller application written in Java.

## Core Architecture Blocks

1. **Subsystem Decomposition**:
   - Robot functionality is split into modular subsystems located under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/`.
   - Key subsystems:
     - `Intake.java`: Front and middle roller control.
     - `Shooter.java`: Double flywheel and hood positioning. Requires `shooter.max_rpm` from config. Exposes only `setTargetPower(double)`; the flywheel is driven exclusively from `periodic()`.
     - `Turret.java`: CRServo-driven turret rotation, positioned via an absolute analog encoder plus the `Follower`'s pose supplier (no separate IMU). Aiming is driven by an `AimMode` (`IDLE` / `HOLD` / `AIM_AT_GOAL`) set via `setAimMode()`/`setHoldAngle()`, resolved once per loop in `periodic()`.
     - `ShotController.java`: Owns the shot lifecycle (`startShot`/`stopShot`/`periodic()`) — gates the intake feed on flywheel velocity and (optionally) turret alignment, and drives the Turret's `AimMode` transitions. Runs its trajectory solve (`ballistics.ShotSolver`) on a dedicated background thread, not the main loop.
     - `Lumos.java`: goBILDA RGB indicator light via `ServoImplEx` with PWM range control. Colors are a `Lumos.Color` enum, not one method per color.
   - Subsystem behaviors are orchestrated using the `com.pedropathing.ivy.Command` command framework.
   - `VisionUtil` (`utilities/`) is not a `periodic()`-driven subsystem — it's polled directly by the OpMode (`TeleOpBase`) for AprilTag-based pose correction (`updateAprilTagPose()`/`isTagFound()`), streaming toggled on/off around when a fix is needed.

2. **Robot Container (`Robot.java`)**:
   - `Robot` is the single composition root for a match: it builds the cached `Follower`, all subsystems (`Intake`, `Shooter`, `Turret`), `Sentinel`, `Casablanca`, `ShotController`, and `VisionUtil`, and is constructed once per OpMode from a `MatchProfile` (see block 7).
   - `Robot.update()` is called once per loop and is the *only* place subsystem state advances: it clears the bulk cache, updates the `Follower`, then calls `periodic()` on `Intake`, `Shooter`, `ShotController`, and `Turret` in that order, then runs `Scheduler.execute()`.
   - Commands and controllers only set target state (e.g. `shooter.setTargetPower(x)`, `turret.setAimMode(...)`); they never write to hardware directly. This "single writer per actuator" contract is what `periodic()` exists to enforce — see the code-quality rule of the same name.
   - `Robot.shutdown()` stops `ShotController`'s background solver thread. It must be called from the owning OpMode's teardown (`stop()`) — a new `Robot` is constructed on every OpMode init, so skipping this leaks one thread per init.

3. **Drivetrain & Pathing (Pedro Pathing)**:
   - Drivetrain control, localizer, and path-following are managed by **Pedro Pathing**.
   - Pedro 3.0.1 uses `Follower`, `Foresight`, `Mecanum`, `PinpointLocalizer`, `com.pedropathing.math.Pose`, and `com.pedropathing.api.Paths`. `pedro/Constants.java` builds them from generated `config.pedro` values and provides the QuickStart-compatible `create(HardwareMap)` entry point.
   - `Constants.create(hardwareMap)` is the standard entry point; Pedro 3 Mecanum caches motor writes itself. Bulk read caching (`LynxModule.BulkCachingMode.MANUAL`) is cleared once per loop in `Robot.update()`. `pedro/Tuning.java` registers AutoTune procedures for this drivetrain, localizer, and Foresight.

4. **Ballistics (Shot Solving)**:
   - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ballistics/` holds the trajectory model used to aim and spin up the shooter: `BallisticsModel`, `ShotSolver`, `ShotTable`, `ShotTimeTable`, `FlywheelFeedforwardFit`.
   - `ShotSolver.solve(...)` is a trajectory-optimization search that is too slow to run synchronously in `periodic()` on every input; `ShotController` runs it continuously on a dedicated daemon thread (`solverThread`) instead. `periodic()` only ever does a cheap `volatile` write to publish the latest `ShotInputs` and a `volatile` read to pick up the most recently completed `ShotSolution` — never call `ShotSolver.solve()` directly from `periodic()` or any control-loop path, and any new state shared between `periodic()` and the solver thread must be `volatile` or guarded the same way (see `ShotController`'s `solveLock`/`pendingInputs`).
   - `FlywheelDipDetector` counts balls fired by the flywheel's brief RPM dip on each ball, measured against a running reference of recent velocity — not the commanded setpoint, since the wheel doesn't recover between shots during a burst.

5. **PID Anti-Windup**:
   - Pedro 3's `PIDController` integral term has no clamp; on a flywheel this causes overshoot during spin-up. `Shooter` uses a separate `AntiWindupIntegrator` (`utilities/`) for the I term instead.
   - When `AntiWindupIntegrator` is in use, the `PIDController` **must** have `kI = 0` to avoid double-counting. Flywheel feedforward is applied separately from the PID controller.

6. **Collision Avoidance (Sentinel & Casablanca)**:
   - `Sentinel` and `Casablanca` (`utilities/`) are per-match **instances** owned by `Robot`, not static utility classes: `new Sentinel(alliance)` computes the goal zones and launch zones once, and `new Casablanca(sentinel)` reads its friction/smoothing/protection config from it. There is no static "current alliance" global — alliance flows in through the constructor via `MatchProfile`.
   - `Sentinel` checks robot footprint geometry against protected alliance goal zones using JTS Topology Suite (`org.locationtech.jts.geom.Polygon.intersects()`). This provides robust, instant 2D polygon collision detection under pure JUnit 4 unit tests without requiring Android Robolectric wrappers.
   - `Casablanca` dynamically reduces velocity or repels the robot to prevent violating opponent goals, using `Casablanca.brakingDistance` with the prior calibrated linear/quadratic stopping model because Pedro 3 removed `PredictiveBrakingController`. Depth (X-axis) and side (Y-axis) protection are computed by the same `calculateAxisState` helper called twice with the appropriately-swapped bounds — keep the axis pairing symmetric when touching this code (see `MathSafetyTest#testCasablancaSymmetryAndCollisionMath`).

7. **Alliance-Parameterized OpModes**:
   - There is one `TeleOp`/`TeleOpBase` pair, not per-alliance TeleOp files. It reads `Alliance` from the blackboard (falling back to `RED`), builds a `MatchProfile` via `config.loadMatchProfile(alliance)`, and constructs `Robot` from it.
   - Autonomous logic lives in a generic abstract base, `AllianceAutoBase<T>`, extended by two alliance-agnostic subclasses, `AllianceAutoNew` and `AllianceOppositeNew`, each taking an `Alliance` in its constructor and loading its poses via `config` (see block 9).
   - `UnifiedAutos.java` registers the four concrete OpModes (`Blue/Red` × `AutoNew/OppositeNew`) as thin nested subclasses annotated with the FTC SDK's `@Autonomous` — there are no hand-duplicated per-alliance auto files.
   - Each auto/teleop OpMode persists its final pose to the blackboard (`"RED_POSE"` / `"BLUE_POSE"`) so the next OpMode (typically TeleOp) can resume from the correct field position.

8. **Dynamic Configuration System**:
   - Config lives in one YAML file, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml`, with human-readable descriptions and min/max constraints kept separately in the sibling `config-docs.yaml` (keyed by leaf name).
   - Runtime loading is handled by the `config-compiler` Gradle module (`org.firstinspires.ftc.teamcode.config.ConfigLoader`), a small reflection-based YAML→POJO binder. It prefers an ADB-pushed override at `/sdcard/FIRST/teamcode/config.yaml` over the bundled classpath resource, so config can be hot-reloaded without an APK rebuild (`./gradlew pushConfig` / `./gradlew resetConfig`, backed by the pure-Java `ConfigPusherMain` — no Python dependency).
   - `config.java` (under `robot/config/generated/`) is a **generated** typed facade over that loader — nested static classes mirroring the YAML structure — and a `loadMatchProfile(Alliance)` helper. It is regenerated from `config.yaml` + `config-docs.yaml` by the pure Java generator `ConfigGeneratorMain` (`./gradlew generateConfig`), which also regenerates `config-schema.json`; never hand-edit `config.java` or `config-schema.json`.
   - `checkConfigKeys` (wired into the `checkConfigKeys` Gradle task, part of `verifyBuild`) is a pure Java JavaExec task that statically checks `config.yaml` for alliance-name asymmetry (a `red_*` key with no `blue_*` mirror) and fuzzy near-duplicate keys, failing the build on the former.

9. **Dedicated Records Package**:
   - The `org.firstinspires.ftc.teamcode.records` package contains simple, immutable record types and structural data elements:
     - `PIDGains`: Holder for kp, ki, and kd gains.
     - `Alliance`: RED / BLUE enum.
     - `Field`: Dynamic goal coordinates derived from active alliance.
     - `MatchProfile`: Bundles `Alliance`, goal X/Y, and the four match poses (start/score/drink/park) for a given alliance; always passed into `Robot`'s constructor. `TeleOpBase` builds it via `config.loadMatchProfile(Alliance)`; `AllianceAutoNew`/`AllianceOppositeNew` build their own from the merged `auto_poses.*` config since their pose sets differ from TeleOp's. There is no static "current alliance/current match" global anywhere in the codebase — it is always threaded through explicitly via `MatchProfile`.
     - `ShotInputs` / `ShotSolution` / `BallisticsParameters` / `EndgameSpot`: immutable data carried between `ShotController` and the `ballistics` package (block 4).

10. **Test Layout — Two Different "tests"**:
    - `TeamCode/src/test/java/...`: real JUnit 4 unit tests, run by `./gradlew testDebugUnitTest` (part of `verifyBuild`). This is where new pure-math/logic coverage (rule 9 of code-quality) belongs.
    - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/`: on-robot calibration and tuning **OpModes** (e.g. `FrictionCalibrationOpMode`, `ShooterAutotuneOpMode`, `TurretCenterCalibrationOpMode`) — despite the package name, these are not JUnit tests and don't run under `testDebugUnitTest`. Don't confuse a request to "add a test" between the two; ask or infer from context which is meant.
