# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is an FRC (FIRST Robotics Competition) robot project for **team 5010** targeting the **2026 season**. It is built on WPILib (GradleRIO) and combines two things in one Gradle project:

- **`org.frc5010.common.*`** — a reusable robot library ("FRC5010Lib") providing drivetrains, motor controllers, sensors, vision, telemetry, and a JSON-driven configuration system.
- **`frc.robot.*`** — concrete example robots (`example.ExampleRobot`, `baby_swerve.BabySwerve`) that consume the library.

Logging/telemetry is built on **AdvantageKit** (`LoggedRobot`), so the robot supports deterministic log replay.

## Build & Run Commands

All commands use the Gradle wrapper (`./gradlew`). Java 17 is required (WPILib bundles its own JDK under `~/wpilib/2026`).

| Task | Command |
|------|---------|
| Build | `./gradlew build` |
| Run all tests | `./gradlew test` |
| Run a single test class | `./gradlew test --tests "org.frc5010.common.arch.GenericRobotTest"` |
| Run a single test method | `./gradlew test --tests "org.frc5010.common.arch.GenericRobotTest.methodName"` |
| Simulate (desktop, with sim GUI) | `./gradlew simulateJava` |
| Deploy to roboRIO | `./gradlew deploy` |
| Generate Javadocs (→ `build/docs/javadoc/`) | `./gradlew javadoc` |
| Apply code formatting | `./gradlew spotlessApply` |
| Check formatting | `./gradlew spotlessCheck` |

**Formatting is enforced automatically:** `compileJava` depends on `spotlessApply`, so a normal build reformats sources in place. Spotless uses **google-java-format** for Java, plus rules for `.gradle`, `.json`, and `.md`. Don't fight the formatter; let it rewrite.

`compileJava` also depends on `createVersionFile`, which uses the **gversion** plugin to generate `src/main/java/frc/robot/BuildConstants.java` (git SHA, build date, etc.). This file is generated — do not edit it by hand.

## Architecture

### Startup / robot-selection flow

```
Main → Robot (AdvantageKit LoggedRobot) → RobotContainer → RobotsParser → GenericRobot subclass
```

1. `frc.robot.Robot` (extends `LoggedRobot`) is the WPILib entry point; it wires up AdvantageKit data receivers/replay based on `Constants.CURRENT_MODE`.
2. `RobotContainer` constructs a `RobotsParser`, which reads **`src/main/deploy/robots.json`**.
3. `robots.json` maps a robot **identity** (roboRIO MAC address, or `simulate`/`competition` flags) to a fully-qualified `robotClass`. The matching class (a `GenericRobot` subclass) is instantiated reflectively.
4. The chosen `GenericRobot` subclass (e.g. `frc.robot.example.ExampleRobot`) loads its hardware from a per-robot config directory under `src/main/deploy/<robotDir>/`.

### Config-driven robot construction (the core pattern)

Robots are assembled mostly from **JSON in `src/main/deploy/`**, not hand-wired in Java. A `GenericRobot(String directory)` constructor delegates to:

- **`RobotParser`** (`config/RobotParser.java`) — reads `robot.json`, `controllers.json` (+ `controllers/*.json`, `controllers/axis/*.json`), `cameras.json` (+ `cameras/*.json`), LED strips, Orchestra, and the drivetrain. `driveType` in `robot.json` selects the drivetrain implementation: `YAGSL_SWERVE_DRIVE` (→ `yagsl_drivetrain.json`) or `AKIT_SWERVE_DRIVE` (→ `akit_swerve_drivetrain.json`). Construction is two-phase: the constructor *loads/parses*, then `createRobot()` *instantiates and registers* subsystems.
- **`SubsystemParser`** (`config/SubsystemParser.java`) — parses individual subsystem configs from `<robotDir>/subsystems/` (motors, YAMS arm/elevator/pivot/shooter, etc.).

Each JSON file has a corresponding Java POJO in **`config/json/`** (e.g. `RobotJson`, `CameraConfigurationJson`, `YAGSLDrivetrainJson`) deserialized via Jackson. JSON Schemas live in `config/schemas/` and are wired into VSCode via `.vscode/settings.json` for autocomplete/validation. See `docs/JSON_SCHEMAS.md` for the full schema↔class↔file mapping.

When adding a new configurable device/subsystem, the pattern is: create the JSON schema + a `*Json` POJO in `config/json/`, wire it into the relevant parser, and add a matching deploy JSON file.

### The `arch` package (base abstractions)

`org.frc5010.common.arch` holds the framework base classes everything extends:

- **`GenericRobot`** — base class for all robots (the library entry point). Holds maps of `subsystems`, `controllers`, and `devices` keyed by name; manages alliance color, the auto chooser (`LoggedDashboardChooser`), pose suppliers (real + simulated), and default-command setup. Subclasses override `configureButtonBindings`, `setupDefaultCommands`, `initAutoCommands`, `generateAutoCommand`, etc.
- **`GenericMechanism`**, **`GenericSubsystem`**, **`GenericCommand`**, **`GenericDeviceHandler`** — base types for mechanisms, subsystems, commands, and named-device registries.
- **`Persisted`** — values persisted to NetworkTables. `WpiNetworkTableValuesHelper` / `WpiHelperInterface` provide NT helpers.

### Library subpackages (`org.frc5010.common.*`)

- `drive/` — drivetrains: `swerve/` (incl. `akit/` AdvantageKit-style IO), `swerve_utils/`, `traction/`, `pose/` (pose estimation).
- `motors/` — `hardware/` (vendor motor controllers), `control/`, `function/` (angular/velocity/position control wrappers).
- `sensors/` — `gyro/`, `encoder/`, `absolute_encoder/`, `camera/`, controllers (gamepads).
- `vision/`, `subsystems/`, `commands/` (incl. `calibration/` for SysId), `telemetry/` (Display* helpers over NT/AdvantageKit), `constants/`, `config/`, `utils/`.

### Hardware abstraction

The `example.subsystems` package shows the AdvantageKit **IO pattern** used throughout: a subsystem talks to an `*IO` interface with `*IOReal` and `*IOSim` implementations (e.g. `ExampleIO` / `ExampleIOReal` / `ExampleIOSim`), so simulation and real hardware share one subsystem. `Constants.CURRENT_MODE` (REAL/SIM/REPLAY) drives which path is active.

## Vendor dependencies

Vendor libs are declared in `vendordeps/*.json` (Phoenix6, REVLib, YAGSL, PathPlanner, PhotonLib, AdvantageKit, YAMS, ThriftyLib, ReduxLib, Studica, QuestNav, GrappleFRC). To change a vendor dep version, edit/replace the corresponding file in `vendordeps/`, not `build.gradle`.

## Conventions

- **Javadoc**: public classes/methods are expected to carry Javadoc. See `docs/JAVADOC_GUIDELINES.md` for the house style and templates; `javadoc.yml` publishes docs to GitHub Pages on push to `main`.
- **Named registries over fields**: subsystems/controllers/devices are stored in `GenericRobot`'s maps and fetched by string key (`ConfigConstants` holds the canonical names, e.g. `DRIVETRAIN`, `ALL_LEDS`).
- The `eventDeploy` Gradle task auto-commits working changes when deploying from a branch prefixed `event`.
