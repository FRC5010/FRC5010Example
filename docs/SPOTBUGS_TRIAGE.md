# SpotBugs Triage Report

Generated from `./gradlew spotbugsMain` (SpotBugs plugin 6.0.26, effort MAX, default
confidence) on branch `claude/init-78wjxp`. Regenerate any time with
`./gradlew spotbugsMain`; reports land in `build/reports/spotbugs/` (HTML + XML).
Generated/vendored sources (`BuildConstants`, `LimelightHelpers`) are excluded via
`config/spotbugs/exclude.xml`.

**Total: 418 findings — 26 high priority (P1), 392 medium (P2).**

| # | Category | Findings | P1 | Priority |
|---|----------|----------|----|----------|
| 1 | Correctness bugs | 41 | 5 | HIGH |
| 2 | Static state & concurrency | 58 | 8 | HIGH-MED |
| 3 | Dead / unused code | 65 | 2 | MED |
| 4 | Exposed mutable internals & API design | 223 | 10 | LOW-MED |
| 5 | Style & naming | 31 | 1 | LOW |

Each category has a corresponding GitHub issue. P1 = SpotBugs high-confidence,
listed first within each section.


## 1. Correctness bugs (41 findings, priority HIGH)

Potential real bugs: possible null dereferences, uninitialized reads, wrong map iteration, switch fallthrough, ignored return values on side-effect-free methods. Each finding should be read and either fixed or explicitly dismissed.

**Caveat:** the 21 `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` findings concentrate in `config/json/**` POJOs whose public fields are written **reflectively by Jackson** — SpotBugs cannot see those writes, so most are false positives. Verify, then move that pattern (scoped to the config package) into `config/spotbugs/exclude.xml`.

**Patterns:** `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` (21), `WMI_WRONG_MAP_ITERATOR` (4), `RV_RETURN_VALUE_IGNORED_NO_SIDE_EFFECT` (3), `NP_NULL_ON_SOME_PATH_FROM_RETURN_VALUE` (3), `UR_UNINIT_READ` (2), `NP_UNWRITTEN_FIELD` (2), `NS_DANGEROUS_NON_SHORT_CIRCUIT` (1), `NP_OPTIONAL_RETURN_NULL` (1), `SF_SWITCH_FALLTHROUGH` (1), `CNT_ROUGH_CONSTANT_VALUE` (1), `UWF_NULL_FIELD` (1), `BC_VACUOUS_INSTANCEOF` (1)

| P | Pattern | Location |
|---|---------|----------|
| 1 | `NS_DANGEROUS_NON_SHORT_CIRCUIT` | `org/frc5010/common/arch/GenericCommandSequence.java:76` |
| 1 | `NP_OPTIONAL_RETURN_NULL` | `org/frc5010/common/drive/GenericDrivetrain.java:79` |
| 1 | `RV_RETURN_VALUE_IGNORED_NO_SIDE_EFFECT` | `org/frc5010/common/motors/function/GenericControlledMotor.java:325` |
| 1 | `UR_UNINIT_READ` | `org/frc5010/common/subsystems/LEDStripSegment.java:226` |
| 1 | `UR_UNINIT_READ` | `org/frc5010/common/subsystems/LEDStripSegment.java:226` |
| 2 | `RV_RETURN_VALUE_IGNORED_NO_SIDE_EFFECT` | `frc/robot/example/subsystems/ExampleSubsystem.java:45` |
| 2 | `NP_NULL_ON_SOME_PATH_FROM_RETURN_VALUE` | `org/frc5010/common/arch/StateMachine.java:165` |
| 2 | `NP_NULL_ON_SOME_PATH_FROM_RETURN_VALUE` | `org/frc5010/common/arch/StateMachine.java:170` |
| 2 | `NP_NULL_ON_SOME_PATH_FROM_RETURN_VALUE` | `org/frc5010/common/arch/StateMachine.java:184` |
| 2 | `SF_SWITCH_FALLTHROUGH` | `org/frc5010/common/arch/WpiNetworkTableValuesHelper.java:387` |
| 2 | `WMI_WRONG_MAP_ITERATOR` | `org/frc5010/common/arch/WpiNetworkTableValuesHelper.java:489` |
| 2 | `CNT_ROUGH_CONSTANT_VALUE` | `org/frc5010/common/commands/DriveToPoseSupplier.java:66` |
| 2 | `NP_UNWRITTEN_FIELD` | `org/frc5010/common/commands/DriveToTrajectory.java:62` |
| 2 | `NP_UNWRITTEN_FIELD` | `org/frc5010/common/commands/DriveToTrajectory.java:121` |
| 2 | `UWF_NULL_FIELD` | `org/frc5010/common/commands/calibration/WheelRadiusCharacterization.java:37` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/AKitSwerveDrivetrainJson.java:70` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/AKitSwerveDrivetrainJson.java:73` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/CameraConfigurationJson.java:310` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/DriveteamControllersJson.java:50` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/DriveteamControllersJson.java:61` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/GamePiecesJson.java:18` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/GamePiecesJson.java:19` |
| 2 | `WMI_WRONG_MAP_ITERATOR` | `org/frc5010/common/config/json/RobotsJson.java:43` |
| 2 | `WMI_WRONG_MAP_ITERATOR` | `org/frc5010/common/config/json/VisionPropertiesJson.java:63` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/VisionPropertiesJson.java:102` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/YAGSLDrivetrainJson.java:49` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/devices/GyroSettingsConfigurationJson.java:29` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/devices/SubsystemJson.java:37` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/devices/YamsConfigCommon.java:147` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:79` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:94` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:115` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/swerve/AkitSwerveConfig.java:141` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/swerve/AkitSwerveConfig.java:142` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/swerve/AkitSwerveConfig.java:196` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:251` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:394` |
| 2 | `NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:499` |
| 2 | `BC_VACUOUS_INSTANCEOF` | `org/frc5010/common/sensors/absolute_encoder/RevAbsoluteEncoder.java:39` |
| 2 | `RV_RETURN_VALUE_IGNORED_NO_SIDE_EFFECT` | `org/frc5010/common/sensors/gyro/NavXGyro.java:38` |
| 2 | `WMI_WRONG_MAP_ITERATOR` | `org/frc5010/common/subsystems/SegmentedLedSystem.java:67` |


## 2. Static state & concurrency (58 findings, priority HIGH-MED)

Instance methods writing static fields (34 sites), unsynchronized singleton getters, lazy static init races, a thread started in a constructor, and constructors that can throw partially-initialized. This independently confirms the static-state problems found during the offseason architecture review (see the deferred Phase 3 de-static work: `YAGSLSwerveDrivetrain.swerveDrive`, `RobotParser` statics, `GenericRobot.subsystemParser`). Fixing these is refactoring work, best done deliberately with the test suite as a guard — not batch-mechanical.

**Patterns:** `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` (34), `CT_CONSTRUCTOR_THROW` (8), `MS_CANNOT_BE_FINAL` (8), `SING_SINGLETON_GETTER_NOT_SYNCHRONIZED` (3), `LI_LAZY_INIT_STATIC` (3), `SC_START_IN_CTOR` (1), `SING_SINGLETON_HAS_NONPRIVATE_CONSTRUCTOR` (1)

| P | Pattern | Location |
|---|---------|----------|
| 1 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `frc/robot/RobotContainer.java:20` |
| 1 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/arch/GenericRobot.java:95` |
| 1 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/arch/GenericRobot.java:260` |
| 1 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/auto/pathplanner/PathFinderCommand.java:281` |
| 1 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/auto/pathplanner/PathFinderCommand.java:449` |
| 1 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/drive/swerve/GenericSwerveDrivetrain.java:84` |
| 1 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/drive/swerve/akit/ModuleIOSpark.java:202` |
| 1 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/drive/swerve/akit/ModuleIOSparkTalon.java:180` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `frc/robot/Robot.java:81` |
| 2 | `CT_CONSTRUCTOR_THROW` | `org/frc5010/common/arch/GenericCommandSequence.java:36` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/arch/GenericRobot.java:292` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/auto/pathplanner/PathFinderCommand.java:129` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/auto/pathplanner/PathFinderCommand.java:178` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/auto/pathplanner/PathFinderCommand.java:291` |
| 2 | `CT_CONSTRUCTOR_THROW` | `org/frc5010/common/config/RobotParser.java:90` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/config/RobotParser.java:100` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/config/RobotParser.java:101` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/config/RobotParser.java:106` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/config/RobotParser.java:107` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/config/RobotParser.java:124` |
| 2 | `CT_CONSTRUCTOR_THROW` | `org/frc5010/common/config/RobotsParser.java:30` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/config/json/AKitSwerveDrivetrainJson.java:86` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/config/json/AKitSwerveDrivetrainJson.java:99` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/config/json/RobotJson.java:82` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/config/json/RobotJson.java:83` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/config/json/RobotJson.java:84` |
| 2 | `MS_CANNOT_BE_FINAL` | `org/frc5010/common/constants/Constants.java:19` |
| 2 | `MS_CANNOT_BE_FINAL` | `org/frc5010/common/constants/Constants.java:20` |
| 2 | `MS_CANNOT_BE_FINAL` | `org/frc5010/common/constants/Constants.java:21` |
| 2 | `CT_CONSTRUCTOR_THROW` | `org/frc5010/common/drive/DifferentialDrivetrain.java:61` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/drive/DifferentialDrivetrain.java:166` |
| 2 | `MS_CANNOT_BE_FINAL` | `org/frc5010/common/drive/GenericDrivetrain.java:78` |
| 2 | `MS_CANNOT_BE_FINAL` | `org/frc5010/common/drive/swerve/SwerveDriveFunctions.java:38` |
| 2 | `MS_CANNOT_BE_FINAL` | `org/frc5010/common/drive/swerve/SwerveDriveFunctions.java:39` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/drive/swerve/YAGSLSwerveDrivetrain.java:98` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/drive/swerve/YAGSLSwerveDrivetrain.java:102` |
| 2 | `CT_CONSTRUCTOR_THROW` | `org/frc5010/common/drive/swerve/YAGSLSwerveDrivetrain.java:105` |
| 2 | `SC_START_IN_CTOR` | `org/frc5010/common/drive/swerve/akit/AkitSwerveDrive.java:132` |
| 2 | `CT_CONSTRUCTOR_THROW` | `org/frc5010/common/drive/swerve/akit/ModuleIOTalonFX.java:114` |
| 2 | `SING_SINGLETON_GETTER_NOT_SYNCHRONIZED` | `org/frc5010/common/drive/swerve/akit/PhoenixOdometryThread.java:47` |
| 2 | `LI_LAZY_INIT_STATIC` | `org/frc5010/common/drive/swerve/akit/PhoenixOdometryThread.java:51` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/drive/swerve/akit/PhoenixOdometryThread.java:58` |
| 2 | `LI_LAZY_INIT_STATIC` | `org/frc5010/common/drive/swerve/akit/SparkOdometryThread.java:33` |
| 2 | `SING_SINGLETON_GETTER_NOT_SYNCHRONIZED` | `org/frc5010/common/drive/swerve/akit/SparkOdometryThread.java:39` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/drive/swerve/akit/SparkOdometryThread.java:44` |
| 2 | `SING_SINGLETON_GETTER_NOT_SYNCHRONIZED` | `org/frc5010/common/drive/swerve/akit/TalonFXOdometryThread.java:43` |
| 2 | `LI_LAZY_INIT_STATIC` | `org/frc5010/common/drive/swerve/akit/TalonFXOdometryThread.java:47` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/drive/swerve/akit/TalonFXOdometryThread.java:55` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/drive/swerve/akit/TalonFXOdometryThread.java:56` |
| 2 | `MS_CANNOT_BE_FINAL` | `org/frc5010/common/drive/swerve/akit/util/SparkUtil.java:19` |
| 2 | `CT_CONSTRUCTOR_THROW` | `org/frc5010/common/motors/control/TalonFXController.java:44` |
| 2 | `CT_CONSTRUCTOR_THROW` | `org/frc5010/common/sensors/absolute_encoder/RevAbsoluteEncoder.java:45` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/sensors/camera/QuestNavInterface.java:245` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/sensors/camera/QuestNavInterface.java:246` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/sensors/camera/QuestNavInterface.java:293` |
| 2 | `ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD` | `org/frc5010/common/sensors/camera/SimulatedCamera.java:58` |
| 2 | `SING_SINGLETON_HAS_NONPRIVATE_CONSTRUCTOR` | `org/frc5010/common/subsystems/PhysicsSim.java:8` |
| 2 | `MS_CANNOT_BE_FINAL` | `org/frc5010/common/vision/AprilTags.java:197` |


## 3. Dead / unused code (65 findings, priority MED)

Fields that are never read, never written, or entirely unused, plus dead local stores and an uncalled private method. Continuation of the Phase 0 dead-code cleanup. Mostly safe deletions, but beware fields read reflectively (Jackson POJOs) or by dashboards.

**Patterns:** `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` (21), `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` (12), `URF_UNREAD_FIELD` (11), `UUF_UNUSED_FIELD` (7), `UUF_UNUSED_PUBLIC_OR_PROTECTED_FIELD` (6), `DLS_DEAD_LOCAL_STORE` (4), `UWF_UNWRITTEN_FIELD` (3), `UPM_UNCALLED_PRIVATE_METHOD` (1)

| P | Pattern | Location |
|---|---------|----------|
| 1 | `DLS_DEAD_LOCAL_STORE` | `org/frc5010/common/commands/DriveToTrajectory.java:62` |
| 1 | `DLS_DEAD_LOCAL_STORE` | `org/frc5010/common/commands/SmartSwerveDrive.java:102` |
| 2 | `UUF_UNUSED_FIELD` | `frc/robot/baby_swerve/BabySwerve.java:None` |
| 2 | `URF_UNREAD_FIELD` | `frc/robot/baby_swerve/BabySwerve.java:19` |
| 2 | `UUF_UNUSED_FIELD` | `frc/robot/example/ExampleRobot.java:None` |
| 2 | `URF_UNREAD_FIELD` | `frc/robot/example/ExampleRobot.java:21` |
| 2 | `URF_UNREAD_FIELD` | `frc/robot/example/ExampleRobot.java:28` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `frc/robot/example/subsystems/ExampleIOReal.java:36` |
| 2 | `UUF_UNUSED_PUBLIC_OR_PROTECTED_FIELD` | `frc/robot/example/subsystems/ExampleIOSim.java:None` |
| 2 | `UUF_UNUSED_PUBLIC_OR_PROTECTED_FIELD` | `frc/robot/example/subsystems/ExampleIOSim.java:None` |
| 2 | `UUF_UNUSED_FIELD` | `org/frc5010/common/commands/DriveToPoseSupplier.java:None` |
| 2 | `URF_UNREAD_FIELD` | `org/frc5010/common/commands/DriveToPoseSupplier.java:42` |
| 2 | `URF_UNREAD_FIELD` | `org/frc5010/common/commands/DriveToPoseSupplier.java:75` |
| 2 | `URF_UNREAD_FIELD` | `org/frc5010/common/commands/DriveToPoseSupplier.java:77` |
| 2 | `DLS_DEAD_LOCAL_STORE` | `org/frc5010/common/commands/DriveToPoseSupplier.java:230` |
| 2 | `UUF_UNUSED_FIELD` | `org/frc5010/common/commands/DriveToTrajectory.java:None` |
| 2 | `UUF_UNUSED_FIELD` | `org/frc5010/common/commands/DriveToTrajectory.java:None` |
| 2 | `URF_UNREAD_FIELD` | `org/frc5010/common/commands/DriveToTrajectory.java:37` |
| 2 | `URF_UNREAD_FIELD` | `org/frc5010/common/commands/DriveToTrajectory.java:57` |
| 2 | `UWF_UNWRITTEN_FIELD` | `org/frc5010/common/commands/DriveToTrajectory.java:62` |
| 2 | `DLS_DEAD_LOCAL_STORE` | `org/frc5010/common/commands/DriveToTrajectory.java:66` |
| 2 | `URF_UNREAD_FIELD` | `org/frc5010/common/commands/DriveToTrajectory.java:84` |
| 2 | `URF_UNREAD_FIELD` | `org/frc5010/common/commands/DriveToTrajectory.java:85` |
| 2 | `UWF_UNWRITTEN_FIELD` | `org/frc5010/common/commands/DriveToTrajectory.java:121` |
| 2 | `UPM_UNCALLED_PRIVATE_METHOD` | `org/frc5010/common/commands/calibration/WheelRadiusCharacterization.java:83` |
| 2 | `UUF_UNUSED_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/GamePieceJson.java:None` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/GamePiecesJson.java:19` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/devices/MotorSetupJson.java:53` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/devices/MotorSetupJson.java:53` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/devices/SubsystemJson.java:38` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/devices/SubsystemJson.java:40` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/config/json/devices/YamsConfigCommon.java:142` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:190` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:195` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:200` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:205` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:210` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:215` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:220` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/pose/GenericPose.java:28` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/swerve/AkitSwerveConfig.java:141` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/swerve/AkitSwerveConfig.java:141` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/swerve/AkitSwerveConfig.java:142` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/swerve/AkitSwerveConfig.java:194` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/drive/swerve/GenericSwerveModuleInfo.java:28` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/function/GenericControlledMotor.java:104` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/function/PercentControlMotor.java:65` |
| 2 | `UUF_UNUSED_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/hardware/GenericTalonFXMotor.java:None` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/hardware/GenericTalonFXMotor.java:508` |
| 2 | `UUF_UNUSED_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:None` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:238` |
| 2 | `UWF_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:263` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:509` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/sensors/absolute_encoder/GenericAbsoluteEncoder.java:17` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/sensors/camera/GenericCamera.java:31` |
| 2 | `UUF_UNUSED_FIELD` | `org/frc5010/common/sensors/camera/QuestNavInterface.java:None` |
| 2 | `UUF_UNUSED_FIELD` | `org/frc5010/common/sensors/camera/QuestNavInterface.java:None` |
| 2 | `UWF_UNWRITTEN_FIELD` | `org/frc5010/common/sensors/camera/QuestNavInterface.java:276` |
| 2 | `URF_UNREAD_FIELD` | `org/frc5010/common/sensors/encoder/SimulatedEncoder.java:15` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/subsystems/CameraSystem.java:56` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/subsystems/FiducialTargetSystem.java:18` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/subsystems/VisibleTargetSystem.java:19` |
| 2 | `UUF_UNUSED_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/telemetry/DisplayAngle.java:None` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/telemetry/DisplayValuesHelper.java:102` |
| 2 | `URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD` | `org/frc5010/common/telemetry/DisplayVoltage.java:107` |


## 4. Exposed mutable internals & API design (223 findings, priority LOW-MED)

Returning/storing mutable internals (`EI_EXPOSE_REP`/`EI_EXPOSE_REP2`), mutable statics, public primitive attributes, mutable enum fields. For robot code, much of this is accepted practice — and the `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` findings in `config/json/**` are **intentional** (Jackson deserializes public fields). Recommended handling: exclude the config POJO package for these patterns in `config/spotbugs/exclude.xml`, then review the remainder case-by-case where a defensive copy is genuinely warranted (e.g., arrays/collections handed across subsystem boundaries).

**Patterns:** `EI_EXPOSE_REP2` (78), `EI_EXPOSE_REP` (70), `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` (32), `MS_SHOULD_BE_FINAL` (13), `MS_PKGPROTECT` (9), `MS_EXPOSE_REP` (7), `ME_MUTABLE_ENUM_FIELD` (7), `MF_CLASS_MASKS_FIELD` (4), `MS_MUTABLE_ARRAY` (1), `EI_EXPOSE_STATIC_REP2` (1), `MS_FINAL_PKGPROTECT` (1)

| P | Pattern | Location |
|---|---------|----------|
| 1 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/arch/WpiNetworkTableValuesHelper.java:23` |
| 1 | `MF_CLASS_MASKS_FIELD` | `org/frc5010/common/drive/SwerveDriveConfig.java:None` |
| 1 | `MF_CLASS_MASKS_FIELD` | `org/frc5010/common/motors/function/AngularControlMotor.java:None` |
| 1 | `MF_CLASS_MASKS_FIELD` | `org/frc5010/common/sensors/camera/SimulatedCamera.java:None` |
| 1 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/sensors/camera/SimulatedCamera.java:23` |
| 1 | `MF_CLASS_MASKS_FIELD` | `org/frc5010/common/subsystems/AprilTagPoseSystem.java:None` |
| 1 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/vision/AprilTags.java:53` |
| 1 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/vision/AprilTags.java:205` |
| 1 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/vision/VisionConstants.java:10` |
| 1 | `MS_MUTABLE_ARRAY` | `org/frc5010/common/vision/VisionConstants.java:23` |
| 2 | `MS_EXPOSE_REP` | `frc/robot/Robot.java:85` |
| 2 | `EI_EXPOSE_REP2` | `frc/robot/example/commands/ExampleCommands.java:47` |
| 2 | `EI_EXPOSE_REP2` | `frc/robot/example/subsystems/ExampleIOReal.java:36` |
| 2 | `EI_EXPOSE_REP2` | `frc/robot/example/subsystems/ExampleIOReal.java:37` |
| 2 | `MS_PKGPROTECT` | `org/frc5010/common/arch/GenericRobot.java:None` |
| 2 | `MS_PKGPROTECT` | `org/frc5010/common/arch/GenericRobot.java:None` |
| 2 | `MS_PKGPROTECT` | `org/frc5010/common/arch/GenericRobot.java:72` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/arch/GenericRobot.java:72` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/arch/GenericRobot.java:171` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/arch/GenericRobot.java:443` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/arch/GenericRobot.java:457` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/arch/GenericRobot.java:467` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/arch/GenericSubsystem.java:75` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/arch/GenericSubsystem.java:84` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/arch/GenericSubsystem.java:170` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/arch/StateMachine.java:134` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/arch/StateMachine.java:380` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/auto/AutoSequence.java:43` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/DefaultDriveCommand.java:49` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/DriveByAngle.java:64` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/DriveToPoseSupplier.java:117` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/DriveToPosition.java:118` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/DriveToTrajectory.java:51` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/JoystickToSwerve.java:34` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/LedBlink.java:23` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/LedColor.java:21` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/LedDefaultCommand.java:19` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/SmartSwerveDrive.java:34` |
| 2 | `EI_EXPOSE_STATIC_REP2` | `org/frc5010/common/commands/calibration/PoseProviderAutoOffset.java:34` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/commands/calibration/WheelRadiusCharacterization.java:57` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/RobotsParser.java:41` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/config/SubsystemParser.java:33` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/config/json/CameraConfigurationJson.java:106` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/config/json/devices/MotorSetupJson.java:34` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/AngleUnit.java:41` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/AngularAccelerationUnit.java:52` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/AngularVelocityUnit.java:53` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/CurrentUnit.java:41` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/DistanceUnit.java:44` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/LinearAccelerationUnit.java:106` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/LinearVelocityUnit.java:56` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/MassUnit.java:45` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/MomentOfInertiaUnit.java:39` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/TimeUnit.java:45` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/config/units/VoltageUnit.java:42` |
| 2 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/constants/RobotConstantsDef.java:17` |
| 2 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/constants/RobotConstantsDef.java:18` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/constants/SwerveConstants.java:95` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/constants/SwerveConstants.java:99` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/constants/SwerveModuleConstants.java:125` |
| 2 | `MS_PKGPROTECT` | `org/frc5010/common/drive/DifferentialDrivetrain.java:None` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/DifferentialDrivetrain.java:62` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/DifferentialDrivetrain.java:63` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/DifferentialDrivetrain.java:66` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/DrivetrainConfig.java:209` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/DrivetrainConfig.java:218` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/DrivetrainConfig.java:432` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/DrivetrainConfig.java:444` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/GenericDrivetrain.java:131` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/GenericDrivetrain.java:140` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/GenericDrivetrain.java:151` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/GenericDrivetrain.java:361` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/GenericDrivetrain.java:362` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/GenericDrivetrain.java:363` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/GenericDrivetrain.java:364` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/GenericDrivetrain.java:372` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/GenericDrivetrain.java:373` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/GenericDrivetrain.java:374` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/GenericDrivetrain.java:375` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/GenericDrivetrain.java:387` |
| 2 | `MS_PKGPROTECT` | `org/frc5010/common/drive/GenericDrivetrain.java:401` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/SwerveDriveConfig.java:142` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/SwerveDriveConfig.java:157` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/SwerveDriveConfig.java:190` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/SwerveDriveConfig.java:195` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/SwerveDriveConfig.java:226` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/SwerveDriveConfig.java:234` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/pose/DifferentialPose.java:38` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/pose/DifferentialPose.java:39` |
| 2 | `ME_MUTABLE_ENUM_FIELD` | `org/frc5010/common/drive/pose/DrivePoseEstimator.java:None` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/pose/DrivePoseEstimator.java:171` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/pose/DrivePoseEstimator.java:298` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/pose/GenericPose.java:45` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/pose/GenericPose.java:60` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/pose/PoseProvider.java:101` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/pose/PoseProvider.java:101` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/pose/PoseProvider.java:113` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/pose/PoseProvider.java:113` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/pose/PoseProvider.java:113` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/pose/PoseProvider.java:113` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/pose/SwerveFunctionsPose.java:19` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/swerve/GenericSwerveDrivetrain.java:78` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/swerve/GenericSwerveDrivetrain.java:79` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/swerve/GenericSwerveDrivetrain.java:174` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/swerve/GenericSwerveModuleInfo.java:26` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/swerve/GenericSwerveModuleInfo.java:27` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/swerve/GenericSwerveModuleInfo.java:28` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/swerve/GenericSwerveModuleInfo.java:29` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/swerve/GenericSwerveModuleInfo.java:30` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/swerve/GenericSwerveModuleInfo.java:31` |
| 2 | `MS_EXPOSE_REP` | `org/frc5010/common/drive/swerve/YAGSLSwerveDrivetrain.java:567` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/swerve/YAGSLSwerveDrivetrain.java:617` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/swerve/akit/AkitSwerveDrive.java:386` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/swerve/akit/AkitSwerveDrive.java:525` |
| 2 | `MS_PKGPROTECT` | `org/frc5010/common/drive/swerve/akit/DriveConstants.java:23` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/swerve/akit/GyroIOSim.java:13` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/drive/swerve/akit/Module.java:45` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/drive/swerve/akit/Module.java:157` |
| 2 | `MS_PKGPROTECT` | `org/frc5010/common/drive/swerve/akit/OdometryThread.java:None` |
| 2 | `MS_EXPOSE_REP` | `org/frc5010/common/drive/swerve/akit/OdometryThread.java:29` |
| 2 | `MS_EXPOSE_REP` | `org/frc5010/common/drive/swerve/akit/PhoenixOdometryThread.java:47` |
| 2 | `MS_EXPOSE_REP` | `org/frc5010/common/drive/swerve/akit/SparkOdometryThread.java:39` |
| 2 | `MS_EXPOSE_REP` | `org/frc5010/common/drive/swerve/akit/TalonFXOdometryThread.java:43` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/drive/swerve/akit/util/SparkUtil.java:19` |
| 2 | `ME_MUTABLE_ENUM_FIELD` | `org/frc5010/common/motors/MotorConstants.java:None` |
| 2 | `ME_MUTABLE_ENUM_FIELD` | `org/frc5010/common/motors/MotorConstants.java:None` |
| 2 | `ME_MUTABLE_ENUM_FIELD` | `org/frc5010/common/motors/MotorConstants.java:None` |
| 2 | `ME_MUTABLE_ENUM_FIELD` | `org/frc5010/common/motors/MotorConstants.java:None` |
| 2 | `MS_PKGPROTECT` | `org/frc5010/common/motors/MotorFactory.java:17` |
| 2 | `MS_PKGPROTECT` | `org/frc5010/common/motors/MotorFactory.java:18` |
| 2 | `MS_FINAL_PKGPROTECT` | `org/frc5010/common/motors/MotorFactory.java:19` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/motors/control/RevSparkController.java:36` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/motors/control/TalonFXController.java:40` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/motors/control/ThriftyNovaController.java:27` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/control/ThriftyNovaController.java:123` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/motors/function/AngularControlMotor.java:204` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/function/GenericControlledMotor.java:121` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/motors/function/GenericControlledMotor.java:125` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/motors/function/GenericControlledMotor.java:353` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/function/GenericControlledMotor.java:358` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/motors/function/GenericFunctionalMotor.java:51` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/motors/function/GenericFunctionalMotor.java:62` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/function/GenericFunctionalMotor.java:104` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/motors/function/GenericFunctionalMotor.java:282` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/function/GenericFunctionalMotor.java:288` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/motors/function/GenericFunctionalMotor.java:424` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/function/VerticalPositionControlMotor.java:161` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericRevBrushlessMotor.java:163` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericRevBrushlessMotor.java:263` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericRevBrushlessMotor.java:279` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericRevBrushlessMotor.java:289` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericRevBrushlessMotor.java:299` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericTalonFXMotor.java:237` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericTalonFXMotor.java:262` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericTalonFXMotor.java:272` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:238` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:253` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:263` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:273` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericThriftyNovaMotor.java:159` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericThriftyNovaMotor.java:168` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericThriftyNovaMotor.java:173` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/motors/hardware/GenericThriftyNovaMotor.java:178` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/Controller.java:73` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/sensors/Controller.java:624` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/sensors/Controller.java:634` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/sensors/Controller.java:644` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/sensors/Controller.java:654` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/sensors/Controller.java:664` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/sensors/Controller.java:674` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/sensors/Controller.java:684` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/sensors/Controller.java:694` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/sensors/Controller.java:704` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/sensors/Controller.java:714` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/Controller.java:725` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/Controller.java:735` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/Controller.java:745` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/Controller.java:755` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/SparkLimit.java:14` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/ThriftyLimit.java:15` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/absolute_encoder/RevAbsoluteEncoder.java:40` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/camera/PhotonVisionFiducialTargetCamera.java:38` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/camera/PhotonVisionFiducialTargetCamera.java:39` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/camera/PhotonVisionFiducialTargetCamera.java:74` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/camera/PhotonVisionFiducialTargetCamera.java:85` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/camera/PhotonVisionPoseCamera.java:76` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/camera/PhotonVisionPoseCamera.java:89` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/camera/PhotonVisionPoseCamera.java:91` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/camera/PhotonVisionPoseCamera.java:316` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/camera/PhotonVisionPoseCamera.java:325` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/camera/PhotonVisionPoseCamera.java:336` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/camera/QuestNavInterface.java:192` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/camera/SimulatedCamera.java:111` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/camera/SimulatedFiducialTargetCamera.java:45` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/camera/SimulatedFiducialTargetCamera.java:86` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/camera/SimulatedFiducialTargetCamera.java:97` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/encoder/RevEncoder.java:24` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/encoder/RevEncoder.java:29` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/encoder/SimulatedEncoder.java:18` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/encoder/ThriftyNovaEncoder.java:25` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/sensors/encoder/WpiEncoder.java:15` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/gyro/NavXGyro.java:76` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/sensors/gyro/PigeonGyro.java:18` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/subsystems/AprilTagPoseSystem.java:52` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/subsystems/AprilTagPoseSystem.java:68` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/subsystems/AprilTagPoseSystem.java:84` |
| 2 | `EI_EXPOSE_REP` | `org/frc5010/common/subsystems/AprilTagPoseSystem.java:132` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/subsystems/CameraSystem.java:72` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/subsystems/DriverDisplaySubsystem.java:24` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/subsystems/LEDStripSegment.java:18` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/subsystems/LEDStripSegment.java:19` |
| 2 | `MS_EXPOSE_REP` | `org/frc5010/common/subsystems/PhysicsSim.java:13` |
| 2 | `EI_EXPOSE_REP2` | `org/frc5010/common/telemetry/DisplayableValue.java:30` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/utils/RobotIdentity.java:21` |
| 2 | `ME_MUTABLE_ENUM_FIELD` | `org/frc5010/common/vision/AprilTags.java:None` |
| 2 | `ME_MUTABLE_ENUM_FIELD` | `org/frc5010/common/vision/AprilTags.java:None` |
| 2 | `PA_PUBLIC_PRIMITIVE_ATTRIBUTE` | `org/frc5010/common/vision/AprilTags.java:197` |
| 2 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/vision/VisionConstants.java:13` |
| 2 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/vision/VisionConstants.java:14` |
| 2 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/vision/VisionConstants.java:18` |
| 2 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/vision/VisionConstants.java:19` |
| 2 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/vision/VisionConstants.java:30` |
| 2 | `MS_SHOULD_BE_FINAL` | `org/frc5010/common/vision/VisionConstants.java:31` |


## 5. Style & naming (31 findings, priority LOW)

Methods that could be static, missing switch defaults, naming-convention deviations, clone() without super.clone(). Fix opportunistically when touching the file.

**Patterns:** `SS_SHOULD_BE_STATIC` (16), `CN_IDIOM_NO_SUPER_CALL` (5), `SF_SWITCH_NO_DEFAULT` (5), `NM_METHOD_NAMING_CONVENTION` (3), `NM_VERY_CONFUSING` (1), `SIC_INNER_SHOULD_BE_STATIC` (1)

| P | Pattern | Location |
|---|---------|----------|
| 1 | `NM_VERY_CONFUSING` | `org/frc5010/common/motors/function/PercentControlMotor.java:103` |
| 2 | `CN_IDIOM_NO_SUPER_CALL` | `frc/robot/example/subsystems/ExampleIOInputsAutoLogged.java:60` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/arch/GenericCommandSequence.java:24` |
| 2 | `SIC_INNER_SHOULD_BE_STATIC` | `org/frc5010/common/arch/StateMachine.java:455` |
| 2 | `SF_SWITCH_NO_DEFAULT` | `org/frc5010/common/arch/WpiNetworkTableValuesHelper.java:351` |
| 2 | `SF_SWITCH_NO_DEFAULT` | `org/frc5010/common/arch/WpiNetworkTableValuesHelper.java:385` |
| 2 | `SF_SWITCH_NO_DEFAULT` | `org/frc5010/common/arch/WpiNetworkTableValuesHelper.java:409` |
| 2 | `SF_SWITCH_NO_DEFAULT` | `org/frc5010/common/arch/WpiNetworkTableValuesHelper.java:450` |
| 2 | `SF_SWITCH_NO_DEFAULT` | `org/frc5010/common/arch/WpiNetworkTableValuesHelper.java:490` |
| 2 | `NM_METHOD_NAMING_CONVENTION` | `org/frc5010/common/auto/AutoPath.java:46` |
| 2 | `NM_METHOD_NAMING_CONVENTION` | `org/frc5010/common/auto/AutoPath.java:61` |
| 2 | `NM_METHOD_NAMING_CONVENTION` | `org/frc5010/common/auto/AutoSequence.java:116` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/commands/DriveToPoseSupplier.java:66` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/commands/DriveToPoseSupplier.java:67` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/commands/DriveToPoseSupplier.java:74` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/commands/DriveToPoseSupplier.java:85` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/commands/calibration/WheelRadiusCharacterization.java:33` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/commands/calibration/WheelRadiusCharacterization.java:34` |
| 2 | `CN_IDIOM_NO_SUPER_CALL` | `org/frc5010/common/drive/pose/DrivePoseEstimatorInputsAutoLogged.java:72` |
| 2 | `CN_IDIOM_NO_SUPER_CALL` | `org/frc5010/common/drive/pose/VisionIOInputsAutoLogged.java:44` |
| 2 | `CN_IDIOM_NO_SUPER_CALL` | `org/frc5010/common/drive/swerve/akit/GyroIOInputsAutoLogged.java:28` |
| 2 | `CN_IDIOM_NO_SUPER_CALL` | `org/frc5010/common/drive/swerve/akit/ModuleIOInputsAutoLogged.java:50` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/motors/function/AngularControlMotor.java:49` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/motors/function/VerticalPositionControlMotor.java:52` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/motors/function/VerticalPositionControlMotor.java:53` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/motors/function/VerticalPositionControlMotor.java:54` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/motors/hardware/GenericRevBrushlessMotor.java:66` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/motors/hardware/GenericTalonFXMotor.java:44` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/motors/hardware/GenericTalonFXSMotor.java:45` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/sensors/absolute_encoder/GenericAbsoluteEncoder.java:15` |
| 2 | `SS_SHOULD_BE_STATIC` | `org/frc5010/common/subsystems/SegmentedLedSystem.java:31` |

