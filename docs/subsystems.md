# Subsystems (`src/main/java/frc/robot/subsystems/`)

- `CommandSwerveDrivetrain` — CTRE Phoenix 6 swerve (generated config in `generated/TunerConstants.java`, produced by Tuner X).
- `TurretSubsystem` — the most complex subsystem (~1200 lines). Plans a shot every loop with `utils/targeting/ShotPlanner` (see `docs/utils.md`) in one of two modes: `LUT` (the empirical table `src/main/deploy/turret_data.json`) or `PHYSICS` (a ballistic solver that falls back to the LUT when out of range). It also recovers motor-controller resets (brownouts) every loop. Mode and tuning knobs are runtime-adjustable over NetworkTables (`Debug/*`, `TurretAssembly/*`). See `Targeting-Tuning-Guide.md` for the full tuning procedure and known-failure-mode diagnostics before touching any turret/shooter constant.
- `IntakeSubsystem`, `IndexerSubsystem` — game-piece handling. (`ClimberSubsystem` is commented out: the climber was physically removed.)
- `vision/` — `PoseEstimatorSubsystem` (AprilTag-based pose fusion), `PhotonRunnable`/`ObjectAvoidanceSubsystem` (PhotonVision), `QuestNavSubsystem` (Meta Quest headset tracking, currently disabled/commented out in `ShipOfTheseus`).
- `SystemsCheck` — builds the automated test-mode diagnostic command (`Theseus.testInit`).
