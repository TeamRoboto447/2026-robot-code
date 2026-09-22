# Subsystems (`src/main/java/frc/robot/subsystems/`)

- `CommandSwerveDrivetrain` — CTRE Phoenix 6 swerve (generated config in `generated/TunerConstants.java`, produced by Tuner X).
- `TurretSubsystem` — the most complex subsystem (~1200 lines). Runs an on-RIO shot solver every loop with three modes (`LUT`, `MODEL`, `AUTO_FALLBACK`): a lookup-table + Newton-method dynamic solver seeded from `src/main/deploy/turret_data.json` (empirical static data), and a physics-model fallback (`utils/ModelShotCalculator.java`, `utils/ShooterTable.java` — see `docs/utils.md`). Mode and tuning knobs are runtime-adjustable over NetworkTables (`Debug/*`, `TurretAssembly/*`). See `Targeting-Tuning-Guide.md` for the full tuning procedure and known-failure-mode diagnostics before touching any turret/shooter constant.
- `IntakeSubsystem`, `IndexerSubsystem`, `ClimberSubsystem` — game-piece handling and endgame climb.
- `vision/` — `PoseEstimatorSubsystem` (AprilTag-based pose fusion), `PhotonRunnable`/`ObjectAvoidanceSubsystem` (PhotonVision), `QuestNavSubsystem` (Meta Quest headset tracking, currently disabled/commented out in `ShipOfTheseus`).
- `SystemsCheck` — builds the automated test-mode diagnostic command (`Theseus.testInit`).
