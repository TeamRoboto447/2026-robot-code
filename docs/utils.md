# `utils/`

- `GameState` — tracks hub active/inactive shift timing per the 2026 game clock (see root `README.md` for the full game rules: Fuel scoring into an active Hub, Tower climbing, alternating Hub shifts in teleop).
- `SimPhysics` — sim-only field-boundary/bump collision physics applied in `Theseus.simulationPeriodic`.
- `Elastic` — helper for switching Elastic dashboard tabs by robot mode.
- `ModelShotCalculator`, `ShooterTable` — physics-model and LUT+Newton-solver pieces used by `TurretSubsystem` (see `docs/subsystems.md` and `Targeting-Tuning-Guide.md`).
