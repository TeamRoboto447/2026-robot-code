# `utils/`

- `GameState` — live DriverStation wrapper for hub timing. Call `update()` once per loop (done in `ShipOfTheseus.periodicUpdate()`); getters read that snapshot. See "Hub timer" below.
- `HubSchedule` — pure (no WPILib) model of the teleop hub schedule, unit tested in `src/test/java/frc/robot/utils/`. See root `README.md` for the full game rules.
- `SimPhysics` — sim-only field-boundary/bump collision physics applied in `Theseus.simulationPeriodic`.
- `Elastic` — helper for switching Elastic dashboard tabs by robot mode.
- `ModelShotCalculator`, `ShooterTable` — physics-model and LUT+Newton-solver pieces used by `TurretSubsystem` (see `docs/subsystems.md` and `Targeting-Tuning-Guide.md`).

## Hub timer

Teleop is 140 s counting down: Transition 140→130 (both hubs active), Shifts 1–4 at 25 s each (130→105→80→55→30, hubs alternate), End Game 30→0 (both active). A time exactly on a boundary belongs to the earlier phase.

- The FMS game-specific message's first character (`R`/`B`, case-insensitive) is taken as the alliance whose hub is **inactive first** (Shift 1). **Verify this against the game manual before an event** — it is isolated in `GameState.parseInactiveFirst()`; getting it backwards inverts every shift.
- **Fails open:** if the alliance or game data is missing/unparseable, or there is no usable teleop clock (auto, disabled, `-1`/NaN match time), `isHubActive()` returns true and the countdown is 0. `isDataValid()` / the `GameState/Data Valid` topic tell you when this is happening.
- The `PHASE_SHIFT_INCOMING` NeoPixel trigger fires once per inactive shift, `HUB_WARNING_LEAD_S` (10 s, in `ShipOfTheseus`) before the hub becomes active, and re-arms when the robot is disabled or the data becomes invalid.
- The timer does **not** gate shots: `isShotAllowed()` does not consult it.
- Tests: `./gradlew test` (`HubScheduleTest`, plus `GameStateTest` driven by `DriverStationSim`).
