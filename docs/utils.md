# `utils/`

- `GameState` — live DriverStation wrapper for hub timing. Call `update()` once per loop (done in `ShipOfTheseus.periodicUpdate()`); getters read that snapshot. See "Hub timer" below.
- `HubSchedule` — pure (no WPILib) model of the teleop hub schedule, unit tested in `src/test/java/frc/robot/utils/`. See root `README.md` for the full game rules.
- `SimPhysics` — sim-only field-boundary/bump collision physics applied in `Theseus.simulationPeriodic`.
- `Elastic` — helper for switching Elastic dashboard tabs by robot mode.
- `targeting/` — the turret shot math used by `TurretSubsystem`. Pure (no motors, NetworkTables or clock), unit tested in `src/test/java/frc/robot/utils/targeting/`. See `Targeting-Tuning-Guide.md` before changing it.
  - `ShotPlanner` — pose + field velocity + target → turret angle and shot. It projects the pose (position and heading) forward by the latency and solves the moving-shot flight time (Newton's method on `tau = TOF(|d - v*tau|)`).
  - `ShotSolver` is the per-distance shot interface (hood, RPM, flight time, in-range), implemented by:
    - `ShooterTable`, the `turret_data.json` LUT, in range within `LUT_RANGE_MARGIN_IN` of its data.
    - `BallisticShotSolver`, a drag-free projectile model with a linear RPM→exit-speed fit.
    - `FallbackShotSolver`, which tries the primary solver and falls back to the second.
  - `TurretAngles` — the turret angle convention (CW-positive, 0 = back of robot) and wrapping into the turret's travel.
  - `TrackingLagEstimator` — measures how far the turret trails its moving setpoint (tracking error ÷ setpoint rate), part of the measured shoot-on-the-fly latency.

## Hub timer

Teleop is 140 s counting down: Transition 140→130 (both hubs active), Shifts 1–4 at 25 s each (130→105→80→55→30, hubs alternate), End Game 30→0 (both active). A time exactly on a boundary belongs to the earlier phase.

- The FMS game-specific message's first character (`R`/`B`, case-insensitive) is taken as the alliance whose hub is **inactive first** (Shift 1). **Verify this against the game manual before an event** — it is isolated in `GameState.parseInactiveFirst()`; getting it backwards inverts every shift.
- **Fails open:** if the alliance or game data is missing/unparseable, or there is no usable teleop clock (auto, disabled, `-1`/NaN match time), `isHubActive()` returns true and the countdown is 0. `isDataValid()` / the `GameState/Data Valid` topic tell you when this is happening.
- The `PHASE_SHIFT_INCOMING` NeoPixel trigger fires once per inactive shift, `HUB_WARNING_LEAD_S` (10 s, in `ShipOfTheseus`) before the hub becomes active, and re-arms when the robot is disabled or the data becomes invalid.
- The timer does **not** gate shots: `isShotAllowed()` does not consult it.
- Tests: `./gradlew test` (`HubScheduleTest`, plus `GameStateTest` driven by `DriverStationSim`).
