# `networking/` — the single place NetworkTables keys live

`NetworkedConfig` (runtime-tunable inputs, organized as nested classes `Turret`/`Indexer`/`Intake`/`Climber`/`Debug`/`SystemsCheck`) and `NetworkedTelemetry` (outputs/logging). Prefer adding new NT keys here over scattering `SmartDashboard.put*`/raw NT calls through subsystem code.

## `GameState/` telemetry (hub timer)

Published every loop from `ShipOfTheseus.periodicUpdate()` via `NetworkedTelemetry.GameState`: `Match Time`, `Hub Active` (true when unknown — fails open), `Hub Active Countdown` (seconds until the hub next becomes active; 0 if active), `Data Valid` (false when FMS game data/alliance is missing), `Phase` (`NONE`/`AUTO`/`TRANSITION`/`SHIFT_1`..`SHIFT_4`/`ENDGAME`), `Inactive First` (`Red`/`Blue`/`Unknown`). Elastic widgets in `src/main/deploy/elastic-layout.json` bind to the first three; see `docs/utils.md` for the timer's behavior.
