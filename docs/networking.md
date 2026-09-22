# `networking/` — the single place NetworkTables keys live

`NetworkedConfig` (runtime-tunable inputs, organized as nested classes `Turret`/`Indexer`/`Intake`/`Climber`/`Debug`/`SystemsCheck`) and `NetworkedTelemetry` (outputs/logging). Prefer adding new NT keys here over scattering `SmartDashboard.put*`/raw NT calls through subsystem code.
