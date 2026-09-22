# `libraries/Repulsor/`

Read this before touching anything under `src/main/java/frc/robot/libraries/Repulsor/`.

In-repo custom library (GPLv3, by Paul Hodges): a large, mostly self-contained autonomous field-awareness/pathing library, separate from the rest of the robot code. Key subpackages:

- `FieldPlanner/` + `Fields/` — potential-field ("repulsor") pathing, with per-game field definitions (`Rebuilt2026`, `Reefscape2025`).
- `Tracking/` — fuses vision + predicted state into a live field map of game elements (`FieldTrackerCore`).
- `Behaviours/` — pluggable autonomous behaviors run by `BehaviourManager`, driven by a `Reasoning/` rule-based `Reasoner`/`Blackboard`.
- `Shooting/` — a separate drag-based ballistic shot planner (`DragShotPlanner*`), distinct from `TurretSubsystem`'s on-RIO LUT solver (see `docs/architecture.md`).
- `Setpoints/`, `Predictive/`, `Simulation/`, `Target/` (sticky-target smoothing), `Metrics/`, `Profiler/` — supporting infrastructure. `Profiler/` output lands in `repulsor-profiler/*.ndjson` at the repo root.
- `DriverStation/` and `Flags/` — a NetworkTables-backed debug driver-station shim used to override/inspect behavior state at runtime.

`Repulsor` (top-level class) is the integration point: `ShipOfTheseus` constructs one `Repulsor` instance and `Theseus.robotPeriodic()` calls `repulsor.update()` every loop.
