# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

FRC 2026 robot code for Team Roboto 447 (Java 17, WPILib command-based via GradleRIO). Entry points are `Theseus`/`ShipOfTheseus` (robot-themed names), not the WPILib defaults `Robot`/`RobotContainer`.

## Commands

- Build: `./gradlew build`
- Test: `./gradlew test` (single class: `--tests "pkg.ClassName"`; no tests exist yet, but JUnit 5 is wired up)
- Deploy to the roboRIO: `./gradlew deploy`
- Desktop simulation (sim GUI + driverstation): `./gradlew simulateJava`
- Generate Javadoc: `./gradlew javadoc`
- Replay an AdvantageKit log: `./gradlew replayWatch`

No linter/formatter task is configured.

## Docs (read on demand — not preloaded)

- `docs/entry-points.md` — `Theseus`/`ShipOfTheseus`, `Constants.java` structure. Read before touching robot init/lifecycle code or `Constants.java`.
- `docs/subsystems.md` — per-subsystem breakdown (drivetrain, turret, intake/indexer/climber, vision, systems check). Read before touching anything in `subsystems/`.
- `docs/networking.md` — `networking/` (NetworkTables config/telemetry) conventions. Read before adding NT keys or dashboard values.
- `docs/utils.md` — `utils/` package (game state, sim physics, shot calculators). Read before touching `utils/`.
- `docs/deploy-data.md` — PathPlanner autos/paths and the turret shot LUT (`turret_data.json`) — generated/GUI-edited, not hand-written. Read before editing anything in `src/main/deploy/`.
- `docs/vendor-deps.md` — what each vendor library (`vendordeps/`) is for.
- `docs/repulsor-library.md` — the in-repo `libraries/Repulsor` autonomous-pathing/behavior library. Read before touching anything under `libraries/Repulsor/`.
- `Targeting-Tuning-Guide.md` — turret/shooter tuning procedure and failure-mode diagnostics. Read before changing turret/shooter constants or the on-RIO shot solver.
- `Design Document.md` — coding-convention philosophy for subsystems/commands. Its "Project Structure" section is stale (no `commands/` package, no `Robot.java`/`RobotContainer.java`) — see `docs/entry-points.md` for the real layout.
