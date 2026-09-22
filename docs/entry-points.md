# Entry points

Not the WPILib defaults — read this before touching robot init/lifecycle code or `Constants.java`.

- `Theseus.java` — the `LoggedRobot` subclass (AdvantageKit-based `TimedRobot`). Owns the mode-transition callbacks (`autonomousInit`, `teleopInit`, `simulationPeriodic`, etc.), wires up AdvantageKit log/replay sources, and each loop calls `repulsor.update()` then `m_robotContainer.periodicUpdate()`.
- `ShipOfTheseus.java` — the `RobotContainer` equivalent. Constructs all subsystems, PathPlanner `NamedCommands`, and button bindings; owns `getAutonomousCommand()` and `getAutoClimbCommand()`.
- `Design Document.md` at the repo root describes an idealized structure (`Robot.java`, `RobotContainer.java`, a `commands/` package) that does **not** match the current code — treat it as background philosophy on subsystem/command design, not a map of this repo. There is no `commands/` package; command logic lives inline as lambdas in `ShipOfTheseus` bindings or as methods on the subsystems themselves.
- `Constants.java` is one large file of nested static classes, one per subsystem/concern (`TurretSubsystemConstants`, `FieldConstants` with nested `TurretTargetPoints`/`ClimbPositions`/`TurretSafety`/`BumpZones`, `RepulsorConstants`, `VisionConstants`, `NeopixelConstants`, etc.) — the standard FRC pattern, just consolidated.
- `Constants.USE_ADV_KIT` toggles between live logging (WPILOG + NT4) and AdvantageKit log-replay mode in `Theseus`.
- Deploy JVM is capped (`-Xmx80m -Xms16m` in `build.gradle`) to fit the RIO 2's 256MB RAM — don't casually remove this if debugging deploy OOMs.
