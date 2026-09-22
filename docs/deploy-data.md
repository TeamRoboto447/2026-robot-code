# Auto/paths and tunable data (not hand-edited as code)

- `src/main/deploy/pathplanner/{autos,paths}/` — PathPlanner GUI-authored autonomous routines; edit via the PathPlanner app, not by hand. `NamedCommands` referenced by these paths are registered in `ShipOfTheseus.initializeNamedCommands()`.
- `src/main/deploy/turret_data.json` — empirical shot lookup table consumed by `TurretSubsystem`. Regenerate/edit via `pythonUtils/turret_lookup_generator.py` (see `pythonUtils/README.md`), then `cp` into `src/main/deploy/` and redeploy. `lookup_table_handler.py`/`lookup_table.json` are legacy visualization-only tools — the RIO ignores them.
