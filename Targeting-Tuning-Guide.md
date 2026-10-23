# Targeting System Tuning Guide (Team 447, 2026)

This guide is a complete, field-usable procedure for tuning the robot targeting stack in this repo.

It is written against the current architecture:
- On-RIO shot planning in `TurretSubsystem` via `utils/targeting/ShotPlanner` (latency projection + Newton moving-shot solve), with a physics (ballistic) solver by default and the LUT as a debug comparison mode
- Lookup data from `src/main/deploy/turret_data.json` (hood angles in true degrees from vertical)
- Runtime debug/tuning controls under NetworkTables `Debug/*` and `TurretAssembly/*`

If you follow the order in this document, you will avoid most tuning loops that burn hours.

## 0) What You Are Tuning

The shot quality depends on 5 layers. Tune in this order only:

1. Mechanism health and homing
2. Actuator control loops (turret, hood, flywheel)
3. Static shot map (`turret_data.json`) at zero robot motion
4. Shoot-on-the-fly compensation (latency + range latency + rotation FF)
5. PHYSICS mode exit-speed calibration (the default targeting mode)

If an earlier layer is unstable, do not tune later layers.

## 1) Safety and Setup Checklist (Do This Every Session)

1. Put robot on a known field coordinate frame (or at least consistent origin/heading).
2. Verify hood homing runs successfully.
   - Hood commands are ignored when hood is not homed (`hoodLimitSet == false`).
3. Confirm all turret motors report connected.
   - `Motor Comm Status` telemetry should be true.
4. Confirm pose estimate and alliance are sane.
   - Wrong alliance or bad pose causes wrong target selection even if turret code is correct.
5. Start in PHYSICS mode (the default), no overrides.
   - `Debug/Turret Targeting Mode = PHYSICS`
   - `Debug/Override RPM Enabled = false`
   - `Debug/Override Hood Angle Enabled = false`
   - `Debug/Turret Target Override = AUTO` (or set explicit target for controlled tests)

## 2) Verify Inputs Before Touching Any Gains

Before changing gains/offsets, validate the solver inputs published every loop:

Under `TurretAssembly/targetting` (sic):
- `robot_x`, `robot_y`, `robot_angle`
- `target_x`, `target_y`, `target_height`
- `target_distance`
- `has_valid_shot`

Expected behavior:
1. `target_distance` changes smoothly as robot moves.
2. `has_valid_shot` is true when target exists and solve succeeds.
3. `target_x/target_y` match selected target and alliance.
4. `robot_*` values match field display/odometry.

If these are wrong, do not tune shooter values yet.

## 3) Mechanical + Sensor Baseline

### 3.1 Hood homing and travel

1. Run hood homing command.
2. Confirm hood reaches stall, zeros encoder, and becomes homed.
3. Command min and max hood angles and verify physical endpoints match constants:
   - `MIN_HOOD_ANGLE` (23°, the homed hard stop)
   - `MAX_HOOD_ANGLE` (≈43.3°, 17 motor rotations above the stop)
4. Check two angles with a protractor. Hood angles are true degrees from vertical: 301 motor rotations = 360° (`HOOD_DEGREES_ROTATION_RATIO`). Commands outside the travel are clamped, not ignored.
5. If hood does not hit setpoint repeatably, fix mechanism first (binding, backlash, slipping coupler, brownout).

### 3.2 Turret angle baseline

1. Command known turret angles (for example 0, +/-30, +/-90 deg).
2. Verify measured `Turret Angle` tracks command with low overshoot.
3. Verify limits are respected (`MIN_TURRET_ANGLE`, `MAX_TURRET_ANGLE`).

### 3.3 Flywheel baseline

1. Command a few fixed RPM values.
2. Verify settle time and steady-state error.
3. Confirm feed trigger behavior: kicker/indexer should only run when at speed and hood safe.
   - "At speed" compares the measured flywheel speed with `Turret/Commanded RPM` (the speed actually sent, including the RPM override), within `FLYWHEEL_READY_TOLERANCE_RPS` (currently 750 RPM).

## 4) Tune Control Loops (Closed-Loop Mechanics First)

Tune these before static shot map collection. Otherwise you are recording compensation for bad control.

### 4.1 Flywheel PID/FF

Parameters:
- `SHOOTER_KP`, `SHOOTER_KI`, `SHOOTER_KD`, `SHOOTER_KV`
- Runtime mirror in `TurretAssembly/flywheel/*`

Procedure:
1. Set `kI = 0`, `kD = 0` initially.
2. Increase `kP` until response is fast but not oscillatory.
3. Add `kV` for better tracking and reduced steady-state error at high RPM.
4. Add small `kD` only if needed for overshoot damping.
5. Keep `kI` near zero unless a persistent bias remains.

Pass criteria:
- Settles quickly to commanded RPM.
- Error band comfortably within `FLYWHEEL_READY_TOLERANCE_RPS`.

### 4.2 Turret angle loop

Parameters:
- `TURRET_KP`, `TURRET_KI`, `TURRET_KD`, `TURRET_KS`

Procedure:
1. Start with `kI = 0`.
2. Raise `kP` until near-critical response.
3. Add `kD` for damping.
4. Use `kS` only to overcome static friction/deadband, not to force aggressive motion.

Pass criteria:
- Tracks moving target without chatter.
- Holds angle without drift.

### 4.3 Hood loop

Parameters:
- `HOOD_KP`, `HOOD_KI`, `HOOD_KD`

Procedure:
1. Tune for no oscillation and consistent final angle.
2. Validate repeatability after repeated up/down moves.

Pass criteria:
- Angle repeatability better than hood tolerance in practical shooting.

## 5) Build or Refresh Static Lookup Data (`turret_data.json`)

This is the most important calibration for stationary accuracy.

Current data file:
- `src/main/deploy/turret_data.json`

### 5.1 Data collection rules

1. Robot stationary (vpar = 0).
2. Use fixed target and known distances.
3. Tune hood+RPM at each distance until repeatable makes.
4. Record realistic `flight_time` for each point (critical for moving-shot lead).
5. Collect 4 to 8 points spanning full intended shot envelope.

### 5.2 Practical spacing

1. Keep denser points where curve bends most (mid/far range transitions).
2. Avoid giant distance gaps; interpolation across big gaps causes weak midpoints.
3. Hub shots are only taken within `LUT_RANGE_MARGIN_IN` (24 in) of the first and last points (`Turret/Shot In Range`). To shoot from farther or closer, add points there. Relay shots to the corners are not range-limited.

### 5.3 Deploying updated table

1. If using python helper from `pythonUtils/`, generate/update `turret_data.json`.
2. Copy to deploy path:
   - `cp turret_data.json ../src/main/deploy/turret_data.json`
3. Redeploy robot code (`./gradlew deploy`).
4. Reboot robot program to ensure fresh table load.

## 6) Static Shot Validation (LUT Mode)

Goal: prove stationary shots are good before dynamic tuning.

1. Set `Debug/Turret Targeting Mode = LUT`.
2. Keep RPM/hood overrides disabled.
3. Test each calibration distance.
4. Record hit rate, left/right miss, short/long miss.

If misses are mostly short/long while stationary:
- LUT data points (hood/rpm) are wrong or too sparse.

If misses are mostly left/right while stationary:
- turret aiming frame/zero/heading convention issue or mechanical slop.

Do not proceed until this phase is stable.

## 7) Dynamic Tuning (Shoot-On-The-Fly)

Now tune motion compensation.

### 7.1 Lateral lead latency (measured + `Latency Trim S`)

Purpose:
- Projects robot/turret position forward to where the robot will be when the turret actually reaches its aim.

How it is computed (every loop, shown under `Turret/Latency *`):
- `Latency Pose Age S`: how old the drivetrain pose estimate is (CTRE state timestamp). Normally a few ms.
- `Latency Turret Lag S`: how far the turret trails its setpoint while auto-aiming, measured live (tracking error ÷ setpoint rate during steady motion, filtered over ~1 s, clamped to 0–0.25 s). It starts at 0.045 s (the old hand-tuned total) and only updates while the turret is tracking a moving aim, e.g. while driving past the hub.
- Trim: `SOTF_LATENCY_TRIM_S` (constant, 0) + `Debug/Latency Trim S` (live). This covers what the robot can't measure: the Quest's capture-to-robot delay and the ball's response.
- `Latency Total S` is the sum.

Test method:
1. Drive perpendicular to target at constant speed, long enough for `Latency Turret Lag S` to settle.
2. Fire repeated shots.
3. Observe whether impacts lag or lead travel direction.

Adjustment (`Debug/Latency Trim S`, then copy the value into `SOTF_LATENCY_TRIM_S`):
- Shots land behind path: increase value.
- Shots land ahead of path: decrease value.

Notes:
- Start with small increments (around 0.01 s).
- Re-test at multiple speeds.
- If `Latency Turret Lag S` sits at 0 or 0.25 s, or jumps around, the measurement isn't working; note it, and use the trim to compensate.

### 7.2 Radial compensation (`SOTF_RANGE_LATENCY_COMPENSATION_S`)

Purpose:
- Correct toward/away motion error without disturbing lateral lead.

Test method:
1. Drive directly toward target and fire.
2. Drive directly away and fire.

Adjustment:
- Toward = long, away = short: decrease value.
- Toward = short, away = long: increase value.

### 7.3 Rotation feedforward (`TURRET_ROTATION_FF`)

Purpose:
- Reduce turret lag while robot yaws. The planner already aims from the heading projected by the latency, so this is a trim on top of that (default 0).

Test method:
1. Rotate chassis while maintaining target lock.
2. Watch turret lag/overshoot.

Adjustment:
- Still lagging: increase FF.
- Overshooting/leading too much: decrease FF.

## 8) PHYSICS Mode (Ballistic Solver, the Default)

Targeting is physics-based by default and does not use `turret_data.json`. `LUT Only (debug)` on the mode chooser switches back to the table for comparison.

What it does:
- Hood: flies the lowest-speed arc to the target (launch pitch = 45° + half the elevation angle, clamped to the hood travel). Near this angle a small hood error barely moves the landing point.
- RPM and flight time: drag-free projectile math from the launch height (`PHYSICS_LAUNCH_HEIGHT_M`) to the target, with exit speed linear in RPM (`PHYSICS_EXIT_SPEED_INTERCEPT_MPS`, `PHYSICS_EXIT_SPEED_PER_RPM_MPS`).
- Target heights come from the field: the hub aim point is the center of its 41.7 in hexagonal opening, whose front edge is 72 in (~1.83 m) off the carpet (2026 Game Manual). Corner relay targets are at floor level.
- No shot when the required RPM is outside `PHYSICS_MIN_RPM`–`PHYSICS_MAX_RPM` or the arc can't reach the target.
- Drag and Magnus lift (backspin, roughly 7–8% of the ball's weight) aren't modeled. Fitting exit speed to real shots absorbs them at shooting distances; they only matter for 8–10 m relay lobs (~0.4–0.7 m short).

Exit-speed calibration:
- The current constants are a stopgap: first fit to the LUT, then shifted ~80 RPM lower because shots carried 6–12 in past the hub center.
- To calibrate properly, use the debug overrides to hold a fixed hood angle and fire at two RPMs (e.g. 3000 and 4500). Measure where each ball first hits the carpet, solve the exit speed for each, and fit the two constants.

Relevant entries:
- `Debug/Turret Targeting Mode`: `PHYSICS` (default) or `LUT`
- `Debug/Physics Exit Speed Scale` (1.0 = as fit). Physics shots long → raise it; short → lower it.

Telemetry:
- `Turret/Selected Solver` (`PHYSICS`, `LUT`, or `NONE`)
- `Turret/LUT-Physics Hood Delta Deg`, `Turret/LUT-Physics RPM Delta` (comparison only)
- `Turret/Time Of Flight S`
- `Turret/Shot Arc` (the modeled ball path)

Procedure:
1. Take static shots at known distances (check `target_distance` against a tape first).
2. If they are all long or short by about the same amount, trim `Physics Exit Speed Scale`. If the error grows with distance, do the exit-speed calibration above.

## 9) Runtime Override Workflow for Fast Point Collection

Use overrides to find a good point quickly while preserving solver telemetry.

1. Set target override to fixed target (`RED_HUB` or `BLUE_HUB` etc.).
2. Enable `Override RPM Enabled` and/or `Override Hood Angle Enabled`.
3. Sweep values until hit quality is good at that distance.
4. Log the winning tuple into `turret_data.json` as a static point.
5. Disable overrides after testing.

Important:
- Overrides affect actuation but the solver solution is still published for comparison.
- Override hood angles are true degrees from vertical and are clamped to the hood travel.

## 10) Known Failure Modes and What To Do

### 10.1 No shots feed even when aiming looks correct

Likely causes:
1. Flywheel not inside `FLYWHEEL_READY_TOLERANCE_RPS`.
2. Hood not homed.
3. Hood safety zones suppressing feed.
4. `has_valid_shot` false due to missing target/solver output.
5. Hub shot outside the LUT range plus margin (`Turret/Shot In Range` false).

Checks:
- Flywheel RPM vs `Turret/Commanded RPM`.
- Hood homed state.
- `has_valid_shot` and selected target.
- Safety-zone position.

### 10.2 Static shots good, moving shots bad

Likely causes:
1. Incorrect `flight_time` values in LUT points.
2. Latency constants not tuned.
3. Pose velocity estimate noisy.

Actions:
1. Re-measure flight times.
2. Re-tune lateral and radial latency constants.
3. Validate odometry/velocity quality.

### 10.3 Left/right misses only

Likely causes:
1. Turret zero sign/convention mismatch.
2. Robot heading bias or gyro drift.
3. Rotation FF too high/low in spin.

Actions:
1. Verify commanded vs actual turret angle conventions.
2. Verify heading reference and pose alignment.
3. Re-tune `TURRET_ROTATION_FF`.

### 10.4 Short/long misses only

Likely causes:
1. Bad LUT hood/rpm points.
2. Wrong target height assumption.
3. Radial latency compensation sign/magnitude wrong.

Actions:
1. Refresh points at affected ranges.
2. Confirm target height constants and field dimensions.
3. Re-tune range latency.

### 10.4a Misses change with robot heading (turret angle)

If the pivot-to-hub distance on the dashboard matches a tape measure at every heading but shots
still land short or to one side at some headings, the kicker/turret spin differs by turret angle.

Fix with the per-turret-angle trims under `TurretAssembly/angle trims`. There are 8 points, one every
45 deg of turret angle (CW-positive, 0 = turret pointing at the back of the robot):
- `Distance Trim In <angle>`: inches added to the solved distance. A shot landing 12 in short gets +12.
- `Yaw Trim Deg <angle>`: degrees added to the turret aim. A shot landing left (from the shooter's
  view) gets a positive value.
- `Enabled`: turns the trims off for A/B comparison. `Applied ... Trim` shows what this loop used.

Procedure: park at one distance, rotate the robot so the turret sits on each table angle in turn,
and adjust that point until the shot lands centered. Values reset to the constants on reboot, so
copy tuned values into `TURRET_ANGLE_DISTANCE_TRIM_IN` / `TURRET_ANGLE_YAW_TRIM_DEG`. If the
needed trims differ a lot between near and far shots, the spin effect scales with distance and a
fixed-inches trim will need re-tuning per range.

### 10.5 Works in one alliance, wrong in other

Likely causes:
1. Target selection/alliance logic mismatch.
2. Pose mirror/field frame mismatch.

Actions:
1. Validate alliance detection.
2. Compare `target_x/y` against expected field points.

### 10.6 Intermittent jitter or unstable solve

Likely causes:
1. Sparse LUT points with abrupt interpolation slope changes.
2. Noisy velocity input.
3. Mechanical backlash in turret/hood.

Actions:
1. Add intermediate LUT points.
2. Reduce noise at source (pose filtering, wiring, CAN health).
3. Tighten mechanism.

## 11) Suggested Session Plan (90-Minute Practice Slot)

1. 10 min: safety checks, homing, telemetry verification.
2. 20 min: control-loop validation (no table edits).
3. 25 min: static point collection/refresh for LUT.
4. 20 min: moving-shot latency tuning (lateral then radial).
5. 10 min: model/fallback comparison checks.
6. 5 min: capture final constants and table snapshot in git.

## 12) Data Hygiene and Versioning

1. Commit every meaningful tuning change with notes:
   - Distances tested
   - Speeds tested
   - Hit rates and miss pattern
2. Keep old `turret_data.json` snapshots (git handles this).
3. Avoid changing more than one compensation variable at a time.

## 13) Quick Reference: Most-Touched Knobs

Primary:
- `src/main/deploy/turret_data.json` points (`distance`, `angle`, `rpm`, `flight_time`)
- `TURRET_KP/KD/KS`, `HOOD_KP/KD`, `SHOOTER_KP/KV`
- `SOTF_LATENCY_TRIM_S` (on top of the measured pose age and turret lag)
- `SOTF_RANGE_LATENCY_COMPENSATION_S`
- `TURRET_ROTATION_FF`

Debug-only runtime knobs:
- Target override chooser
- Mode chooser (`LUT`, `PHYSICS`)
- RPM/hood override toggles and values
- `Physics Exit Speed Scale`
- `Latency Trim S`

---

If you hit a state where nothing makes sense, reset to this baseline:
1. PHYSICS mode
2. Overrides off
3. Fresh hood homing
4. Confirm pose + target telemetry
5. Re-test one known-good static distance

That sequence isolates 90% of integration failures quickly.
