// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.Constants.FieldConstants.FieldZone;
import frc.robot.Constants.FieldConstants.TurretTarget;
import frc.robot.Constants.FieldConstants.TurretTargetPoints;
import frc.robot.networking.NetworkedConfig;
import frc.robot.networking.NetworkedTelemetry;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShotSolution;
import frc.robot.utils.TargettingUtils.ControlTarget;

/**
 * A subsystem that allows control of the flywheel, the hood, 
 * the turret's angle, and the kicker.
 */
public class TurretSubsystem extends SubsystemBase {
    private final PoseProvider poseProvider;
    
    private ControlTarget currentControlTarget = new ControlTarget();
    private boolean hoodLimitSet = false;
    private boolean rightShooterMotorConnected, leftShooterMotorConnected,
        angleMotorConnected, hoodMotorConnected;
    private final Trigger hoodLowerLimitTrigger;
    private final Trigger feedTrigger;

    private final TalonFX rightShooterMotor;
    private final TalonFX leftShooterMotor;
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkClosedLoopController hoodController;
    private final TalonFX angleMotor;
    private final SparkMax kickerMotor;

    // Cached status signals — initialized in constructor after motors are created.
    // Calling .getVelocity()/.getPosition() repeatedly creates new signal objects
    // and triggers redundant CAN traffic; caching them reduces bus utilization.
    private StatusSignal<AngularVelocity> shooterVelocitySignal;
    private StatusSignal<edu.wpi.first.units.measure.Angle> anglePositionSignal;

    /** The current turret target as an enum. */
    public TurretTarget turretTarget = TurretTarget.NONE;

    /**
     * Firing table loaded from {@code deploy/turret_data.json} at startup.
     * Provides linear interpolation across static data points and a Newton-method
     * shoot-on-the-fly solver. Replaces the external Python coprocessor round-trip.
     */
    private final ShooterTable shooterTable = new ShooterTable();

    /**
     * Most-recent shot solution from the on-RIO solver. {@code null} until the
     * first successful solve. Cached here so {@link #hasValidTarget()} and
     * {@link #getFeedTrigger()} can inspect it without re-running the solver.
     */
    private ShotSolution lastSolution = null;

    private TalonFXConfiguration ShooterFxConfigs = new TalonFXConfiguration();
    private final VelocityTorqueCurrentFOC velocityReq = new VelocityTorqueCurrentFOC(0).withSlot(0);
    /** Sent to the shooter motors when we want them to coast to a stop, not brake. */
    private final NeutralOut coastReq = new NeutralOut();

    private TalonFXConfiguration AngleFxConfigs = new TalonFXConfiguration();
    private final PositionVoltage anglePositionReq = new PositionVoltage(0).withSlot(0);
    
    private boolean runFlywheel = false;
    private boolean shooting = false;
    private final boolean hoodMotorInverted = false;

    /**
     * {@code true} while the driver or autonomous logic is actively attempting a
     * shoot-on-the-fly shot (shoot button held or {@code autoShootTrigger} active).
     * Read by {@link ShipOfTheseus} to cap drive speed via
     * {@link frc.robot.Constants.TurretSubsystemConstants#SOTF_MAX_DRIVE_SPEED_MPS}.
     */
    private boolean shootingActive = false;

    /** Counts 20ms loops; used to rate-limit low-priority dashboard publishing. */
    private int periodicLoopCount = 0;
    /** How many 20ms loops between dashboard-visualization updates (5 = 10 Hz). */
    private static final int TELEMETRY_LOOP_DIVISOR = 5;

    /**
     * When {@code true}, {@link #periodic()} skips the trajectory-tracking override
     * (hood angle, turret angle, flywheel idle) so the systems-check commands have
     * exclusive control of the mechanism. Set via {@link #setSystemsCheckMode(boolean)}.
     */
    private boolean systemsCheckMode = false;
    /* SysId routine for profiling the shooter (flywheel) */
    private final SysIdRoutine m_sysIdRoutineFlywheel;
    /**
     * Get the trigger that indicates when the shooter is at target velocity.
     * This can be used to coordinate feeding game pieces when the shooter is ready.
     * 
     * @return Trigger that activates when shooter is at target speed
     */
    public Trigger getFeedTrigger() {
        return feedTrigger;
    }

    public void motorStatusCheck() {
        rightShooterMotorConnected = rightShooterMotor.isConnected();
        leftShooterMotorConnected = leftShooterMotor.isConnected();
        angleMotorConnected = angleMotor.isConnected();

        hoodMotorConnected = hoodMotor.getFirmwareVersion() != 0; // If response is zero, there is some issue with the controller
    }
    
    /** 
     * Creates a new TurretSubsystem.
     * 
     * @param poseProvider An object that can fetch the current robot pose
     */
    public TurretSubsystem(PoseProvider poseProvider) {
        this.poseProvider = poseProvider;

        this.rightShooterMotor = new TalonFX(TurretSubsystemConstants.RIGHT_SHOOTER_MOTOR_ID);
        var shooterSlot0config = ShooterFxConfigs.Slot0;
        shooterSlot0config.kP = TurretSubsystemConstants.SHOOTER_KP;
        shooterSlot0config.kI = TurretSubsystemConstants.SHOOTER_KI;
        shooterSlot0config.kD = TurretSubsystemConstants.SHOOTER_KD;
        shooterSlot0config.kV = TurretSubsystemConstants.SHOOTER_KV;
        shooterSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;
        // Coast when no control request is active so the flywheel spins down freely
        // instead of being actively braked (which would fight the follower motor).
        ShooterFxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // Never allow reverse output — the PID must not spin the flywheel backwards
        // while trying to correct overspeed. Clamped at the firmware level so it
        // applies regardless of which control request is active.
        ShooterFxConfigs.MotorOutput.PeakReverseDutyCycle = 0.0;
        // Current limits — prevent brownouts from the high-inertia flywheel.
        ShooterFxConfigs.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(TurretSubsystemConstants.SHOOTER_STATOR_CURRENT_LIMIT_A)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(TurretSubsystemConstants.SHOOTER_SUPPLY_CURRENT_LIMIT_A)
            .withSupplyCurrentLimitEnable(true);

        this.rightShooterMotor.getConfigurator().apply(ShooterFxConfigs);

        this.leftShooterMotor = new TalonFX(TurretSubsystemConstants.LEFT_SHOOTER_MOTOR_ID);

        this.leftShooterMotor.setControl(new Follower(TurretSubsystemConstants.RIGHT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));

        /* build SysId routine now that motors are initialized */
        m_sysIdRoutineFlywheel = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,               // default ramp rate
                Volts.of(6),        // dynamic step voltage (6 V)
                null,               // default timeout (10 s)
                state -> SignalLogger.writeString("SysIdFlywheel_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> {
                    /* apply commanded volts as a percent output (scale by battery voltage) */
                    double percent = volts.in(Volts) / RobotController.getBatteryVoltage();
                    rightShooterMotor.set(percent);
                    /* log requested volts and measured velocity for post-processing */
                    SignalLogger.writeDouble("Flywheel_Request_V", volts.in(Volts));
                    SignalLogger.writeDouble("Flywheel_RPS", rightShooterMotor.getVelocity().getValueAsDouble());
                },
                null,
                this
            )
        );

        this.hoodMotor = new SparkMax(TurretSubsystemConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.inverted(hoodMotorInverted);
        hoodConfig.closedLoop
            .p(TurretSubsystemConstants.HOOD_KP)
            .i(TurretSubsystemConstants.HOOD_KI)
            .d(TurretSubsystemConstants.HOOD_KD);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.hoodEncoder = hoodMotor.getEncoder();
        hoodController = this.hoodMotor.getClosedLoopController();

        hoodLowerLimitTrigger = new Trigger(() -> this.hoodMotor.getForwardLimitSwitch().isPressed());
        hoodLowerLimitTrigger.onTrue(Commands.runOnce((() -> {
            this.hoodEncoder.setPosition(0);
        }), this));

        this.angleMotor = new TalonFX(TurretSubsystemConstants.ANGLE_MOTOR_ID);

        var angleSlot0config = AngleFxConfigs.Slot0;
        angleSlot0config.kP = TurretSubsystemConstants.TURRET_KP;
        angleSlot0config.kI = TurretSubsystemConstants.TURRET_KI;
        angleSlot0config.kD = TurretSubsystemConstants.TURRET_KD;
        angleSlot0config.kS = TurretSubsystemConstants.TURRET_KS;
        angleSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        AngleFxConfigs.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(TurretSubsystemConstants.TURRET_STATOR_CURRENT_LIMIT_A)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(TurretSubsystemConstants.TURRET_SUPPLY_CURRENT_LIMIT_A)
            .withSupplyCurrentLimitEnable(true);

        this.angleMotor.getConfigurator().apply(AngleFxConfigs);
        this.angleMotor.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(TurretSubsystemConstants.TURRET_GEAR_RATIO));
        this.angleMotor.setPosition(0);

        this.kickerMotor = new SparkMax(TurretSubsystemConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

        // Cache status signals to avoid creating new signal objects on every periodic() call.
        // Each un-cached call to .getVelocity()/.getPosition() implicitly registers a new
        // 50 Hz signal on CAN, rapidly filling bus bandwidth with redundant frames.
        this.shooterVelocitySignal = this.rightShooterMotor.getVelocity();
        this.anglePositionSignal   = this.angleMotor.getPosition();

        // Reduce update frequencies to 20 Hz for telemetry-only signals.
        // These values are only used for dashboard display and targeting decisions
        // that run at the 20 ms robot loop rate, so 50 Hz is wasteful.
        // this.shooterVelocitySignal.setUpdateFrequency(20);
        this.anglePositionSignal.setUpdateFrequency(20);

        // Silence all other status frames on these motors that we never read.
        // Phoenix 6 motors broadcast many signals by default; this tells the firmware
        // to suppress any frame not explicitly configured above.
        // this.rightShooterMotor.optimizeBusUtilization();
        // this.leftShooterMotor.optimizeBusUtilization();
        // this.angleMotor.optimizeBusUtilization();

        this.feedTrigger = new Trigger(() -> {
            // Use the on-RIO solution's RPM if available; otherwise fall back to the
            // NT-configured value (used for manual/systems-check overrides).
            double targetRPS = (lastSolution != null)
                    ? lastSolution.rpm / 60.0
                    : NetworkedConfig.Turret.getTargetRPM() / 60.0;
            // Only "ready" when the flywheel is commanded to spin AND within tolerance.
            if (!runFlywheel || targetRPS < 16) return false;
            return this.flywheelAtSpeed();
        });

        motorStatusCheck();

    }

    public Command turnToTarget() {
        return this.run(() ->{
            this.turnToAngle(Degrees.of(NetworkedConfig.Turret.getTargetTurretAngle()));
        });
    }
    
    /**
     * Get the current alliance from DriverStation.
     * 
     * @return The current alliance, or Alliance.Red as default if alliance is not set
     */
    private Alliance getCurrentAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }

    /**
     * Updates the subsystem's various elements, including the on-RIO shot solver,
     * motor setpoints, and NetworkTables telemetry.
     *
     * <h3>Aiming pipeline (runs every 20 ms, zero NT round-trip)</h3>
     * <ol>
     *   <li>Compute the turret pivot position in field coordinates (robot pose +
     *       rotated TURRET_TO_ROBOT offset).</li>
     *   <li>Look up the current target 3-D position from {@link #turretTarget}.</li>
     *   <li>Feed the displacement vector and robot chassis velocity (in inches) into
     *       {@link ShooterTable#solve}, which runs a Newton TOF-recursion to produce
     *       a field-relative aim bearing and interpolated hood angle / RPM.</li>
     *   <li>Convert the aim bearing to a robot-frame turret angle, apply a yaw-rate
     *       feed-forward lead, and command the motor.</li>
     * </ol>
     */
    @Override
    public void periodic() {
        // ── 1. Refresh CAN signals ────────────────────────────────────────────
        // Single batched read — avoids creating new signal objects every loop.
        BaseStatusSignal.refreshAll(shooterVelocitySignal, anglePositionSignal);

        // ── 2. Geometry: turret pivot position in field frame ─────────────────
        Pose2d currentPose = poseProvider.getPose();

        // ── 4. Robot velocity in inches/s ─────────────────────────────────────
        // Read chassis speeds early so we can use them for latency compensation
        // as well as the omega-offset correction below.
        var chassis = poseProvider.getChassisSpeeds();
        double omegaRadPerSec = chassis.omegaRadiansPerSecond;

        // ── Latency compensation ───────────────────────────────────────────────
        // Project the turret pivot position forward by SOTF_LATENCY_COMPENSATION_S
        // to account for combined camera, network, loop, and motor-response delays.
        // This ensures the virtual-target solve uses the robot's expected position at
        // ball-departure time rather than the stale sensor-capture position.
        // Tune SOTF_LATENCY_COMPENSATION_S by driving perpendicular to the target:
        //   shots landing behind your path → increase; ahead of your path → decrease.
        double latencyS = TurretSubsystemConstants.SOTF_LATENCY_COMPENSATION_S;
        double projectedRobotX = currentPose.getX() + chassis.vxMetersPerSecond * latencyS;
        double projectedRobotY = currentPose.getY() + chassis.vyMetersPerSecond * latencyS;

        // Offset the projected robot centre by the turret mount point, rotated to field frame.
        Translation2d turretOffset = new Translation2d(
            TurretSubsystemConstants.TURRET_TO_ROBOT.getX(),
            TurretSubsystemConstants.TURRET_TO_ROBOT.getY()
        ).rotateBy(currentPose.getRotation());
        double turretXm = projectedRobotX + turretOffset.getX();
        double turretYm = projectedRobotY + turretOffset.getY();

        // ── 3. Target position ────────────────────────────────────────────────
        Translation3d currentTargetPose = getTargetFromEnum(turretTarget);

        // Displacement from the projected turret pivot to the target, in inches.
        double dxIn = Units.metersToInches(currentTargetPose.getX() - turretXm);
        double dyIn = Units.metersToInches(currentTargetPose.getY() - turretYm);
        double distanceInches = Math.hypot(dxIn, dyIn);

        // Include the velocity contribution of the turret pivot offset due to robot
        // yaw (v_turret = v_robot_centre + ω × r_offset).
        double vxIps = Units.metersToInches(chassis.vxMetersPerSecond)
                       - omegaRadPerSec * Units.metersToInches(turretOffset.getY());
        double vyIps = Units.metersToInches(chassis.vyMetersPerSecond)
                       + omegaRadPerSec * Units.metersToInches(turretOffset.getX());

        // ── 5. On-RIO shot solve ──────────────────────────────────────────────
        ShotSolution sol = null;
        if (shooterTable.isLoaded() && turretTarget != TurretTarget.NONE && distanceInches > 1.0) {
            sol = shooterTable.solve(dxIn, dyIn, vxIps, vyIps);
        }
        lastSolution = sol;

        // ── 6. Publish NT telemetry (coprocessor now reads these for visualisation) ──
        // Publish the actual (non-projected) turret position for field-visualization accuracy.
        double actualTurretXm = currentPose.getX() + turretOffset.getX();
        double actualTurretYm = currentPose.getY() + turretOffset.getY();
        NetworkedConfig.Turret.setRobotX(Units.metersToInches(actualTurretXm));
        NetworkedConfig.Turret.setRobotY(Units.metersToInches(actualTurretYm));
        NetworkedConfig.Turret.setRobotAngle(currentPose.getRotation().getDegrees());
        NetworkedConfig.Turret.setTargetX(Units.metersToInches(currentTargetPose.getX()));
        NetworkedConfig.Turret.setTargetY(Units.metersToInches(currentTargetPose.getY()));
        NetworkedConfig.Turret.setTargetHeight(Units.metersToInches(currentTargetPose.getZ()));
        NetworkedConfig.Turret.setDistanceToTarget(distanceInches);

        boolean validShot = (sol != null);
        NetworkedConfig.Turret.setValidTarget(validShot);
        if (validShot) {
            // Publish the raw LUT solution so both values remain visible on the
            // dashboard regardless of whether the debug overrides are active.
            NetworkedConfig.Turret.setTargetHoodAngle(sol.angleDeg);
            NetworkedConfig.Turret.setTargetRPM(sol.rpm);
        }

        // ── Debug / LUT-tuning overrides for actuation ────────────────────────
        // These replace the solved RPM and/or hood angle sent to the motors while
        // leaving the NT-published LUT solution untouched, so you can compare the
        // override value against what the solver computed on the same dashboard.
        // Enable via Debug.Override RPM/Hood Angle Enabled on SmartDashboard.
        double actuationRPM     = validShot ? sol.rpm      : 0;
        double actuationHoodDeg = validShot ? sol.angleDeg : TurretSubsystemConstants.MIN_HOOD_ANGLE.magnitude();

        if (NetworkedConfig.Debug.isOverrideRPMEnabled()) {
            actuationRPM = NetworkedConfig.Debug.getOverrideRPMValue();
            // Override is active even without a valid LUT solution so you can spin
            // up while parked outside the table range.
            validShot = true;
        }
        if (NetworkedConfig.Debug.isOverrideHoodAngleEnabled()) {
            actuationHoodDeg = NetworkedConfig.Debug.getOverrideHoodAngleValue();
            validShot = true;
        }

        updateTurretTarget();
        periodicLoopCount++;
        updateNetworkTables();

        // ── 7. Actuate ────────────────────────────────────────────────────────
        if (!systemsCheckMode) {
            double targetRPS = (runFlywheel && validShot) ? actuationRPM / 60.0 : 0;
            if (targetRPS > 16) // ≈ 1000 RPM — spin up whenever a valid shot exists
                rightShooterMotor.setControl(velocityReq.withVelocity(targetRPS));
            else
                rightShooterMotor.setControl(coastReq); // coast freely

            if (validShot && shooting) {
                this.setHoodAngle(Degrees.of(actuationHoodDeg));

                // Convert field-relative aim bearing to robot-frame turret angle.
                // Both the bearing and the robot heading are CCW-positive; the turret
                // convention is CW-positive with 0 pointing toward the back of the robot.
                // turret_angle = robot_heading - aim_bearing + 180 (then negate for CW).
                // When only the RPM/hood are overridden, sol is still used for bearing;
                // when there is no LUT solution at all (pure override), hold last angle.
                if (sol != null) {
                    double robotHeadingDeg = currentPose.getRotation().getDegrees();
                    double baseTargetDeg   = robotHeadingDeg - sol.aimBearingDeg + 180.0;
                    // Normalise to [-180, 180]
                    baseTargetDeg = ((baseTargetDeg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;

                    // Yaw-rate feed-forward: lead the turret ahead by one loop worth of
                    // robot rotation so it tracks rather than lagging.
                    double omegaDegPerSec    = Units.radiansToDegrees(omegaRadPerSec);
                    double compensatedTarget = baseTargetDeg
                        - omegaDegPerSec * TurretSubsystemConstants.TURRET_ROTATION_FF * 0.02;

                    this.turnToAngle(Degrees.of(compensatedTarget));
                }
            } else {
                this.setHoodAngle(TurretSubsystemConstants.MIN_HOOD_ANGLE);
                // Do not call stopTurret() — that writes open-loop set(0) which fights
                // the closed-loop position hold from the last turnToAngle() call.
            }
        }
    }

    /**
     * Signals the shooter to spin up. The target RPM comes from the most-recent
     * {@link ShooterTable} solution computed in {@link #periodic()}; if no solution
     * is available the flywheel coasts.
     */
    public void shoot() {
        shooting = true;
        runFlywheel = true;
        double targetRPS = (lastSolution != null) ? lastSolution.rpm / 60.0
                                                  : NetworkedConfig.Turret.getTargetRPM() / 60.0;
        rightShooterMotor.setControl(velocityReq.withVelocity(targetRPS));
    }

    /**
     * Spins the flywheel directly at the given RPM, bypassing NetworkTables.
     * Intended for use in systems checks and other automated sequences that
     * need to target a specific speed without mutating shared NT state.
     *
     * @param rpm The target flywheel speed in RPM.
     */
    public void spinFlywheelAtRPM(double rpm) {
        shooting = true;
        rightShooterMotor.setControl(velocityReq.withVelocity(rpm / 60.0));
    }

    /**
     * Stops the shooter. Does not immediately stop due to inertia.
     */
    public void stopShooter() {
        shooting = false;
        this.runFlywheel = false;
    }

    /**
     * Stops the hood.
     */
    public void stopHood() {
        this.hoodMotor.set(0);
    }

    public void stopAll() {
        this.stopHood();
        this.stopKicker();
        this.stopTurret();
        this.stopShooter();
    }

    /**
     * Returns a {@link Command} that homes the hood by slowly driving it toward
     * the lower hard stop and zeroing the encoder when a current-based stall is
     * detected.
     *
     * <p>The hood is a NEO 550 on a SparkMax with a 301:1 effective gear ratio
     * (16:1 gearbox driving a 16-tooth pinion on a 301-tooth ring gear).  The
     * command drives the motor at {@code HOOD_HOMING_SPEED} (a small open-loop
     * output toward the lower mechanical stop) until the SparkMax output current
     * exceeds {@code HOOD_HOMING_STALL_AMPS} for at least
     * {@code HOOD_HOMING_STALL_DURATION_S} continuous seconds.  It then zeros
     * the built-in relative encoder and sets {@code hoodLimitSet = true},
     * unlocking normal closed-loop angle control.</p>
     *
     * <p>The NEO 550 free-run current is roughly 1 A; stall is roughly 8 A.
     * The default threshold of 4 A sits comfortably in between and will trip
     * well before the motor reaches its thermal stall point at the slow homing
     * speed.</p>
     *
     * @return the homing command
     */
    public Command homeHood() {
        Timer stallTimer = new Timer();

        return this.runOnce(() -> {
                    // Stop any ongoing closed-loop hood command and start the timer.
                    hoodMotor.set(0);
                    stallTimer.restart();
                })
                .andThen(this.run(() -> {
                    // Drive slowly toward the lower hard stop.
                    hoodMotor.set(TurretSubsystemConstants.HOOD_HOMING_SPEED);

                    // Keep the stall timer reset while current is below the threshold
                    // so it only accumulates time for *continuous* over-current periods.
                    if (hoodMotor.getOutputCurrent() < TurretSubsystemConstants.HOOD_HOMING_STALL_AMPS) {
                    stallTimer.restart();
                    }
                }))
                .until(() ->
                    hoodMotor.getOutputCurrent() >= TurretSubsystemConstants.HOOD_HOMING_STALL_AMPS
                    && stallTimer.hasElapsed(TurretSubsystemConstants.HOOD_HOMING_STALL_DURATION_S))
                .finallyDo((interrupted) -> {
                    hoodMotor.set(0);
                    stallTimer.stop();

                    if (!interrupted) {
                        // Hard stop confirmed — zero the relative encoder and allow
                        // setHoodAngle() to accept commands.
                        hoodEncoder.setPosition(0);
                        hoodLimitSet = true;
                    }
                })
                .unless(() -> hoodLimitSet);
    }

    /** Returns true once the hood has been successfully homed. */
    public boolean isHoodHomed() {
        return hoodLimitSet;
    }

    /** Clears the hood-homed flag, forcing a re-home on the next homing command. */
    public void resetHoodHoming() {
        hoodLimitSet = false;
    }

    /** Returns true when the on-RIO solver has computed a valid shot solution this loop. */
    public boolean hasValidTarget() {
        return lastSolution != null && lastSolution.inRange;
    }

    /**
     * Signals that the robot is actively attempting a shoot-on-the-fly shot.
     * Called by {@link frc.robot.ShipOfTheseus} when the shoot button or
     * autonomous shoot trigger becomes active/inactive.  Read by
     * {@link #isShootingActive()} to gate the drive-speed cap.
     *
     * @param active {@code true} when a shot attempt is in progress
     */
    public void setShootingActive(boolean active) {
        shootingActive = active;
    }

    /**
     * Returns {@code true} while a shoot-on-the-fly attempt is in progress.
     * Used by the drive default command in {@link frc.robot.ShipOfTheseus} to
     * cap the robot's translational speed at
     * {@link frc.robot.Constants.TurretSubsystemConstants#SOTF_MAX_DRIVE_SPEED_MPS}.
     */
    public boolean isShootingActive() {
        return shootingActive;
    }

    /**
     * Enables or disables systems-check mode. While enabled, {@link #periodic()}
     * will not apply the trajectory-tracking overrides (hood/turret angle targeting,
     * idle flywheel spin-up), giving the systems-check commands exclusive control
     * of the mechanism.
     *
     * @param enabled {@code true} to enter systems-check mode, {@code false} to resume normal operation
     */
    public void setSystemsCheckMode(boolean enabled) {
        systemsCheckMode = enabled;
    }

    /**
     * Returns true when the turret is currently targeting either alliance hub.
     * Used to gate shooting during inactive hub periods — if the hub is not
     * active and the turret is aimed at it, shooting should be blocked.
     */
    public boolean isTargetingHub() {
        return turretTarget == TurretTarget.RED_HUB
            || turretTarget == TurretTarget.BLUE_HUB;
    }

    /** Returns the current turret rotation angle in degrees (from the cached position signal). */
    public double getTurretAngleDegrees() {
        return anglePositionSignal.getValueAsDouble();
    }

    /**
     * Returns the current hood angle in degrees (absolute, offset from MIN_HOOD_ANGLE),
     * derived from the SparkMax relative encoder.
     */
    public double getHoodAngleDegrees() {
        return TurretSubsystemConstants.MIN_HOOD_ANGLE
            .plus(TurretSubsystemConstants.HOOD_DEGREES_ROTATION_RATIO
                .times(hoodEncoder.getPosition()))
            .magnitude();
    }

    /** Returns the current flywheel speed in RPM. */
    public double getFlywheelRPM() {
        return shooterVelocitySignal.getValueAsDouble() * 60.0;
    }
    /**
     * Stops the kicker.
     */
    public void stopKicker() {
        this.kickerMotor.set(0);
    }

    /**
     * Drives the kicker motor directly at {@code strength} without checking flywheel
     * speed or trajectory validity. Intended for systems-check use only.
     *
     * @param strength Motor output on [-1, 1].
     */
    public void runKickerRaw(double strength) {
        this.kickerMotor.set(strength);
    }
    /**
     * Stops the turret.
     */
    public void stopTurret() {
        this.angleMotor.set(0);
    }

    /**
     * Starts the turret moving towards a new angle.
     * 
     * @param newAngle The angle to move toward.
     */
    public void turnToAngle(Angle newAngle) {
        if (newAngle.lt(TurretSubsystemConstants.MIN_TURRET_ANGLE) || newAngle.gt(TurretSubsystemConstants.MAX_TURRET_ANGLE)) {
            return;
        }
        // Always issue the closed-loop position request — even when within tolerance
        // this keeps the motor actively holding position instead of going open-loop (set(0))
        // which would allow the mechanism to drift under gravity or mechanism compliance.
        angleMotor.setControl(anglePositionReq.withPosition(newAngle));
    }

    public void turnRaw(double power) {
        angleMotor.set(power);
    }

    public boolean flywheelAtSpeed() {
        // Prefer the on-RIO solution's RPM; fall back to the NT override for manual use.
        double targetRPS = (lastSolution != null)
                ? lastSolution.rpm / 60.0
                : NetworkedConfig.Turret.getTargetRPM() / 60.0;
        return Math.abs(shooterVelocitySignal.getValueAsDouble() - targetRPS)
            <= TurretSubsystemConstants.FLYWHEEL_READY_TOLERANCE_RPS;
    }

    /**
     * Runs the kicker motor only when the flywheel is within
     * {@link TurretSubsystemConstants#FLYWHEEL_READY_TOLERANCE_RPS} of its target
     * speed AND the coprocessor has a valid trajectory. Stops the kicker otherwise
     * so a game piece is never fed into an under-speed flywheel.
     *
     * @param strength The strength to run the motor at, on a scale of -1 (full reverse) to 1 (full forward).
     */
    public void kick(double strength) {
        if (lastSolution != null && flywheelAtSpeed())
            runKickerRaw(strength);
        else
            runKickerRaw(0);
    }

    /**
     * Starts the hood moving towards a new angle.
     * 
     * @param newAngle The new angle for the hood to move to.
     */
    public void setHoodAngle(Angle newAngle) {
        if ((!hoodLimitSet) ||
                (newAngle.compareTo(TurretSubsystemConstants.MAX_HOOD_ANGLE) > 0) ||
                (newAngle.compareTo(TurretSubsystemConstants.MIN_HOOD_ANGLE) < 0)) {
            return;
        } else {
            double angleToRotations = newAngle
                .minus(TurretSubsystemConstants.MIN_HOOD_ANGLE)
                .div(TurretSubsystemConstants.HOOD_DEGREES_ROTATION_RATIO)
                .magnitude();
            hoodController.setSetpoint(angleToRotations, ControlType.kPosition);
            
        }
    }

    /**
     * Updates the data posted to the NetworkTables.
     * Control-critical values (turret angle, hood angle, flywheel speed) are
     * published every loop at 50 Hz. Dashboard-only visualizations (target circle,
     * control-target debug values) are throttled to every
     * {@link #TELEMETRY_LOOP_DIVISOR} loops (~10 Hz) to reduce NT bus load.
     */
    private void updateNetworkTables() {
        // Use cached signals refreshed at the top of periodic() — no additional CAN reads here.
        NetworkedConfig.Turret.setTurretAngle(anglePositionSignal.getValueAsDouble());
        NetworkedConfig.Turret.setHoodAngle(TurretSubsystemConstants.MIN_HOOD_ANGLE.plus(TurretSubsystemConstants.HOOD_DEGREES_ROTATION_RATIO.times(this.hoodEncoder.getPosition())).magnitude());
        NetworkedConfig.Turret.setFlywheelSpeed(shooterVelocitySignal.getValueAsDouble() * 60);
        NetworkedConfig.Turret.setTurretTarget(this.turretTarget.toString());

        // Rate-limit pure visualization data — dashboards don't need 50 Hz updates.
        if (periodicLoopCount % TELEMETRY_LOOP_DIVISOR == 0) {
            Translation3d targetPosition = getTargetFromEnum(this.turretTarget);
            NetworkedTelemetry.Pose.publishTargetCircle(targetPosition, Units.inchesToMeters(12));

            NetworkedTelemetry.Turret.setCTHoodAngle(currentControlTarget.hoodAngle);
            NetworkedTelemetry.Turret.setCTFlywheelRPM(currentControlTarget.rpm);
            NetworkedTelemetry.Turret.setCTValidTrajectory(currentControlTarget.properlySet);
            NetworkedTelemetry.Turret.setMotorCommStatus(
                rightShooterMotorConnected && leftShooterMotorConnected &&
                angleMotorConnected && hoodMotorConnected);
        }
    }

    /**
     * Updates the current turret target.
     */
    public void updateTurretTarget() {
        // ── Debug / LUT-tuning override ───────────────────────────────────────
        // If the SmartDashboard chooser has been set to anything other than "AUTO",
        // skip the field-zone logic and lock the turret to the chosen target.
        // Reset the chooser to "AUTO" on the dashboard to resume normal targeting.
        String targetOverride = NetworkedConfig.Debug.getTurretTargetOverride();
        if (!"AUTO".equals(targetOverride)) {
            switch (targetOverride) {
                case "RED_HUB":       this.turretTarget = TurretTarget.RED_HUB;        return;
                case "BLUE_HUB":      this.turretTarget = TurretTarget.BLUE_HUB;       return;
                case "AUDIENCE_CORNER": this.turretTarget = TurretTarget.AUDIENCE_CORNER; return;
                case "SCORING_CORNER":  this.turretTarget = TurretTarget.SCORING_CORNER;  return;
                case "NONE":          this.turretTarget = TurretTarget.NONE;            return;
                default: break; // unrecognised string — fall through to auto logic
            }
        }

        // ── Normal field-zone-based targeting ─────────────────────────────────
        FieldZone currentFieldZone = this.poseProvider.getFieldZone();
        Alliance currentAlliance = getCurrentAlliance();

        NetworkedConfig.Turret.setDebugFieldZone(currentFieldZone.toString());

        switch(currentFieldZone) {
            case RED_ALLIANCE_AUDIENCE_SIDE:
            case RED_ALLIANCE_SCORING_SIDE: {
                // If in red zone and we're red alliance, aim for hub
                if (currentAlliance == Alliance.Red) {
                    this.turretTarget = TurretTarget.RED_HUB;
                } else {
                    // Blue alliance in opponent zone
                    this.turretTarget = (currentFieldZone == FieldZone.RED_ALLIANCE_AUDIENCE_SIDE) 
                        ? TurretTarget.AUDIENCE_CORNER 
                        : TurretTarget.SCORING_CORNER;
                }
                break;
            }
            case BLUE_ALLIANCE_AUDIENCE_SIDE:
            case BLUE_ALLIANCE_SCORING_SIDE: {
                // If in blue zone and we're blue alliance, aim for hub
                if (currentAlliance == Alliance.Blue) {
                    this.turretTarget = TurretTarget.BLUE_HUB;
                } else {
                    // Blue alliance in opponent zone
                    this.turretTarget = (currentFieldZone == FieldZone.BLUE_ALLIANCE_AUDIENCE_SIDE) 
                        ? TurretTarget.AUDIENCE_CORNER 
                        : TurretTarget.SCORING_CORNER;
                }
                break;
            }
            case SCORING_NEUTRAL_ZONE: {
                // In neutral zone, aim for scoring corner
                this.turretTarget = TurretTarget.SCORING_CORNER;
                break;
            }
            case AUDIENCE_NEUTRAL_ZONE: {
                // In neutral zone, aim for audience corner
                this.turretTarget = TurretTarget.AUDIENCE_CORNER;
                break;
            }
            case OUT_OF_FIELD: {
                this.turretTarget = TurretTarget.NONE;
                break;
            }
        }
    }

    /**
     * Pulls data from the NetworkTables.
     */
    public void pullNetworkTableData() {
        var shooterSlot0config = ShooterFxConfigs.Slot0;
        
        shooterSlot0config.kP = NetworkedConfig.Turret.getShooterKP();
        shooterSlot0config.kI = NetworkedConfig.Turret.getShooterKI();
        shooterSlot0config.kD = NetworkedConfig.Turret.getShooterKD();
        shooterSlot0config.kV = NetworkedConfig.Turret.getShooterKV();
        shooterSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        this.rightShooterMotor.getConfigurator().apply(ShooterFxConfigs);

        var angleSlot0config = AngleFxConfigs.Slot0;

        angleSlot0config.kP = NetworkedConfig.Turret.getTurretKP();
        angleSlot0config.kI = NetworkedConfig.Turret.getTurretKI();
        angleSlot0config.kD = NetworkedConfig.Turret.getTurretKD();
        angleSlot0config.kS = NetworkedConfig.Turret.getTurretKS();
        angleSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        this.angleMotor.getConfigurator().apply(AngleFxConfigs);
        this.angleMotor.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(TurretSubsystemConstants.TURRET_GEAR_RATIO));
        
        // SparkMaxConfig hoodConfig = new SparkMaxConfig();
        // hoodConfig.inverted(hoodMotorInverted);
        // hoodConfig.closedLoop
        //     .p(NetworkedConfig.Turret.getHoodKP())
        //     .i(NetworkedConfig.Turret.getHoodKI())
        //     .d(NetworkedConfig.Turret.getHoodKD());
        
        // this.hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        System.out.println("Updated Turret PIDs.");
        
    }

    /**
     * Runs the SysId Quasistatic test for the turret flywheel.
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineFlywheel.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test for the turret flywheel.
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineFlywheel.dynamic(direction);
    }


    /**
     * Gets the coordinates of the current target from an enum.
     * 
     * @param target A {@link TurretTarget} enum to retrieve the postion from.
     * @return The target coordinates in the form of a {@link Translation3d} object.
     */
    private Translation3d getTargetFromEnum(TurretTarget target) {
        Alliance currentAlliance = getCurrentAlliance();
        
        switch (target) {
            case RED_HUB: {
                return TurretTargetPoints.RED_HUB;
            }
            case BLUE_HUB: {
                return TurretTargetPoints.BLUE_HUB;
            }
            case AUDIENCE_CORNER: {
                if (currentAlliance == Alliance.Red) {
                    return TurretTargetPoints.RED_LEFT_CORNER;
                } else {
                    return TurretTargetPoints.BLUE_RIGHT_CORNER;
                }
            }
            case SCORING_CORNER: {
                if (currentAlliance == Alliance.Red) {
                    return TurretTargetPoints.RED_RIGHT_CORNER;
                } else {
                    return TurretTargetPoints.BLUE_LEFT_CORNER;
                }
            }
            default: return new Translation3d();
        }    
    }
}
