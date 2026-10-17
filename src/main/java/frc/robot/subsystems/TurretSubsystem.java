// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.ShipOfTheseus;
import frc.robot.Constants.FieldConstants.FieldZone;
import frc.robot.Constants.FieldConstants.TurretTarget;
import frc.robot.Constants.FieldConstants.TurretTargetPoints;
import frc.robot.Constants.FieldConstants.TurretSafety;
import frc.robot.networking.NetworkedConfig;
import frc.robot.networking.NetworkedTelemetry;
import frc.robot.utils.AppliedGains;
import frc.robot.utils.targeting.BallisticShotSolver;
import frc.robot.utils.targeting.ShooterTable;
import frc.robot.utils.targeting.ShotPlanner;
import frc.robot.utils.targeting.ShotPlanner.ShotPlan;
import frc.robot.utils.targeting.ShotSolution;
import frc.robot.utils.targeting.ShotSolver;
import frc.robot.utils.targeting.TrackingLagEstimator;
import frc.robot.utils.targeting.TurretAngles;

/**
 * A subsystem that allows control of the flywheel, the hood, 
 * the turret's angle, and the kicker.
 */
public class TurretSubsystem extends SubsystemBase {
    private final PoseProvider poseProvider;

    private boolean hoodLimitSet = false;
    private boolean rightShooterMotorConnected, leftShooterMotorConnected,
        angleMotorConnected, hoodMotorConnected;
    private boolean hoodSafe;
    // private final Trigger hoodLowerLimitTrigger;
    private final Trigger feedTrigger;

    private final TalonFX rightShooterMotor;
    private final TalonFX leftShooterMotor;
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkClosedLoopController hoodController;
    private final TalonFX angleMotor;
    private final SparkMax kickerMotor;

    private final MutAngle turretAngleOffset;

    // Cached status signals — initialized in constructor after motors are created.
    // Calling .getVelocity()/.getPosition() repeatedly creates new signal objects
    // and triggers redundant CAN traffic; caching them reduces bus utilization.
    private StatusSignal<AngularVelocity> shooterVelocitySignal;
    private StatusSignal<edu.wpi.first.units.measure.Angle> anglePositionSignal;

    /** The current turret target as an enum. */
    public TurretTarget turretTarget = TurretTarget.NONE;

    /** Turret pivot position in the robot frame. */
    private static final Translation2d PIVOT_OFFSET = new Translation2d(
        TurretSubsystemConstants.TURRET_TO_ROBOT.getX(),
        TurretSubsystemConstants.TURRET_TO_ROBOT.getY());

    /** Empirical firing table loaded from {@code deploy/turret_data.json} at startup. */
    private final ShooterTable shooterTable = new ShooterTable(TurretSubsystemConstants.LUT_RANGE_MARGIN_IN);

    /**
     * Ballistic solver (the default). Independent of the firing table: it flies the lowest-speed arc
     * to the target, where a small hood error barely moves the landing point.
     */
    private final BallisticShotSolver physicsSolver = new BallisticShotSolver(
        new BallisticShotSolver.Params(
            TurretSubsystemConstants.PHYSICS_LAUNCH_HEIGHT_M,
            TurretSubsystemConstants.PHYSICS_EXIT_SPEED_INTERCEPT_MPS,
            TurretSubsystemConstants.PHYSICS_EXIT_SPEED_PER_RPM_MPS,
            TurretSubsystemConstants.PHYSICS_MIN_RPM,
            TurretSubsystemConstants.PHYSICS_MAX_RPM,
            TurretSubsystemConstants.MIN_HOOD_ANGLE.in(Degrees),
            TurretSubsystemConstants.MAX_HOOD_ANGLE.in(Degrees)),
        BallisticShotSolver::minimumSpeedHoodDeg,
        NetworkedConfig.Debug::getPhysicsExitSpeedScale);

    /**
     * This loop's shot plan from {@link #periodic()}: turret angle and shot, or no solution. Cached so
     * the readiness checks and telemetry use the same result without re-solving.
     */
    private ShotPlan lastPlan = null;

    /** The flywheel speed {@link #periodic()} commanded this loop (overrides and offsets included); 0 = coasting. */
    private double commandedRpm = 0.0;
    /** Added to the solver's RPM while shooting (see {@link #shootAutoTargetWithRPMOffset}). Cleared by {@link #stopShooter()}. */
    private double rpmOffset = 0.0;

    /** Measures how far the turret trails its setpoint while aiming; part of the shoot-on-the-fly latency. */
    private final TrackingLagEstimator turretLag = new TrackingLagEstimator(
        TurretSubsystemConstants.TURRET_LAG_SEED_S,
        TurretSubsystemConstants.TURRET_LAG_MIN_RATE_DEG_PER_S,
        TurretSubsystemConstants.TURRET_LAG_MAX_RATE_DEG_PER_S,
        TurretSubsystemConstants.TURRET_LAG_MAX_ERROR_DEG,
        TurretSubsystemConstants.TURRET_LAG_FILTER_TIME_CONSTANT_S,
        TurretSubsystemConstants.TURRET_LAG_MAX_S);
    /** The turret setpoint last sent by {@link #turnToAngle} (degrees, after wrapping into travel). */
    private double lastTurretSetpointDeg = Double.NaN;
    private double lastPeriodicTimeS = Double.NaN;

    // Controller-reset recovery (see recoverFromMotorResets()).
    private static final double RESET_RETRY_PERIOD_S = 1.0;
    private final Follower followerReq =
        new Follower(TurretSubsystemConstants.RIGHT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed);
    private boolean shooterConfigPending = false;
    private boolean angleConfigPending = false;
    private double lastTurretPositionRot = 0.0;
    private double nextResetRetryS = 0.0;

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
    private static final int SHOT_ARC_SAMPLES = 20;

    /**
     * When {@code true}, {@link #periodic()} skips the trajectory-tracking override
     * (hood angle, turret angle, flywheel idle) so the systems-check commands have
     * exclusive control of the mechanism. Set via {@link #setSystemsCheckMode(boolean)}.
     */
    private boolean systemsCheckMode = false;

    private double targetKickerSpeed = 0.0;

    /**
     * Timestamp (seconds) when the shooting state transitioned from active to inactive.
     * Used to implement smooth speed ramping: the drive speed gradually increases from
     * {@link TurretSubsystemConstants#SOTF_MAX_DRIVE_SPEED_MPS} back to full speed
     * over {@link TurretSubsystemConstants#SOTF_SPEED_RAMP_TIME_S}.
     * Initialized to -1 to indicate no ramp is in progress.
     */
    private double shootingStoppedTime = -1.0;

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

    /**
     * Runs the shot planner with both solvers a few hundred times on representative inputs so their
     * classes load and the JIT compiles them during boot. Otherwise the first enabled
     * {@link #periodic()} paid ~80-140 ms for it. Touches no motors or subsystem state (the physics
     * solver only reads its exit-speed scale from NetworkTables).
     */
    public void warmUpSolvers() {
        for (int i = 0; i < 300; i++) {
            Pose2d pose = new Pose2d(10.5 - (i % 40) * 0.1, 3.0 + (i % 25) * 0.08, Rotation2d.fromDegrees(i));
            ChassisSpeeds speeds = new ChassisSpeeds((i % 7) * 0.1 - 0.3, (i % 5) * 0.1 - 0.2, (i % 3) * 0.3 - 0.3);
            ShotPlanner.plan(pose, speeds, PIVOT_OFFSET, TurretTargetPoints.RED_HUB, shooterTable, 0.05, -0.02);
            ShotPlanner.plan(pose, speeds, PIVOT_OFFSET, TurretTargetPoints.RED_HUB, physicsSolver, 0.05, -0.02);
        }
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
    public TurretSubsystem(PoseProvider poseProvider, MutAngle turretOffsetAngle) {
        this.poseProvider = poseProvider;
        this.turretAngleOffset = turretOffsetAngle;

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

        this.leftShooterMotor.setControl(followerReq);

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
        hoodConfig.smartCurrentLimit(TurretSubsystemConstants.HOOD_CURRENT_LIMIT_AMPS);
        hoodConfig.closedLoop
            .p(TurretSubsystemConstants.HOOD_KP)
            .i(TurretSubsystemConstants.HOOD_KI)
            .d(TurretSubsystemConstants.HOOD_KD);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.hoodEncoder = hoodMotor.getEncoder();
        hoodController = this.hoodMotor.getClosedLoopController();

        // hoodLowerLimitTrigger = new Trigger(() -> this.hoodMotor.getForwardLimitSwitch().isPressed());
        // hoodLowerLimitTrigger.onTrue(Commands.runOnce((() -> {
        //     this.hoodEncoder.setPosition(0);
        // }), this));

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

        AngleFxConfigs.Feedback.SensorToMechanismRatio = TurretSubsystemConstants.TURRET_GEAR_RATIO;
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
        // Flywheel velocity drives flywheelAtSpeed()/feed gating, so keep it at the 50 Hz loop rate
        // rather than the 20 Hz the telemetry-only signals use.
        this.shooterVelocitySignal.setUpdateFrequency(50);
        this.anglePositionSignal.setUpdateFrequency(20);

        // Silence all other status frames on these motors that we never read.
        // Phoenix 6 motors broadcast many signals by default; this tells the firmware
        // to suppress any frame not explicitly configured above. (The CAN bus was ~54% utilized
        // while disabled, and the CAN receive thread was the biggest non-Java CPU consumer.)
        // Only the two signals above are read; the left shooter motor is a follower.
        this.rightShooterMotor.optimizeBusUtilization();
        this.leftShooterMotor.optimizeBusUtilization();
        this.angleMotor.optimizeBusUtilization();

        // Ready when the flywheel is commanded to spin and is at the commanded speed.
        this.feedTrigger = new Trigger(() -> runFlywheel && this.flywheelAtSpeed() && this.hoodSafe);

        // The first hasResetOccurred() call reports the power-on boot. Consume it here so only later
        // resets (brownouts, reboots) trigger recovery in periodic().
        this.rightShooterMotor.hasResetOccurred();
        this.leftShooterMotor.hasResetOccurred();
        this.angleMotor.hasResetOccurred();

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
     * The solver for the dashboard-selected mode: {@code "LUT"} is the firing table (debug comparison;
     * {@code null} if it failed to load); anything else is the ballistic solver.
     */
    private ShotSolver activeSolver() {
        String mode = NetworkedConfig.Debug.getTurretTargetingMode();
        if (mode != null && "LUT".equalsIgnoreCase(mode.trim())) return shooterTable.isLoaded() ? shooterTable : null;
        return physicsSolver;
    }

    /** This loop's shot, or {@code null} when there is no target or no solution. */
    private ShotSolution currentSolution() {
        return (lastPlan != null) ? lastPlan.solution() : null;
    }

    private static boolean isDebugOverrideActive() {
        return NetworkedConfig.Debug.isOverrideRPMEnabled() || NetworkedConfig.Debug.isOverrideHoodAngleEnabled();
    }

    /**
     * Plans the shot, publishes telemetry and drives the flywheel, hood and turret.
     *
     * <h4>Aiming pipeline (every 20 ms)</h4>
     * <ol>
     *   <li>Pick the target from field zone, alliance and the debug override.</li>
     *   <li>{@link ShotPlanner#plan} projects the pose forward by the latency, compensates for robot
     *       motion, and returns the turret angle and the shot from the selected solver.</li>
     *   <li>Apply the debug overrides and RPM offset, then command the motors.</li>
     * </ol>
     */
    @Override
    public void periodic() {
        // Single batched read — avoids creating new signal objects every loop.
        BaseStatusSignal.refreshAll(shooterVelocitySignal, anglePositionSignal);
        recoverFromMotorResets();

        // Update target selection before solving so this loop aims at the latest
        // field-zone/alliance/override target rather than the previous loop's target.
        updateTurretTarget();

        Pose2d currentPose = poseProvider.getPose();
        // getChassisSpeeds() is robot-relative (CTRE state.Speeds); the planner needs field-relative.
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            poseProvider.getChassisSpeeds(), currentPose.getRotation());
        Translation3d targetPosition = getTargetFromEnum(turretTarget);

        // Latency compensation: aim for where the robot will be when the turret actually gets there.
        // Measured parts (pose age, turret tracking lag) plus trims for the rest (see SOTF_LATENCY_TRIM_S).
        double poseAgeS = poseProvider.getPoseAgeSeconds();
        double turretLagS = turretLag.getLagSeconds();
        double latencyS = poseAgeS + turretLagS
            + TurretSubsystemConstants.SOTF_LATENCY_TRIM_S + NetworkedConfig.Debug.getLatencyTrimSeconds();
        NetworkedTelemetry.Turret.setLatency(poseAgeS, turretLagS, latencyS);
        ShotPlan plan = ShotPlanner.plan(
            currentPose,
            fieldSpeeds,
            PIVOT_OFFSET,
            (turretTarget == TurretTarget.NONE) ? null : targetPosition,
            activeSolver(),
            latencyS,
            TurretSubsystemConstants.SOTF_RANGE_LATENCY_COMPENSATION_S);
        lastPlan = plan;
        ShotSolution sol = plan.solution();

        // ── Telemetry: solver inputs and the raw solution ────────────────────
        // The actual (non-projected) pivot position, for field-visualization accuracy.
        NetworkedConfig.Turret.setRobotX(Units.metersToInches(plan.pivot().getX()));
        NetworkedConfig.Turret.setRobotY(Units.metersToInches(plan.pivot().getY()));
        NetworkedConfig.Turret.setRobotAngle(currentPose.getRotation().getDegrees());
        NetworkedConfig.Turret.setTargetX(Units.metersToInches(targetPosition.getX()));
        NetworkedConfig.Turret.setTargetY(Units.metersToInches(targetPosition.getY()));
        NetworkedConfig.Turret.setTargetHeight(Units.metersToInches(targetPosition.getZ()));
        double targetDistanceM = Double.isFinite(plan.targetDistanceM()) ? plan.targetDistanceM() : 0.0;
        NetworkedConfig.Turret.setDistanceToTarget(Units.metersToInches(targetDistanceM));

        checkHoodSafety(plan.pivot());

        NetworkedConfig.Turret.setValidTarget(sol != null);
        if (sol != null) {
            // The raw solution stays visible even while the debug overrides drive the motors.
            NetworkedConfig.Turret.setTargetHoodAngle(sol.hoodDeg());
            NetworkedConfig.Turret.setTargetRPM(sol.rpm());
        }
        publishSolverComparison(plan, targetPosition.getZ());

        // ── Debug / LUT-tuning overrides for actuation ────────────────────────
        // These replace the solved RPM and/or hood angle sent to the motors while
        // leaving the NT-published solution untouched, so you can compare the
        // override value against what the solver computed on the same dashboard.
        // Enable via Debug.Override RPM/Hood Angle Enabled on SmartDashboard.
        boolean validShot = (sol != null);
        double actuationRPM = validShot ? sol.rpm() + rpmOffset : 0;
        double actuationHoodDeg = validShot ? sol.hoodDeg() : TurretSubsystemConstants.MIN_HOOD_ANGLE.in(Degrees);
        if (NetworkedConfig.Debug.isOverrideRPMEnabled()) {
            actuationRPM = NetworkedConfig.Debug.getOverrideRPMValue();
            // Override is active even without a valid solution so you can spin
            // up while parked outside the table range.
            validShot = true;
        }
        if (NetworkedConfig.Debug.isOverrideHoodAngleEnabled()) {
            actuationHoodDeg = NetworkedConfig.Debug.getOverrideHoodAngleValue();
            validShot = true;
        }

        // ── Actuate ───────────────────────────────────────────────────────────
        boolean aiming = false;
        if (!systemsCheckMode) {
            boolean spin = runFlywheel && validShot
                && actuationRPM >= TurretSubsystemConstants.FLYWHEEL_MIN_CLOSED_LOOP_RPM;
            commandedRpm = spin ? actuationRPM : 0.0;
            if (spin) {
                rightShooterMotor.setControl(velocityReq.withVelocity(commandedRpm / 60.0));
            } else {
                rightShooterMotor.setControl(coastReq); // coast freely
            }

            if (validShot && shooting && hoodSafe) {
                this.setHoodAngle(Degrees.of(actuationHoodDeg));

                // With only RPM/hood overridden the solution still supplies the aim; with no
                // solution at all (pure override), the turret holds its last angle.
                if (sol != null) {
                    // Optional extra lead in the direction the robot is turning (TURRET_ROTATION_FF).
                    double omegaDegPerSec = Units.radiansToDegrees(fieldSpeeds.omegaRadiansPerSecond);
                    double turretDeg = plan.turretAngleDeg()
                        + omegaDegPerSec * TurretSubsystemConstants.TURRET_ROTATION_FF * 0.02;
                    this.turnToAngle(Degrees.of(turretDeg).plus(turretAngleOffset));
                    aiming = true;
                }
            } else {
                this.setHoodAngle(TurretSubsystemConstants.MIN_HOOD_ANGLE);
                // Do not call stopTurret() — that writes open-loop set(0) which fights
                // the closed-loop position hold from the last turnToAngle() call.
            }
        }

        // Measure turret tracking lag from auto-aim only (manual/debug turret commands don't count).
        // The position was read at the top of this loop, before this loop's setpoint was sent.
        double nowS = Timer.getFPGATimestamp();
        double dtS = Double.isNaN(lastPeriodicTimeS) ? 0.0 : nowS - lastPeriodicTimeS;
        lastPeriodicTimeS = nowS;
        turretLag.update(
            aiming,
            lastTurretSetpointDeg,
            getTurretAngleDegrees(),
            anglePositionSignal.getTimestamp().getLatency(),
            dtS);

        periodicLoopCount++;
        updateNetworkTables();
    }

    /** Publishes how far the LUT and physics solvers disagree at the current shot distance. */
    private void publishSolverComparison(ShotPlan plan, double targetHeightM) {
        if (!plan.hasSolution() || !shooterTable.isLoaded()) return;
        ShotSolution lut = shooterTable.solve(plan.virtualDistanceM(), targetHeightM);
        ShotSolution physics = physicsSolver.solve(plan.virtualDistanceM(), targetHeightM);
        if (lut == null || physics == null) return;
        NetworkedTelemetry.Turret.setLutPhysicsHoodDeltaDeg(lut.hoodDeg() - physics.hoodDeg());
        NetworkedTelemetry.Turret.setLutPhysicsRPMDelta(lut.rpm() - physics.rpm());
    }

    /**
     * Restores a motor controller's setup after it resets (brownout or reboot wipes it). Runs every
     * loop: {@code hasResetOccurred()} is cheap, but it consumes the flag, so this is its only caller.
     *
     * <p>The follower request is re-sent at once (non-blocking). Full configuration re-applies block
     * on CAN, but only run after a reset and retry once a second until they succeed. The turret
     * controller also loses its position, so it is restored to the last position read before the
     * reset; any movement while it was unpowered is not recovered.
     */
    private void recoverFromMotorResets() {
        if (rightShooterMotor.hasResetOccurred()) shooterConfigPending = true;
        if (angleMotor.hasResetOccurred()) angleConfigPending = true;
        if (leftShooterMotor.hasResetOccurred()) {
            leftShooterMotor.setControl(followerReq);
            DriverStation.reportWarning("Left flywheel controller reset; follower request re-sent.", false);
        }

        if (!angleConfigPending) {
            lastTurretPositionRot = anglePositionSignal.getValueAsDouble();
        }

        if (!(shooterConfigPending || angleConfigPending) || Timer.getFPGATimestamp() < nextResetRetryS) return;

        if (shooterConfigPending) {
            // ShooterFxConfigs.Slot0 holds the most recent NetworkTables gains.
            boolean ok = rightShooterMotor.getConfigurator().apply(ShooterFxConfigs).isOK();
            if (ok) {
                shooterVelocitySignal.setUpdateFrequency(50);
                rightShooterMotor.optimizeBusUtilization();
                var slot0 = ShooterFxConfigs.Slot0;
                shooterGains.record(true, slot0.kP, slot0.kI, slot0.kD, slot0.kV);
            }
            shooterConfigPending = !ok;
            reportResetRecovery("flywheel", ok);
        }
        if (angleConfigPending) {
            // The config carries the gear ratio, so it must land before the position is restored.
            boolean ok = angleMotor.getConfigurator().apply(AngleFxConfigs).isOK()
                && angleMotor.setPosition(lastTurretPositionRot).isOK();
            if (ok) {
                anglePositionSignal.setUpdateFrequency(20);
                angleMotor.optimizeBusUtilization();
                var slot0 = AngleFxConfigs.Slot0;
                angleGains.record(true, slot0.kP, slot0.kI, slot0.kD, slot0.kS);
            }
            angleConfigPending = !ok;
            reportResetRecovery("turret angle", ok);
        }
        if (shooterConfigPending || angleConfigPending) {
            nextResetRetryS = Timer.getFPGATimestamp() + RESET_RETRY_PERIOD_S;
        }
    }

    private static void reportResetRecovery(String motor, boolean ok) {
        if (ok) {
            DriverStation.reportWarning(motor + " controller reset; full configuration restored.", false);
        } else {
            DriverStation.reportWarning(motor + " controller reset; restoring its configuration failed, retrying.", false);
        }
    }

    /**
     * Signals the shooter to spin up and aim. {@link #periodic()} commands the RPM from this loop's
     * shot solution; with no solution the flywheel coasts.
     */
    public void shootAutoTarget() {
        shootAutoTargetWithRPMOffset(0);
    }

    /**
     * Like {@link #shootAutoTarget()}, with {@code rpmOffset} added to the solved RPM until
     * {@link #stopShooter()}. Not applied while the debug RPM override is on.
     */
    public void shootAutoTargetWithRPMOffset(double rpmOffset) {
        shooting = true;
        runFlywheel = true;
        this.rpmOffset = rpmOffset;
    }

    /**
     * Spins the flywheel directly at the given RPM, bypassing the solver and NetworkTables.
     * For systems checks: {@link #periodic()} only leaves the flywheel alone in systems-check mode.
     *
     * @param rpm The target flywheel speed in RPM.
     */
    public void spinFlywheelAtRPM(double rpm) {
        shooting = true;
        commandedRpm = rpm;
        rightShooterMotor.setControl(velocityReq.withVelocity(rpm / 60.0));
    }

    /**
     * Stops the shooter. Does not immediately stop due to inertia.
     */
    public void stopShooter() {
        shooting = false;
        this.runFlywheel = false;
        this.rpmOffset = 0.0;
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
                .withTimeout(10.0)
                .finallyDo((interrupted) -> {
                    hoodMotor.set(0);
                    stallTimer.stop();

                    if (!interrupted) {
                        // Hard stop confirmed — zero the relative encoder and allow
                        // setHoodAngle() to accept commands.
                        hoodEncoder.setPosition(0);
                        hoodLimitSet = true;
                    } else {
                        DriverStation.reportWarning("Hood homing timed out/interrupted; hood remains unhomed.", false);
                    }
                })
                .unless(() -> (hoodLimitSet || RobotBase.isSimulation()));
    }

    /** Returns true once the hood has been successfully homed. */
    public boolean isHoodHomed() {
        return hoodLimitSet;
    }

    /** Clears the hood-homed flag, forcing a re-home on the next homing command. */
    public void resetHoodHoming() {
        hoodLimitSet = false;
    }

    /**
     * True when this loop has a shot worth taking, or a debug override is on. Hub shots must be in the
     * solver's range; relay shots to the alliance-zone corners only need a solution, since the table
     * has no data that far out and a clamped lob is what those shots want.
     */
    public boolean hasValidTarget() {
        if (isDebugOverrideActive()) return true;
        ShotSolution sol = currentSolution();
        return sol != null && (sol.inRange() || !isTargetingHub());
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
        // When transitioning from shooting to not shooting, record the current time
        // to begin the speed ramp-up process.
        if (shootingActive && !active) {
            shootingStoppedTime = Timer.getFPGATimestamp();
        }
        // When transitioning from not shooting to shooting, reset the ramp timer.
        else if (!shootingActive && active) {
            shootingStoppedTime = -1.0;
        }
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
     * Returns the current speed cap for the drive system, accounting for SOTF limits
     * and the ramp-up process after shooting stops.
     *
     * <p>While shooting is active, returns {@link TurretSubsystemConstants#SOTF_MAX_DRIVE_SPEED_MPS}.
     * After shooting stops, linearly interpolates back to the full {@code maxSpeed} over
     * {@link TurretSubsystemConstants#SOTF_SPEED_RAMP_TIME_S}. Once the ramp completes,
     * returns the full {@code maxSpeed}.
     *
     * @param maxSpeed The robot's maximum speed in m/s when not shooting
     * @return The current speed cap to apply to drive commands
     */
    public double getRampedSpeedCap(double maxSpeed) {
        // If shooting is currently active, apply the SOTF speed cap with no ramp.
        if (shootingActive) {
            shootingStoppedTime = -1.0; // Ensure ramp timer is inactive.
            return TurretSubsystemConstants.SOTF_MAX_DRIVE_SPEED_MPS;
        }

        // If no ramp is in progress, return full speed.
        if (shootingStoppedTime < 0.0) {
            return maxSpeed;
        }

        // Calculate elapsed time since shooting stopped.
        double elapsedTime = Timer.getFPGATimestamp() - shootingStoppedTime;
        double rampDuration = TurretSubsystemConstants.SOTF_SPEED_RAMP_TIME_S;

        // If the ramp period has elapsed, return full speed and stop tracking the ramp.
        if (elapsedTime >= rampDuration) {
            shootingStoppedTime = -1.0;
            return maxSpeed;
        }

        // Linear interpolation: ramp from SOTF_MAX_DRIVE_SPEED_MPS to maxSpeed.
        double progress = elapsedTime / rampDuration;
        double rampedSpeed = TurretSubsystemConstants.SOTF_MAX_DRIVE_SPEED_MPS
            + (maxSpeed - TurretSubsystemConstants.SOTF_MAX_DRIVE_SPEED_MPS) * progress;

        return rampedSpeed;
    }

    /**
     * Returns the current angular-rate cap for the drive system, accounting for SOTF
     * limits and the ramp-up process after shooting stops.
     *
     * <p>While shooting is active, returns
     * {@link TurretSubsystemConstants#SOTF_MAX_ANGULAR_RATE_RAD_PER_SEC}. After
     * shooting stops, linearly interpolates back to the full {@code maxAngularRate}
     * over {@link TurretSubsystemConstants#SOTF_SPEED_RAMP_TIME_S}. Once the ramp
     * completes, returns the full {@code maxAngularRate}.
     *
     * @param maxAngularRate The robot's maximum angular rate in rad/s when not shooting
     * @return The current angular-rate cap to apply to drive commands
     */
    public double getRampedAngularRateCap(double maxAngularRate) {
        if (shootingActive) {
            shootingStoppedTime = -1.0; // Ensure ramp timer is inactive.
            return TurretSubsystemConstants.SOTF_MAX_ANGULAR_RATE_RAD_PER_SEC;
        }

        if (shootingStoppedTime < 0.0) {
            return maxAngularRate;
        }

        double elapsedTime = Timer.getFPGATimestamp() - shootingStoppedTime;
        double rampDuration = TurretSubsystemConstants.SOTF_SPEED_RAMP_TIME_S;

        if (elapsedTime >= rampDuration) {
            shootingStoppedTime = -1.0;
            return maxAngularRate;
        }

        double progress = elapsedTime / rampDuration;
        double rampedAngularRate = TurretSubsystemConstants.SOTF_MAX_ANGULAR_RATE_RAD_PER_SEC
            + (maxAngularRate - TurretSubsystemConstants.SOTF_MAX_ANGULAR_RATE_RAD_PER_SEC) * progress;

        return rampedAngularRate;
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
        // The signal is in mechanism rotations (the gear ratio is configured on the controller).
        return anglePositionSignal.getValueAsDouble() * 360.0;
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
        targetKickerSpeed = 0;
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
     * Starts the turret moving towards a new angle. An angle outside the turret's travel is replaced
     * by its equivalent (±360°) inside it, so every direction stays reachable.
     *
     * @param newAngle The angle to move toward (CW-positive from the back of the robot).
     */
    public void turnToAngle(Angle newAngle) {
        double deg = newAngle.in(Degrees);
        if (!Double.isFinite(deg)) return;
        double travelDeg = TurretAngles.wrapIntoTravel(
            deg,
            TurretSubsystemConstants.MIN_TURRET_ANGLE.in(Degrees),
            TurretSubsystemConstants.MAX_TURRET_ANGLE.in(Degrees));
        lastTurretSetpointDeg = travelDeg;
        // Always issue the closed-loop position request — even when within tolerance
        // this keeps the motor actively holding position instead of going open-loop (set(0))
        // which would allow the mechanism to drift under gravity or mechanism compliance.
        angleMotor.setControl(anglePositionReq.withPosition(Degrees.of(travelDeg)));
    }

    public void turnRaw(double power) {
        angleMotor.set(power);
    }

    /**
     * True when the flywheel is being driven closed-loop and is within
     * {@link TurretSubsystemConstants#FLYWHEEL_READY_TOLERANCE_RPS} of the speed actually commanded
     * (debug override and RPM offset included).
     */
    public boolean flywheelAtSpeed() {
        if (commandedRpm <= 0) return false;
        return Math.abs(shooterVelocitySignal.getValueAsDouble() - commandedRpm / 60.0)
            <= TurretSubsystemConstants.FLYWHEEL_READY_TOLERANCE_RPS;
    }

    /**
     * Runs the kicker motor only when there is a shot worth taking ({@link #hasValidTarget()}) and
     * the flywheel is at its commanded speed. Stops the kicker otherwise so a game piece is never
     * fed into an under-speed flywheel.
     *
     * @param strength The strength to run the motor at, on a scale of -1 (full reverse) to 1 (full forward).
     */
    public void kick(double strength) {
        if (hasValidTarget() && flywheelAtSpeed()) {
            runKickerRaw(strength);
            targetKickerSpeed = strength;
        } else {
            runKickerRaw(0);
            targetKickerSpeed = 0;
        }
    }

    /**
     * Starts the hood moving towards a new angle, clamped to the hood's travel. Ignored until the
     * hood has been homed.
     *
     * @param newAngle The new angle for the hood to move to (degrees from vertical).
     */
    public void setHoodAngle(Angle newAngle) {
        double deg = newAngle.in(Degrees);
        if (!hoodLimitSet || !Double.isFinite(deg)) return;
        double minDeg = TurretSubsystemConstants.MIN_HOOD_ANGLE.in(Degrees);
        double clampedDeg = MathUtil.clamp(deg, minDeg, TurretSubsystemConstants.MAX_HOOD_ANGLE.in(Degrees));
        double rotations = (clampedDeg - minDeg) / TurretSubsystemConstants.HOOD_DEGREES_ROTATION_RATIO.in(Degrees);
        hoodController.setSetpoint(rotations, ControlType.kPosition);
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
        NetworkedConfig.Turret.setTurretAngle(getTurretAngleDegrees());
        NetworkedConfig.Turret.setHoodAngle(getHoodAngleDegrees());
        NetworkedConfig.Turret.setFlywheelSpeed(getFlywheelRPM());
        NetworkedConfig.Turret.setTurretTarget(this.turretTarget.toString());

        ShotSolution sol = currentSolution();
        NetworkedTelemetry.Turret.setSelectedSolver((sol != null) ? sol.source().name() : "NONE");
        NetworkedTelemetry.Turret.setShotInRange(sol != null && sol.inRange());
        NetworkedTelemetry.Turret.setTimeOfFlightS((sol != null) ? sol.timeOfFlightS() : 0.0);
        NetworkedTelemetry.Turret.setCommandedRPM(commandedRpm);

        // Rate-limit pure visualization data — dashboards don't need 50 Hz updates.
        if (periodicLoopCount % TELEMETRY_LOOP_DIVISOR == 0) {
            Translation3d targetPosition = getTargetFromEnum(this.turretTarget);
            NetworkedTelemetry.Pose.publishTargetCircle(targetPosition, Units.inchesToMeters(12));
            publishShotArcVisualization();

            NetworkedTelemetry.Turret.setMotorCommStatus(
                rightShooterMotorConnected && leftShooterMotorConnected &&
                angleMotorConnected && hoodMotorConnected);
            
            NetworkedConfig.Turret.setKickerTarget(targetKickerSpeed);
            NetworkedConfig.Turret.setKickerSpeed(kickerMotor.get());
        }
    }

    /**
     * Publishes the predicted ball path (relative to the robot, as if stationary) using the physics
     * solver's exit-speed model. A physics shot's arc ends at the target; a LUT shot's arc shows
     * where the model thinks the table's RPM would land.
     */
    private void publishShotArcVisualization() {
        ShotSolution sol = currentSolution();
        if (sol == null || !shooting || turretTarget == TurretTarget.NONE) {
            NetworkedTelemetry.Turret.setShotArc(new Pose3d[0]);
            return;
        }

        double startX = lastPlan.pivot().getX();
        double startY = lastPlan.pivot().getY();
        double startZ = physicsSolver.launchHeightM();

        double yawRad = lastPlan.aimBearingRad();
        double launchPitchRad = Units.degreesToRadians(90.0 - sol.hoodDeg());

        double launchSpeedMps = physicsSolver.exitSpeedMps(sol.rpm());
        double vx0 = launchSpeedMps * Math.cos(launchPitchRad) * Math.cos(yawRad);
        double vy0 = launchSpeedMps * Math.cos(launchPitchRad) * Math.sin(yawRad);
        double vz0 = launchSpeedMps * Math.sin(launchPitchRad);

        double totalT = Math.max(0.05, sol.timeOfFlightS());
        Pose3d[] samples = new Pose3d[SHOT_ARC_SAMPLES];

        for (int i = 0; i < SHOT_ARC_SAMPLES; i++) {
            double frac = (SHOT_ARC_SAMPLES == 1) ? 0.0 : (double) i / (SHOT_ARC_SAMPLES - 1);
            double t = frac * totalT;

            double x = startX + vx0 * t;
            double y = startY + vy0 * t;
            double z = startZ + vz0 * t - 0.5 * BallisticShotSolver.GRAVITY_MPS2 * t * t;

            samples[i] = new Pose3d(x, y, Math.max(0.0, z), new Rotation3d());
        }

        NetworkedTelemetry.Turret.setShotArc(samples);
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
    // Gains currently on the motors (seeded with the constructor's values). Each CTRE apply() blocks
    // the robot thread on a CAN round trip, so unchanged NetworkTables values skip it.
    private final AppliedGains shooterGains = new AppliedGains(
        TurretSubsystemConstants.SHOOTER_KP, TurretSubsystemConstants.SHOOTER_KI,
        TurretSubsystemConstants.SHOOTER_KD, TurretSubsystemConstants.SHOOTER_KV);
    private final AppliedGains angleGains = new AppliedGains(
        TurretSubsystemConstants.TURRET_KP, TurretSubsystemConstants.TURRET_KI,
        TurretSubsystemConstants.TURRET_KD, TurretSubsystemConstants.TURRET_KS);

    /** Applies the NetworkTables gains to the motors unconditionally. */
    public void pullNetworkTableData() {
        applyNetworkTableData(true);
    }

    /**
     * Applies the NetworkTables gains only if they differ from what is already on the motors. (A
     * controller reset is handled every loop by {@link #recoverFromMotorResets()}.)
     */
    public void applyChangedNetworkTableData() {
        applyNetworkTableData(false);
    }

    private void applyNetworkTableData(boolean force) {
        // Slot0 only: re-applying the whole config rewrites unrelated settings. The gains are also
        // stored in the full configs so a reset recovery restores the latest values.
        double[] shooter = {
            NetworkedConfig.Turret.getShooterKP(), NetworkedConfig.Turret.getShooterKI(),
            NetworkedConfig.Turret.getShooterKD(), NetworkedConfig.Turret.getShooterKV()};
        if (shooterGains.shouldApply(force, shooter)) {
            var shooterSlot0config = ShooterFxConfigs.Slot0;
            shooterSlot0config.kP = shooter[0];
            shooterSlot0config.kI = shooter[1];
            shooterSlot0config.kD = shooter[2];
            shooterSlot0config.kV = shooter[3];
            shooterSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;
            var status = this.rightShooterMotor.getConfigurator().apply(shooterSlot0config);
            shooterGains.record(status.isOK(), shooter);
            reportGainApply("shooter", status.isOK());
        }

        double[] angle = {
            NetworkedConfig.Turret.getTurretKP(), NetworkedConfig.Turret.getTurretKI(),
            NetworkedConfig.Turret.getTurretKD(), NetworkedConfig.Turret.getTurretKS()};
        if (angleGains.shouldApply(force, angle)) {
            var angleSlot0config = AngleFxConfigs.Slot0;
            angleSlot0config.kP = angle[0];
            angleSlot0config.kI = angle[1];
            angleSlot0config.kD = angle[2];
            angleSlot0config.kS = angle[3];
            angleSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;
            var status = this.angleMotor.getConfigurator().apply(angleSlot0config);
            angleGains.record(status.isOK(), angle);
            reportGainApply("turret angle", status.isOK());
        }

        // SparkMaxConfig hoodConfig = new SparkMaxConfig();
        // hoodConfig.inverted(hoodMotorInverted);
        // hoodConfig.closedLoop
        //     .p(NetworkedConfig.Turret.getHoodKP())
        //     .i(NetworkedConfig.Turret.getHoodKI())
        //     .d(NetworkedConfig.Turret.getHoodKD());
        
        // this.hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private static void reportGainApply(String motor, boolean ok) {
        if (!ok) {
            DriverStation.reportWarning("Failed to apply " + motor + " motor gains over CAN; will retry.", false);
        } else {
            System.out.println("Updated " + motor + " PIDs.");
        }
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

    public Angle getTargetAngleFromPos() {
        Pose2d currentRobotPose = this.poseProvider.getPose();
        Translation3d currentTargetPose = getTargetFromEnum(this.turretTarget);
        double deltaX = currentTargetPose.getX() - currentRobotPose.getX();
        double deltaY = currentTargetPose.getY() - currentRobotPose.getY();
        Angle targetAngleFieldRelative = Radians.of(Math.atan2(deltaY, deltaX));
        return targetAngleFieldRelative/*.minus(currentRobotPose.getRotation().getMeasure())*/;
    }

    public Angle getRelativeAngleToTarget() {
        Pose2d currentRobotPose = this.poseProvider.getPose();
        Translation3d currentTargetPose = getTargetFromEnum(this.turretTarget);
        double deltaX = currentTargetPose.getX() - currentRobotPose.getX();
        double deltaY = currentTargetPose.getY() - currentRobotPose.getY();
        Angle targetAngleFieldRelative = Radians.of(Math.atan2(deltaY, deltaX));
        return targetAngleFieldRelative.minus(currentRobotPose.getRotation().getMeasure());
    }

    public boolean isTurretSafe() {
        return this.hoodSafe;
    }

    private void checkHoodSafety(Translation2d turretPos) {
        this.hoodSafe = !(
            TurretSafety.RED_DEPOT_SIDE_TRENCH.contains(turretPos)    ||
            TurretSafety.RED_OUTPOST_SIDE_TRENCH.contains(turretPos)  ||
            TurretSafety.BLUE_DEPOT_SIDE_TRENCH.contains(turretPos)   ||
            TurretSafety.BLUE_OUTPOST_SIDE_TRENCH.contains(turretPos)
        );
    }
}
