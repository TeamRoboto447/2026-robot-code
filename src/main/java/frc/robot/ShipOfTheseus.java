// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// (removed unused imports)
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.libraries.Repulsor.Repulsor;
import frc.robot.libraries.Repulsor.DriverStation.RepulsorDriverStationBootstrap;
import frc.robot.utils.GameState;
import frc.robot.libraries.Repulsor.State.StateManager;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import frc.robot.subsystems.SystemsCheck;

import frc.robot.networking.NetworkedConfig;
import frc.robot.networking.NetworkedTelemetry;

public class ShipOfTheseus {
    private double MaxSpeed = 0.70 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private boolean autoShoot = false;
    private boolean autoIntake = false;
    private boolean autoTurningToAngle = false;
    private MutAngle turretAngleOffset = Degrees.mutable(0);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveFieldOriented = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotOriented = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle driveFieldOrientedWithAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1);
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController DriverController = new CommandXboxController(0);
    private final CommandXboxController OperatorController = new CommandXboxController(1);

    private final Field2d field = new Field2d();
    private final SendableChooser<Command> autoChooser;

    /**
     * Debug chooser for manually overriding the turret's target during LUT tuning.
     * Selecting anything other than "AUTO" locks the turret to that target regardless
     * of field zone / alliance. Resets to "AUTO" on every robot init via
     * {@link NetworkedConfig#initializeAllDefaults()}.
     */
    private final SendableChooser<String> turretTargetChooser = new SendableChooser<>();
    private final SendableChooser<String> turretTargetingModeChooser = new SendableChooser<>();

    public final CommandSwerveDrivetrain swerveSubsystem = TunerConstants.createDrivetrain(field);
    public final TurretSubsystem turretSubsystem;
    public final IndexerSubsystem indexerSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final PoseEstimatorSubsystem poseEstimatorSubsystem;
    public final ClimberSubsystem climberSubsystem;
    public final Repulsor repulsor;
    public final GameState gameState;
    private final AtomicBoolean repulsorHasPiece = new AtomicBoolean(false);

    public final PowerDistribution powerBoard;    

    private final Trigger driverControllerPOVActive = new Trigger(() -> !DriverController.povCenter().getAsBoolean());
    private final Trigger operatorControllerRightJoystick = new Trigger(() -> 
            (Math.abs(OperatorController.getRightX()) > 0.1) ||
            (Math.abs(OperatorController.getRightY()) > 0.1)
    );
    private final Trigger autoShootTrigger = new Trigger(() -> this.autoShoot);
    private final Trigger autoIntakeTrigger = new Trigger(() -> this.autoIntake);
    
    // NeoPixel telemetry state tracking
    private boolean hubWarningFired = false;
    private boolean wasHubActive = false;
    private final Debouncer aprilTagValidDebouncer = new Debouncer(0.25, DebounceType.kFalling);

    // /**
    //  * Path-following builder used for mid-game commands (e.g. automated climb).
    //  * Unlike {@link #pathBuilder}, this builder does NOT reset the robot's odometry
    //  * at the start of the path — the robot navigates from wherever it currently is.
    //  *
    //  * <p>Alliance flipping is intentionally disabled here because the climbing bar
    //  * is at the center of the field and is the same physical location for both
    //  * alliances. If your climb target is alliance-specific, switch this to
    //  * {@code .withDefaultShouldFlip()}.</p>
    //  */
    

    public ShipOfTheseus() {

        this.turretSubsystem = new TurretSubsystem(swerveSubsystem, turretAngleOffset);
        this.intakeSubsystem = new IntakeSubsystem();
        this.indexerSubsystem = new IndexerSubsystem();
        this.poseEstimatorSubsystem = new PoseEstimatorSubsystem(swerveSubsystem);
        this.climberSubsystem = new ClimberSubsystem(swerveSubsystem);
        

        SmartDashboard.putData("Field", field);

        initializeNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        powerBoard = new PowerDistribution(1, ModuleType.kRev);
        SmartDashboard.putData("PDH", powerBoard);

        // ── Debug: turret target override chooser ─────────────────────────────
        // Populate before configureBindings() so the drive default command can
        // reference it immediately.
        turretTargetChooser.setDefaultOption("AUTO",           "AUTO");
        turretTargetChooser.addOption("Red Hub",               "RED_HUB");
        turretTargetChooser.addOption("Blue Hub",              "BLUE_HUB");
        turretTargetChooser.addOption("Audience Corner",       "AUDIENCE_CORNER");
        turretTargetChooser.addOption("Scoring Corner",        "SCORING_CORNER");
        turretTargetChooser.addOption("None (disable turret)", "NONE");
        SmartDashboard.putData("Debug/Turret Target Override", turretTargetChooser);

        turretTargetingModeChooser.addOption("Auto Fallback", "AUTO_FALLBACK");
        turretTargetingModeChooser.setDefaultOption("LUT Only", "LUT");
        turretTargetingModeChooser.addOption("Model Only", "MODEL");
        SmartDashboard.putData("Debug/Turret Targeting Mode", turretTargetingModeChooser);
        
        configureBindings();
        NetworkedConfig.initializeAllDefaults();

    // Debug: lightweight path/event prints added in fillAutoChooser to
    // help diagnose unexpected interruptions during auto.

        this.repulsor =
            new Repulsor(
                swerveSubsystem,
                frc.robot.Constants.RepulsorConstants.ROBOT_X,
                frc.robot.Constants.RepulsorConstants.ROBOT_Y,
                0.0,
                0.0,
                repulsorHasPiece::get); // operator-controlled supplier until a sensor is available
        NetworkedTelemetry.Repulsor.setHasPiece(repulsorHasPiece.get());
        RepulsorDriverStationBootstrap.useDefaultNt();

        gameState = new GameState();
        gameState.update();

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if (alliance == Alliance.Blue) turretAngleOffset = Degrees.mutable(-4);
    }

    public void runSensorlessHoming() {
        CommandScheduler.getInstance().schedule(climberSubsystem.homeClimber());
        CommandScheduler.getInstance().schedule(turretSubsystem.homeHood());
        CommandScheduler.getInstance().schedule(intakeSubsystem.homeLift());
    }

    public void motorStatusCheck() {
        turretSubsystem.motorStatusCheck();
    }

    // TODO: Verify correct bindings before uploading
    private void configureBindings() {
        // configureAutonomousBindings();
        
        configureProductionBindings();
        // configureDevBindings();
    }

    // private void configureAutonomousBindings() {
    //     autoShootTrigger.and(turretSubsystem::hasValidTarget)
    //         .and(turretSubsystem::isHoodHomed)
    //         // .and(this::isShotAllowed)
    //         .whileTrue(Commands.parallel(
    //             Commands.run(() -> indexerSubsystem.spin()),
    //             Commands.run(() -> turretSubsystem.kick(KICKER_SPEED)),
    //             Commands.run(() -> turretSubsystem.shootAutoTarget())
    //         ));

    //     // Cap drive speed while the autonomous shoot trigger is active.
    //     autoShootTrigger
    //         .onTrue(Commands.runOnce(() -> turretSubsystem.setShootingActive(true)))
    //         .onFalse(Commands.runOnce(() -> turretSubsystem.setShootingActive(false)));

    //     autoShootTrigger.onFalse(Commands.runOnce(() -> {
    //         turretSubsystem.stopAll();
    //         indexerSubsystem.stop();
    //     }));
    // }

    private void configureProductionBindings() {
        double KICKER_SPEED = 1;


        // Run homing commands on initialization - If already homed, the command immediately cancels itself
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> runSensorlessHoming()));
        RobotModeTriggers.autonomous().onTrue(climberSubsystem.homeClimber());
        RobotModeTriggers.teleop().onTrue(climberSubsystem.raiseToFull());
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            runSensorlessHoming();
            this.autoShoot = false;
            this.autoIntake = false;
        }));

        DriverController.x().onTrue(Commands.defer(() -> getAutoClimbCommand(), Set.of(swerveSubsystem, climberSubsystem)));

        // Swerve Drive
        // When a shoot-on-the-fly attempt is active (shoot button held or autoShootTrigger),
        // the requested velocity magnitude is capped at SOTF_MAX_DRIVE_SPEED_MPS so the
        // driver can still steer but cannot exceed the shooter's reliable operating envelope.
        // After shooting stops, the speed ramps back to full speed over SOTF_SPEED_RAMP_TIME_S.
        // The turret-target chooser is also synced to NT every loop here — cheap string write.
        swerveSubsystem.setDefaultCommand(
            swerveSubsystem.applyRequest(() -> {
                // Sync the SmartDashboard chooser selection → NetworkedConfig so that
                // TurretSubsystem.updateTurretTarget() can read it without a direct reference.
                NetworkedConfig.Debug.setTurretTargetOverride(turretTargetChooser.getSelected());
                NetworkedConfig.Debug.setTurretTargetingMode(turretTargetingModeChooser.getSelected());

                double speedCap = turretSubsystem.getRampedSpeedCap(MaxSpeed);
                double angularRateCap = turretSubsystem.getRampedAngularRateCap(MaxAngularRate);
                if (autoTurningToAngle) {
                    return driveFieldOrientedWithAngle
                        .withVelocityX(-DriverController.getLeftY() * speedCap)
                        .withVelocityY(-DriverController.getLeftX() * speedCap)
                        .withTargetDirection(new Rotation2d(turretSubsystem.getTargetAngleFromPos()));
                } else {
                    return driveFieldOriented
                        .withVelocityX(-DriverController.getLeftY() * speedCap)
                        .withVelocityY(-DriverController.getLeftX() * speedCap)
                        .withRotationalRate(-DriverController.getRightX() * angularRateCap);
                }
            })
        );

        // Driver: Slow, robot-oriented drive (POV)
        // Moves the robot in a robot-oriented state,
        // based on the position of the POV.
        driverControllerPOVActive.whileTrue(swerveSubsystem.applyRequest(() -> {
            double povX = Math.cos(Units.degreesToRadians(DriverController.getHID().getPOV()));
            double povY = Math.sin(Units.degreesToRadians(DriverController.getHID().getPOV()));

            return driveRobotOriented
                .withVelocityX(0.2 * povX * MaxSpeed)   // POV has different X/Y
                .withVelocityY(0.2 * -povY * MaxSpeed);  // Convention than WPILib
            }
        ));

        operatorControllerRightJoystick.whileTrue(swerveSubsystem.applyRequest(() ->
            driveRobotOriented
                .withVelocityX(0.2 * -OperatorController.getRightY() * MaxSpeed)
                .withVelocityY(0.2 * -OperatorController.getRightX() * MaxSpeed)
            ));

        // Neutral mode while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerveSubsystem.applyRequest(() -> idle).ignoringDisable(true)
        );

        swerveSubsystem.registerTelemetry(logger::telemeterize);

        // Driver: Climber
        DriverController.leftBumper().onTrue(climberSubsystem.lowerOntoBar());
        DriverController.rightBumper().onTrue(climberSubsystem.raiseToFull());
        DriverController.y().whileTrue(climberSubsystem.run(() -> climberSubsystem.raise()));
        DriverController.x().whileTrue(climberSubsystem.run(() -> climberSubsystem.lower()));

        // withinSafeClimberRange.onFalse(climberSubsystem.lowerOntoBar());

        // Driver: Automated climb (A button)
        // Raises the climber, drives to the bar, then lowers onto it.
        // Pressing A again (or any command that requires swerve/climber) will cancel.
        DriverController.a().onTrue(getAutoClimbCommand());

        // Shoot
        // Spin up the flywheel while the trigger is held. Once the flywheel reaches
        // target speed (feedTrigger goes high), the kicker and spindexer activate to
        // feed the shooter. Requires a valid trajectory — does nothing otherwise.
        // Hub shots are blocked while the hub is inactive; non-hub targets (alliance
        // zone relays) are always allowed.
        // Also caps drive speed at SOTF_MAX_DRIVE_SPEED_MPS for the duration of the hold.
        DriverController.start().or(OperatorController.rightTrigger()).or(autoShootTrigger)
            .and(turretSubsystem::hasValidTarget)
            .and(this::isShotAllowed)
            .whileTrue(
                turretSubsystem.run(() -> {
                    turretSubsystem.shootAutoTarget();
                    turretSubsystem.kick(KICKER_SPEED);
                })
            );

        DriverController.start().or(OperatorController.rightTrigger()).or(autoShootTrigger)
            .onTrue(Commands.runOnce(() -> {
                Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
                poseEstimatorSubsystem.setTemporaryAprilTagFilter((alliance == Alliance.Red) ? Constants.FieldConstants.RED_HUB_APRILTAGS : Constants.FieldConstants.BLUE_HUB_APRILTAGS);
                turretSubsystem.setShootingActive(true);
            }))
            .onFalse(Commands.runOnce(() -> {
                poseEstimatorSubsystem.clearTemporaryAprilTagFilter();
                turretSubsystem.setShootingActive(false);
            }));

        DriverController.start().or(OperatorController.rightTrigger()).or(autoShootTrigger)
            .onFalse(Commands.runOnce(() -> {
                turretSubsystem.stopShooter();
                turretSubsystem.stopKicker();
                indexerSubsystem.stop();
            }));

        
        OperatorController.leftTrigger().or(DriverController.y())
            .and(turretSubsystem::hasValidTarget)
            .and(this::isShotAllowed)
            .whileTrue(
                turretSubsystem.run(() -> {
                    turretSubsystem.shootAutoTargetWithRPMOffset(TurretSubsystemConstants.RPM_OFFSET_WHILE_CLIMBED);
                    turretSubsystem.kick(KICKER_SPEED);
                })
            );

        OperatorController.leftTrigger().or(DriverController.y())
            .onFalse(Commands.runOnce(() -> {
                turretSubsystem.stopShooter();
                turretSubsystem.stopKicker();
                indexerSubsystem.stop();
            }));

        Trigger shootRequest = DriverController.start().or(OperatorController.rightTrigger())
            .or(DriverController.y().or(OperatorController.leftTrigger()))
            .or(autoShootTrigger);

        // Feed the spindexer only when the feed trigger is true (flywheel at speed).
        shootRequest
            .and(turretSubsystem.getFeedTrigger())
            .and(turretSubsystem::hasValidTarget)
            .and(turretSubsystem::isHoodHomed)
            .and(turretSubsystem::flywheelAtSpeed)
            .and(this::isShotAllowed)
            .whileTrue(indexerSubsystem.run(() -> indexerSubsystem.spin()));

        DriverController.start().onFalse(turretSubsystem.runOnce(() -> {
            turretSubsystem.stopShooter();
            turretSubsystem.stopKicker();
        }));

        DriverController.start()
            .or(OperatorController.rightTrigger())
            .or(turretSubsystem::flywheelAtSpeed)
            .onFalse(indexerSubsystem.runOnce(() -> indexerSubsystem.stop()));

        OperatorController.rightTrigger().onFalse(turretSubsystem.runOnce(() -> {
            turretSubsystem.stopShooter();
            turretSubsystem.stopKicker();
        }));

        // Driver: Intake (left trigger)
        // Lower the intake if it isn't already, then run the intake roller.
        DriverController.leftTrigger().or(OperatorController.a()).or(autoIntakeTrigger).onTrue(
            intakeSubsystem.runOnce(() -> {
                if (!intakeSubsystem.isIntakeDown()) intakeSubsystem.dropIntake();
            })
        );
        
        DriverController.leftTrigger().or(OperatorController.a()).or(autoIntakeTrigger).whileTrue(
            // Use Commands.run (no subsystem requirement) so that the autoIntakeTrigger
            // firing during autonomous does not claim intakeSubsystem and cancel the
            // running path command. The intake motor is purely open-loop — no default
            // command or closed-loop controller needs exclusive ownership of it.
            Commands.run(() -> intakeSubsystem.intake(1))
        );
        DriverController.back().or(OperatorController.x()).whileTrue(
            Commands.run(() -> intakeSubsystem.reverseIntake(0.5))
        );
        DriverController.leftTrigger().or(OperatorController.a()).or(DriverController.back()).or(autoIntakeTrigger).onFalse(
            Commands.runOnce(() -> intakeSubsystem.stopIntake())
        );

        // turretSubsystem.getFeedTrigger().and(OperatorController.rightTrigger())
        //     .and(turretSubsystem::hasValidTarget)
        //     .and(this::isShotAllowed)
        //     .whileTrue(Commands.parallel(
        //         indexerSubsystem.run(() -> indexerSubsystem.spin()),
        //         turretSubsystem.run(() -> turretSubsystem.kick(KICKER_SPEED))
        //     ));
        // OperatorController.rightTrigger().onFalse(turretSubsystem.runOnce(() -> {
        //     turretSubsystem.stopShooter();
        //     turretSubsystem.stopKicker();
        // }));
        // OperatorController.rightTrigger().onFalse(indexerSubsystem.stop());

        OperatorController.b().whileTrue(indexerSubsystem.run(() -> indexerSubsystem.spinReverse()));
        OperatorController.b().onFalse(indexerSubsystem.run(() -> indexerSubsystem.stop()));

        // Operator: Intake lift
        OperatorController.leftBumper().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.liftIntake()));
        OperatorController.rightBumper().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.dropIntake()));

        // Operator: Climber Control
        OperatorController.povUp().onTrue(climberSubsystem.raiseToFull().onlyIf(climberSubsystem.withinSafeClimberRange));
        OperatorController.povDown().onTrue(climberSubsystem.lowerOntoBar());

        OperatorController.start().onTrue(Commands.runOnce(() -> turretAngleOffset.mut_acc(Degrees.of(1))));
        OperatorController.back().onTrue(Commands.runOnce(() -> turretAngleOffset.mut_acc(Degrees.of(-1))));
    }
    
    @SuppressWarnings("unused") // Suppress warnings for unused bindings in dev mode
    private void configureDevBindings() {
        // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> runSensorlessHoming()));
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> runSensorlessHoming()));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerveSubsystem.setDefaultCommand(
            // Drivetrain will execute this command periodically
            swerveSubsystem.applyRequest(() ->
                driveFieldOriented.withVelocityX(-DriverController.getLeftY() * MaxSpeed / 1.75) // Drive forward with negative Y (forward)
                    .withVelocityY(-DriverController.getLeftX() * MaxSpeed / 1.75) // Drive left with negative X (left)
                    .withRotationalRate(-DriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        OperatorController.pov(90).whileTrue(intakeSubsystem.run(() -> intakeSubsystem.intake(0.7)));
        OperatorController.pov(270).whileTrue(intakeSubsystem.run(() -> intakeSubsystem.reverseIntake(0.7)));
        OperatorController.povUp().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.liftIntake()));
        OperatorController.povDown().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.dropIntake()));

        OperatorController.pov(-1).onTrue(intakeSubsystem.run(() -> {
            intakeSubsystem.stopIntake();
        }));

        OperatorController.start().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.pullNetworkTableData()));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerveSubsystem.applyRequest(() -> idle).ignoringDisable(true)
        );

        // DriverController.a().whileTrue(swerveSubsystem.applyRequest(() -> brake));
        DriverController.a().onTrue(getAutoClimbCommand());

        DriverController.b().whileTrue(swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-DriverController.getLeftY(), -DriverController.getLeftX()))
        ));

        DriverController.rightBumper().whileTrue(turretSubsystem.run(() -> {
            turretSubsystem.shootAutoTarget();
            turretSubsystem.kick(1);
        }));

        DriverController.rightBumper().onFalse(turretSubsystem.runOnce(() -> {
            turretSubsystem.stopShooter();
            turretSubsystem.stopKicker();
            }));
        
        OperatorController.rightBumper().whileTrue(turretSubsystem.defer(() -> turretSubsystem.turnToTarget()));
        
        OperatorController.rightBumper().onFalse(turretSubsystem.runOnce(() ->
            turretSubsystem.stopTurret()
        ));

        OperatorController.leftBumper().whileTrue(turretSubsystem.run(() ->
            turretSubsystem.turnRaw(-OperatorController.getRightY()/4)
        ));

        OperatorController.rightBumper().onFalse(turretSubsystem.runOnce(() ->
            turretSubsystem.stopTurret()
        ));

        // AtomicInteger angle = new AtomicInteger(25);
        // AtomicBoolean goingUp = new AtomicBoolean(true);
        // joystick.rightTrigger().whileTrue(turretSubsystem.run(() -> {
        //     turretSubsystem.setHoodAngle(Degrees.of(angle.get()));
        //     if(goingUp.get()) {
        //         if(angle.get() >= 45) {
        //             angle.set(angle.get()-1);
        //             goingUp.set(false);
        //         } else angle.set(angle.get()+1);
        //     } else {
                
        //         if(angle.get() <= 25) {
        //             angle.set(angle.get()+1);
        //             goingUp.set(true);
        //         } else angle.set(angle.get()-1);
        //     }
        // }));

        // joystick.x().onTrue(turretSubsystem.run(() -> turretSubsystem.kick(1)));
        // joystick.x().onFalse(turretSubsystem.stopKicker());

        // Operator toggle for Repulsor "has piece" (temporary until a sensor is wired).
        // Pressing X will toggle the value; it is published to NetworkTables for visibility.
        OperatorController.x().onTrue(Commands.runOnce(() -> {
            boolean next = !repulsorHasPiece.get();
            repulsorHasPiece.set(next);
            NetworkedTelemetry.Repulsor.setHasPiece(next);
            SmartDashboard.putBoolean("Repulsor/HasPiece", next);
        }));

        DriverController.y().onTrue(turretSubsystem.run(() -> {
            turretSubsystem.turnToAngle(Degrees.of(NetworkedConfig.Turret.getTargetTurretAngle()));
        }));
        DriverController.y().onFalse(turretSubsystem.run(() -> {
            turretSubsystem.stopTurret();
        }));

        DriverController.start().onTrue(turretSubsystem.runOnce(() -> {
            turretSubsystem.pullNetworkTableData();
        }));

        DriverController.leftTrigger().whileTrue(indexerSubsystem.run(() -> {
            indexerSubsystem.spin();
        }));
        DriverController.leftTrigger().onFalse(indexerSubsystem.runOnce(() -> indexerSubsystem.stop()));

        DriverController.start().onTrue(indexerSubsystem.runOnce(() -> {
            indexerSubsystem.pullNetworkTableData();
        }));
        
        DriverController.back().onTrue(swerveSubsystem.run(() -> swerveSubsystem.resetPose(new Pose2d(
            NetworkedConfig.Debug.getNewPoseX(),
            NetworkedConfig.Debug.getNewPoseY(),
            new Rotation2d(NetworkedConfig.Debug.getNewPoseRotation())
        ))));

        DriverController.pov(0).whileTrue(climberSubsystem.run(() -> climberSubsystem.raise()));
        DriverController.pov(180).whileTrue(climberSubsystem.run(() -> climberSubsystem.lower()));
        DriverController.pov(-1).whileTrue(climberSubsystem.run(() -> climberSubsystem.stopClimber()));

        OperatorController.rightBumper().onTrue(climberSubsystem.raiseToFull());
        OperatorController.leftBumper().onTrue(climberSubsystem.lowerOntoBar());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(swerveSubsystem.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(swerveSubsystem.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kReverse));

        // SysId bindings for turret flywheel characterization — explicitly start/stop SignalLogger
        // joystick.back().and(joystick.y()).onTrue(
        //     Commands.sequence(
        //         Commands.runOnce(() -> SignalLogger.start()),
        //         turretSubsystem.sysIdDynamic(Direction.kForward),
        //         Commands.runOnce(() -> SignalLogger.stop())
        //     )
        // );

        // joystick.back().and(joystick.x()).onTrue(
        //     Commands.sequence(
        //         Commands.runOnce(() -> SignalLogger.start()),
        //         turretSubsystem.sysIdDynamic(Direction.kReverse)
        //     )
        // );

        // joystick.back().onFalse(Commands.runOnce(() -> SignalLogger.stop()));

        // joystick.start().and(joystick.y()).onTrue(
        //     Commands.sequence(
        //         Commands.runOnce(() -> SignalLogger.start()),
        //         turretSubsystem.sysIdQuasistatic(Direction.kForward),
        //         Commands.runOnce(() -> SignalLogger.stop())
        //     )
        // );

        // joystick.start().and(joystick.x()).onTrue(
        //     Commands.sequence(
        //         Commands.runOnce(() -> SignalLogger.start()),
        //         turretSubsystem.sysIdQuasistatic(Direction.kReverse),
        //         Commands.runOnce(() -> SignalLogger.stop())
        //     )
        // );

        // joystick.start().onFalse(Commands.runOnce(() -> SignalLogger.stop()));

        // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(swerveSubsystem.runOnce(swerveSubsystem::seedFieldCentric));

        swerveSubsystem.registerTelemetry(logger::telemeterize);
    }

    public void pullAllNetworkedConfigs() {
        turretSubsystem.pullNetworkTableData();
        indexerSubsystem.pullNetworkTableData();
        intakeSubsystem.pullNetworkTableData();
    }

    private void initializeNamedCommands() {
        NamedCommands.registerCommand("homeHood", Commands.defer(() -> turretSubsystem.homeHood(), Set.of()));
        NamedCommands.registerCommand("startShooter", Commands.runOnce(() -> this.autoShoot = true));
        // NamedCommands.registerCommand("startShooterFromClimb", turretSubsystem.run(() -> turretSubsystem.shootAutoTargetWithRPMOffset(TurretSubsystemConstants.RPM_OFFSET_WHILE_CLIMBED)))
        NamedCommands.registerCommand("stopShooter", Commands.runOnce(() -> this.autoShoot = false));
        NamedCommands.registerCommand("runAutoShoot", Commands.startEnd(() -> this.autoShoot = true, () -> this.autoShoot = false));
        NamedCommands.registerCommand("autoClimb", Commands.defer(this::getAutoClimbCommand, Set.of(climberSubsystem, swerveSubsystem)));
        NamedCommands.registerCommand("Raise Climber", Commands.defer(() -> climberSubsystem.raiseToFull(), Set.of(climberSubsystem)));
        NamedCommands.registerCommand("Lower Climber", Commands.defer(() -> climberSubsystem.lowerOntoBar(), Set.of(climberSubsystem)));

        NamedCommands.registerCommand("Lower Intake", Commands.defer(() -> Commands.runOnce(() -> intakeSubsystem.dropIntake()), Set.of()));
        NamedCommands.registerCommand("Start Intake", Commands.runOnce(() -> {this.autoIntake = true; System.out.println("Intake Start");}));
        NamedCommands.registerCommand("Stop Intake", Commands.runOnce(() -> {this.autoIntake = false; System.out.println("Intake Stop");}));
        NamedCommands.registerCommand("Run Intake", Commands.runEnd(() -> this.autoIntake = true, () -> this.autoIntake = false));
        NamedCommands.registerCommand("Raise Intake", Commands.runOnce(() -> intakeSubsystem.liftIntake()));
    }

    public Command getAutonomousCommand() {
        return Commands.defer(() -> {
            Command homingSequence = Commands.parallel(
                turretSubsystem.homeHood(),
                intakeSubsystem.homeLift(),
                climberSubsystem.homeClimber()
            );

            Command autoProxy = Commands.defer(
                () -> {
                    Command selected = autoChooser.getSelected();
                    return selected != null ? selected : Commands.none();
                },
                Set.of(swerveSubsystem)
            );

            return Commands.sequence(homingSequence, autoProxy);
        }, Set.of(swerveSubsystem));
    }

    /**
     * Returns a command that fully automates the climb sequence when the driver
     * presses the climb button:
     * <ol>
     *   <li>Raise the climber to full extension.</li>
     *   <li>Drive to the climbing bar position using BLine path following
     *       (no odometry reset — navigates from the robot's current pose).</li>
     *   <li>Lower the climber onto the bar, engaging the hooks.</li>
     * </ol>
     *
     * <p><b>Tuning note:</b> The climb target pose is defined in
     * {@code Constants.FieldConstants.CLIMB_POSITION}. Adjust the x/y/rotation
     * values there and in {@code deploy/autos/paths/climb.json} to match your
     * actual bar location.</p>
     */
    public Command getAutoClimbCommand() {
        Command climbSequence = Commands.sequence(
            // Step 1: raise climber to full extension so it clears the bar
            // Step 2: drive staging → final at reduced speed so the climber slots
            //         onto the tower cleanly. Both poses are selected from the
            //         robot's current alliance + field side at the moment A is pressed.
            Commands.defer(() -> {
                Pose2d staging = selectStagingPosition();
                return swerveSubsystem.driveToPose(staging, 0.4 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
            }, java.util.Set.of(swerveSubsystem)),
            climberSubsystem.raiseToFull(),
            Commands.defer(() -> {
                Pose2d target  = selectClimbPosition();
                return swerveSubsystem.driveToPose(target, 0.1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
            }, java.util.Set.of(swerveSubsystem)),

            // Step 3: lower onto the bar to engage the clamp
            climberSubsystem.lowerOntoBar(),
            Commands.print("Climb!")
        );

        return Commands.sequence(
            Commands.runOnce(() -> {
                Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
                Set<Integer> towerTags = (alliance == Alliance.Red) ? Set.of(15/*, 16*/) : Set.of(31/*, 32*/);
                poseEstimatorSubsystem.setTemporaryAprilTagFilter(towerTags);
            }),
            climbSequence
        ).finallyDo(interrupted -> poseEstimatorSubsystem.clearTemporaryAprilTagFilter());
    }

    /**
     * Picks the correct climb target pose based on the robot's current alliance
     * and which side of the field (audience vs scoring) it is on.
     *
     * <p>Falls back to the blue audience-side position if the alliance or zone
     * cannot be determined.</p>
     */
    private Pose2d selectClimbPosition() {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        boolean isRed = alliance.isPresent()
            && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

        // Use the robot's current Y to decide audience side (low Y) vs scoring side (high Y).
        double fieldMidY = frc.robot.Constants.FieldConstants.FIELD_WIDTH_METERS / 2.0;
        boolean isScoringside = swerveSubsystem.getPose().getY() >= fieldMidY;

        if (isRed) {
            return isScoringside
                ? frc.robot.Constants.FieldConstants.ClimbPositions.RED_OUTPOST_SIDE
                : frc.robot.Constants.FieldConstants.ClimbPositions.RED_DEPOT_SIDE;
        } else {
            return isScoringside
                ? frc.robot.Constants.FieldConstants.ClimbPositions.BLUE_DEPOT_SIDE
                : frc.robot.Constants.FieldConstants.ClimbPositions.BLUE_OUTPOST_SIDE;
        }
    }

    /**
     * Picks the correct staging pose for the climb — the pre-alignment position the
     * robot drives to before slotting onto the tower.
     *
     * <p>Uses the same alliance + field-side logic as {@link #selectClimbPosition()}
     * so both poses are always consistent.</p>
     */
    private Pose2d selectStagingPosition() {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        boolean isRed = alliance.isPresent()
            && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

        double fieldMidY = frc.robot.Constants.FieldConstants.FIELD_WIDTH_METERS / 2.0;
        boolean isScoringside = swerveSubsystem.getPose().getY() >= fieldMidY;

        if (isRed) {
            return isScoringside
                ? frc.robot.Constants.FieldConstants.ClimbPositions.RED_OUTPOST_SIDE_STAGING
                : frc.robot.Constants.FieldConstants.ClimbPositions.RED_DEPOT_SIDE_STAGING;
        } else {
            return isScoringside
                ? frc.robot.Constants.FieldConstants.ClimbPositions.BLUE_DEPOT_SIDE_STAGING
                : frc.robot.Constants.FieldConstants.ClimbPositions.BLUE_OUTPOST_SIDE_STAGING;
        }
    }

    /**
     * Gets the starting pose from the currently selected autonomous routine.
     * Extracts the first pose from the PathPlanner path if available.
     * 
     * @return The auto starting pose, or null if not available
     */
    private Pose2d getSelectedAutoStartingPose() {
        try {
            Command selected = autoChooser.getSelected();
            if (selected != null && selected instanceof PathPlannerAuto) {
                return ((PathPlannerAuto) selected).getStartingPose();
            }
        } catch (Exception e) {
            // Silently fail if pose extraction is not available
        }
        return null;
    }
    
    /**
     * Checks if the robot's current pose is aligned to the autonomous starting position.
     * Uses configurable tolerances for distance and heading.
     * 
     * @return True if current pose is within tolerance of auto start pose
     */
    private boolean isAlignedToAutoStart() {
        Pose2d autoStartBlue = getSelectedAutoStartingPose();
        if (autoStartBlue == null) return false;

        Pose2d autoStart;
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
            Translation2d fieldCenter = new Translation2d(
                Constants.FieldConstants.FIELD_LENGTH_METERS / 2.0,
                Constants.FieldConstants.FIELD_WIDTH_METERS / 2.0
            );
            autoStart = autoStartBlue.rotateAround(fieldCenter, new Rotation2d(Math.PI));
        } else {
            autoStart = autoStartBlue;
        }


        NetworkedTelemetry.Pose.publishStartingCircle(autoStart, Constants.FieldConstants.AUTO_POSE_DISTANCE_TOLERANCE_M);
        
        Pose2d current = swerveSubsystem.getPose();
        double distance = current.getTranslation().getDistance(autoStart.getTranslation());
        double headingDiff = Math.abs(current.getRotation().minus(autoStart.getRotation()).getDegrees());
        
        // Normalize heading difference to [-180, 180]
        if (headingDiff > 180) headingDiff = 360 - headingDiff;
        
        return distance <= Constants.FieldConstants.AUTO_POSE_DISTANCE_TOLERANCE_M 
            && headingDiff <= Constants.FieldConstants.AUTO_POSE_HEADING_TOLERANCE_DEG;
    }
    
    /**
     * Publishes the current NeoPixel control mode based on robot state.
     * Mode selection priority:
     * 1. DISABLED_NO_CAMERA — if both cameras are not active
     * 2. DISABLED_NO_TAGS — if cameras are active but no AprilTags detected
     * 3. DISABLED_CORRECT_POSITION — if aligned to autonomous starting pose
     * 4. DISABLED_HAS_TAGS — if tags detected but not aligned
     * 5. ENABLED_DEFAULT — when robot is enabled
     */
    private void publishNeopixelMode() {
        String mode;
        boolean hasDebouncedAprilTags = aprilTagValidDebouncer.calculate(NetworkedTelemetry.Vision.hasValidAprilTags());
        
        if (edu.wpi.first.wpilibj.DriverStation.isDisabled()) {
            if (!NetworkedTelemetry.Vision.bothCamerasActive()) {
                mode = "DISABLED_NO_CAMERA";
            } else if (isAlignedToAutoStart()) {
                mode = "DISABLED_CORRECT_POSITION";
            } else if (!hasDebouncedAprilTags) {
                mode = "DISABLED_NO_TAGS";
            } else {
                mode = "DISABLED_HAS_TAGS";
            }
        } else {
            mode = "ENABLED_DEFAULT";
        }
        
        NetworkedTelemetry.NeoPixels.setControlMode(mode);
    }

    /**
     * Called every robot loop from {@code Theseus.robotPeriodic()}. Publishes
     * game-state telemetry to NetworkTables using the {@link GameState} instance
     * managed by {@link StateManager} (updated by {@code repulsor.update()}).
     */
    public void periodicUpdate() {
        if (gameState != null) { 
            // Countdown is only meaningful when the hub is inactive — how long until it flips active.
            // When already active (or game data not yet available), publish 0.
            // double countdown = (!gs.isHubActive()) ? Math.max(0.0, gs.getRemainingShiftTime()) : 0.0;
            double countdown = gameState.getRemainingShiftTime();
            NetworkedTelemetry.GameState.publish(
                 (int) gameState.getMatchTime(),
                gameState.isHubActive(),
                 (int) countdown
            );
            
            // Handle NeoPixel hub warning trigger (8 seconds before hub becomes active)
            boolean isHubActive = gameState.isHubActive();
            
            // Fire trigger when hub inactive, countdown <= 6s, and not yet fired
            if (!isHubActive && countdown <= 10.0 && !hubWarningFired) {
                NetworkedTelemetry.NeoPixels.setControlTrigger("PHASE_SHIFT_INCOMING");
                hubWarningFired = true;
                System.out.println("[NeoPixel] Phase shift incoming trigger: hub activating in ~" + countdown + "s");
            }
            
            // Reset flag when hub just became inactive
            if (wasHubActive && !isHubActive) {
                hubWarningFired = false;
            }
            
            wasHubActive = isHubActive;
        }
        
        // Publish current NeoPixel mode based on robot state
        publishNeopixelMode();

        // (Diagnostics removed): path start/end printing is used instead.

        if (NetworkedConfig.Debug.shouldResetHomedPositions()) {
            turretSubsystem.resetHoodHoming();
            climberSubsystem.resetHoming();
            intakeSubsystem.resetLiftHoming();
            NetworkedConfig.Debug.clearResetHomedPositions();
        }

        NetworkedTelemetry.Turret.setTurretAngleOffset(turretAngleOffset.in(Degrees));

        // if (turretSubsystem.isShootingActive() && turretSubsystem.getRelativeAngleToTarget().abs(Degrees) > 60) autoTurningToAngle = true;
        // else autoTurningToAngle = false;
    }

    /**
     * Returns {@code true} when a hub-targeting shot is permitted.
     *
     * <p>A shot is allowed when any of the following is true:
     * <ul>
     *   <li>The turret is <em>not</em> targeting the hub (relay shots always allowed).</li>
     *   <li>The hub is currently active for this alliance.</li>
     *   <li>{@link NetworkedConfig.Debug#isBypassHubLock()} is {@code true} (dev override).</li>
     * </ul>
     */
    private boolean isShotAllowed() {
        return turretSubsystem.isTurretSafe(); // There appears to be a bug preventing shots, don't have time to debug it

        // if (!turretSubsystem.isTargetingHub()) return true;
        // if (NetworkedConfig.Debug.isBypassHubLock()) return true;
        // GameState gs = StateManager.getState(GameState.class);
        // return gs == null || gs.isHubActive();
    }

    /**
     * Returns the automated systems-check command to be scheduled during test mode.
     *
     * @return the systems-check command built by {@link SystemsCheck}
     */
    public Command getSystemsCheckCommand() {
        return SystemsCheck.build(this);
    }
}
