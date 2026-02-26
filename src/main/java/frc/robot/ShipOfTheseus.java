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
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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
 
import frc.robot.generated.TunerConstants;
import frc.robot.libraries.Repulsor.Repulsor;
import frc.robot.libraries.Repulsor.DriverStation.RepulsorDriverStationBootstrap;
import frc.robot.libraries.Repulsor.State.GameState;
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
    private double MaxSpeed = 0.25 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private boolean autoShoot = false;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveFieldOriented = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotOriented = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController DriverController = new CommandXboxController(0);
    private final CommandXboxController OperatorController = new CommandXboxController(1);

    private final Field2d field = new Field2d();
    private final SendableChooser<Command> autoChooser;

    public final CommandSwerveDrivetrain swerveSubsystem = TunerConstants.createDrivetrain(field);
    public final TurretSubsystem turretSubsystem;
    public final IndexerSubsystem indexerSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final PoseEstimatorSubsystem poseEstimatorSubsystem;
    public final ClimberSubsystem climberSubsystem;
    public final Repulsor repulsor;
    private final AtomicBoolean repulsorHasPiece = new AtomicBoolean(false);
    

    private final Trigger driverControllerPOVActive = new Trigger(() -> !DriverController.povCenter().getAsBoolean());
    private final Trigger operatorControllerRightJoystick = new Trigger(() -> 
            (Math.abs(OperatorController.getRightX()) > 0.1) ||
            (Math.abs(OperatorController.getRightY()) > 0.1)
    );
    private final Trigger autoShootTrigger = new Trigger(() -> this.autoShoot);

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

        this.turretSubsystem = new TurretSubsystem(swerveSubsystem);
        this.intakeSubsystem = new IntakeSubsystem();
        this.indexerSubsystem = new IndexerSubsystem();
        this.poseEstimatorSubsystem = new PoseEstimatorSubsystem(swerveSubsystem);
        this.climberSubsystem = new ClimberSubsystem(swerveSubsystem);


        SmartDashboard.putData("Field", field);

        initializedNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
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
    }

    public void runSensorlessHoming() {
        CommandScheduler.getInstance().schedule(climberSubsystem.homeClimber());
        CommandScheduler.getInstance().schedule(turretSubsystem.homeHood());
        CommandScheduler.getInstance().schedule(intakeSubsystem.homeLift());
    }

    // TODO: Verify correct bindings before uploading
    private void configureBindings() {
        configureAutonomousBindings();
        
        configureProductionBindings();
        // configureDevBindings();
    }

    private void configureAutonomousBindings() {
        autoShootTrigger.and(turretSubsystem::hasValidTarget)
            .and(turretSubsystem::isHoodHomed)
            .and(this::isShotAllowed)
            .whileTrue(Commands.parallel(
                Commands.run(() -> indexerSubsystem.spin()),
                Commands.run(() -> turretSubsystem.kick(0.35)),
                Commands.run(() -> turretSubsystem.shoot())
            ));
        autoShootTrigger.onFalse(Commands.runOnce(() -> {
            turretSubsystem.stopAll();
            indexerSubsystem.stop();
        }));
    }

    private void configureProductionBindings() {
        // Run homing commands on initialization - If already homed, the command immediately cancels itself
        // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> runSensorlessHoming()));
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            runSensorlessHoming();
            this.autoShoot = false;
        }));

        climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stopClimber()));
        DriverController.x().onTrue(Commands.defer(() -> getAutoClimbCommand(), Set.of(swerveSubsystem, climberSubsystem)));

        // Swerve Drive
        swerveSubsystem.setDefaultCommand(
            swerveSubsystem.applyRequest(() ->
                driveFieldOriented.withVelocityX(-DriverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-DriverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-DriverController.getRightX() * MaxAngularRate)
            )
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
        DriverController.rightBumper().onTrue(climberSubsystem.raiseToFull().onlyIf(climberSubsystem.withinSafeClimberRange));
        DriverController.y().whileTrue(climberSubsystem.run(() -> climberSubsystem.raise()));

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
        DriverController.rightTrigger().or(OperatorController.rightTrigger())
            .and(turretSubsystem::hasValidTarget)
            .and(this::isShotAllowed)
            .whileTrue(
                turretSubsystem.run(() -> turretSubsystem.shoot())
            );

        turretSubsystem.getFeedTrigger().and(
            DriverController.rightTrigger().or(OperatorController.rightTrigger()))
            .and(turretSubsystem::hasValidTarget)
            .and(turretSubsystem::isHoodHomed)
            .and(this::isShotAllowed)
            .whileTrue(Commands.parallel(
                indexerSubsystem.run(() -> indexerSubsystem.spin()),
                turretSubsystem.run(() -> turretSubsystem.kick(0.35))
            ));

        DriverController.rightTrigger().onFalse(turretSubsystem.runOnce(() -> {
            turretSubsystem.stopShooter();
            turretSubsystem.stopKicker();
        }));

        DriverController.rightTrigger().onFalse(indexerSubsystem.runOnce(() -> indexerSubsystem.stop()));

        // Driver: Intake (left trigger)
        // Lower the intake if it isn't already, then run the intake roller.
        DriverController.leftTrigger().onTrue(
            intakeSubsystem.runOnce(() -> {
                if (!intakeSubsystem.isIntakeDown()) intakeSubsystem.dropIntake();
            })
        );
        DriverController.leftTrigger().whileTrue(
            intakeSubsystem.run(() -> intakeSubsystem.intake(0.5))
        );
        DriverController.leftTrigger().onFalse(
            intakeSubsystem.runOnce(() -> intakeSubsystem.stopIntake())
        );

        // turretSubsystem.getFeedTrigger().and(OperatorController.rightTrigger())
        //     .and(turretSubsystem::hasValidTarget)
        //     .and(this::isShotAllowed)
        //     .whileTrue(Commands.parallel(
        //         indexerSubsystem.run(() -> indexerSubsystem.spin()),
        //         turretSubsystem.run(() -> turretSubsystem.kick(0.35))
        //     ));
        // OperatorController.rightTrigger().onFalse(turretSubsystem.runOnce(() -> {
        //     turretSubsystem.stopShooter();
        //     turretSubsystem.stopKicker();
        // }));
        // OperatorController.rightTrigger().onFalse(indexerSubsystem.stop());

        // Operator: Intake (left trigger)
        OperatorController.a().onTrue(
            intakeSubsystem.runOnce(() -> {
                if (!intakeSubsystem.isIntakeDown()) intakeSubsystem.dropIntake();
            })
        );
        OperatorController.a().whileTrue(
            intakeSubsystem.run(() -> intakeSubsystem.intake(0.5))
        );
        OperatorController.a().onFalse(
            intakeSubsystem.runOnce(() -> intakeSubsystem.stopIntake())
        );

        // Operator: Intake lift
        OperatorController.leftBumper().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.liftIntake()));
        OperatorController.rightBumper().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.dropIntake()));

        // Operator: Climber Control
        OperatorController.povUp().onTrue(climberSubsystem.raiseToFull().onlyIf(climberSubsystem.withinSafeClimberRange));
        OperatorController.povDown().onTrue(climberSubsystem.lowerOntoBar());
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

        OperatorController.pov(90).whileTrue(intakeSubsystem.run(() -> intakeSubsystem.intake(0.5)));
        OperatorController.pov(270).whileTrue(intakeSubsystem.run(() -> intakeSubsystem.reverseIntake(1)));
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
            turretSubsystem.shoot();
            turretSubsystem.kick(0.35);
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

    private void initializedNamedCommands() {
        NamedCommands.registerCommand("homeHood", turretSubsystem.homeHood());
        NamedCommands.registerCommand("startShooter", Commands.runOnce(() -> this.autoShoot = true));
        NamedCommands.registerCommand("stopShooter", Commands.runOnce(() -> this.autoShoot = false));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
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
        return Commands.sequence(
            // Step 1: raise climber to full extension so it clears the bar
            // climberSubsystem.raiseToFull(),
            // Step 2: drive staging → final at reduced speed so the climber slots
            //         onto the tower cleanly. Both poses are selected from the
            //         robot's current alliance + field side at the moment A is pressed.
            Commands.defer(() -> {
                Pose2d staging = selectStagingPosition();
                return swerveSubsystem.driveToPose(staging);
            }, java.util.Set.of(swerveSubsystem)),
            Commands.defer(() -> {
                Pose2d target  = selectClimbPosition();
                return swerveSubsystem.driveToPose(target);
            }, java.util.Set.of(swerveSubsystem)),
            
            // Step 3: lower onto the bar to engage the clamp
            // climberSubsystem.lowerOntoBar(),
            Commands.print("Climb!")
        );
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
                ? frc.robot.Constants.FieldConstants.ClimbPositions.RED_SCORING_SIDE
                : frc.robot.Constants.FieldConstants.ClimbPositions.RED_AUDIENCE_SIDE;
        } else {
            return isScoringside
                ? frc.robot.Constants.FieldConstants.ClimbPositions.BLUE_SCORING_SIDE
                : frc.robot.Constants.FieldConstants.ClimbPositions.BLUE_AUDIENCE_SIDE;
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
                ? frc.robot.Constants.FieldConstants.ClimbPositions.RED_SCORING_SIDE_STAGING
                : frc.robot.Constants.FieldConstants.ClimbPositions.RED_AUDIENCE_SIDE_STAGING;
        } else {
            return isScoringside
                ? frc.robot.Constants.FieldConstants.ClimbPositions.BLUE_SCORING_SIDE_STAGING
                : frc.robot.Constants.FieldConstants.ClimbPositions.BLUE_AUDIENCE_SIDE_STAGING;
        }
    }

    /**
     * Called every robot loop from {@code Theseus.robotPeriodic()}. Publishes
     * game-state telemetry to NetworkTables using the {@link GameState} instance
     * managed by {@link StateManager} (updated by {@code repulsor.update()}).
     */
    public void periodicUpdate() {
        GameState gs = StateManager.getState(GameState.class);
        if (gs != null) {
            NetworkedTelemetry.GameState.publish(
                edu.wpi.first.wpilibj.DriverStation.getMatchTime(),
                gs.isHubActive()
            );
        }

        // (Diagnostics removed): path start/end printing is used instead.

        if (NetworkedConfig.Debug.shouldResetHomedPositions()) {
            turretSubsystem.resetHoodHoming();
            climberSubsystem.resetHoming();
            intakeSubsystem.resetLiftHoming();
            NetworkedConfig.Debug.clearResetHomedPositions();
        }
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
        if (!turretSubsystem.isTargetingHub()) return true;
        if (NetworkedConfig.Debug.isBypassHubLock()) return true;
        GameState gs = StateManager.getState(GameState.class);
        return gs == null || gs.isHubActive();
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
