// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.ctre.phoenix6.SignalLogger;

import frc.robot.generated.TunerConstants;
import frc.robot.libraries.Repulsor.Repulsor;
import frc.robot.libraries.Repulsor.DriverStation.RepulsorDriverStationBootstrap;
import frc.robot.adapters.SwerveRepulsorAdapter;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import frc.robot.subsystems.MotorTestingSubsystem;

import frc.robot.lib.BLine.*;
import frc.robot.networking.NetworkedConfig;

public class ShipOfTheseus {
    private double MaxSpeed = 0.25 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Field2d field = new Field2d();

    public final CommandSwerveDrivetrain swerveSubsystem = TunerConstants.createDrivetrain(field);
    // TODO: MotorTestingSubsystem is for local testing only — remove before merging to main
    public final MotorTestingSubsystem motorTestingSubsystem = new MotorTestingSubsystem();
    public final TurretSubsystem turretSubsystem;
    public final IndexerSubsystem indexerSubsystem;
    public final PoseEstimatorSubsystem poseEstimatorSubsystem;
    public final ClimberSubsystem climberSubsystem;
    public final Repulsor repulsor;

    FollowPath.Builder pathBuilder = new FollowPath.Builder(
        swerveSubsystem,
        swerveSubsystem::getPose,
        swerveSubsystem::getChassisSpeeds,
        swerveSubsystem::driveWithChassisSpeeds,
        new PIDController(5.0, 0.0, 0.0),
        new PIDController(3.0, 0.0, 0.0),
        new PIDController(2.0, 0.0, 0.0)
        ).withDefaultShouldFlip()
        .withPoseReset(swerveSubsystem::resetPose);

    public ShipOfTheseus() {

        this.turretSubsystem = new TurretSubsystem(swerveSubsystem);
        this.indexerSubsystem = new IndexerSubsystem();
        this.poseEstimatorSubsystem = new PoseEstimatorSubsystem(swerveSubsystem);
        this.climberSubsystem = new ClimberSubsystem();

        SmartDashboard.putData("Field", field);
        
        configureBindings();
        NetworkedConfig.initializeAllDefaults();

        this.repulsor =
            new Repulsor(
                new SwerveRepulsorAdapter(swerveSubsystem),
                frc.robot.Constants.RepulsorConstants.ROBOT_X,
                frc.robot.Constants.RepulsorConstants.ROBOT_Y,
                0.0,
                0.0,
                () -> false); // TODO: replace this with a real supplier to indicate if we have gamepieces
        RepulsorDriverStationBootstrap.useDefaultNt();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerveSubsystem.setDefaultCommand(
            // Drivetrain will execute this command periodically
            swerveSubsystem.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left) //TODO: re-enable rotation
            )
        );

        // Default command for testing motor — right joystick Y drives the TalonFX.
        // TODO: Remove this testing binding and the MotorTestingSubsystem before merging to main
        // motorTestingSubsystem.setDefaultCommand(motorTestingSubsystem.run(() -> motorTestingSubsystem.setPercent(joystick.getRightY())));
        joystick.pov(90).whileTrue(motorTestingSubsystem.run(() -> motorTestingSubsystem.setPercent(0.8)));
        joystick.pov(270).whileTrue(motorTestingSubsystem.run(() -> motorTestingSubsystem.setPercent(-0.8)));
        joystick.pov(-1).whileTrue(motorTestingSubsystem.run(() -> motorTestingSubsystem.setPercent(0)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerveSubsystem.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(swerveSubsystem.applyRequest(() -> brake));
        joystick.b().whileTrue(swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.rightBumper().whileTrue(turretSubsystem.run(() -> {
            turretSubsystem.shoot();
            turretSubsystem.kick(0.35);
        }));

        joystick.rightBumper().onFalse(turretSubsystem.runOnce(() -> {
            turretSubsystem.stopShooter();
            turretSubsystem.stopKicker();
            }));

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

        joystick.y().onTrue(turretSubsystem.run(() -> {
            turretSubsystem.turnToAngle(Degrees.of(NetworkedConfig.Turret.getTargetTurretAngle()));
        }));
        joystick.y().onFalse(turretSubsystem.run(() -> {
            turretSubsystem.stopTurret();
        }));

        joystick.start().onTrue(turretSubsystem.runOnce(() -> {
            turretSubsystem.pullNetworkTableData();
        }));

        joystick.leftTrigger().whileTrue(indexerSubsystem.run(() -> {
            indexerSubsystem.spin();
        }));
        joystick.leftTrigger().onFalse(indexerSubsystem.stop());

        joystick.start().onTrue(indexerSubsystem.runOnce(() -> {
            indexerSubsystem.pullNetworkTableData();
        }));
        
        joystick.back().onTrue(swerveSubsystem.run(() -> swerveSubsystem.resetPose(new Pose2d(
            NetworkedConfig.Debug.getNewPoseX(),
            NetworkedConfig.Debug.getNewPoseY(),
            new Rotation2d(NetworkedConfig.Debug.getNewPoseRotation())
        ))));

        joystick.pov(0).whileTrue(climberSubsystem.run(() -> climberSubsystem.climb()));
        joystick.pov(180).whileTrue(climberSubsystem.run(() -> climberSubsystem.lower()));
        joystick.pov(-1).whileTrue(climberSubsystem.run(() -> climberSubsystem.stopClimber()));

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

        // swerveSubsystem.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        Path testPath = new Path("Square Test");
        FollowPath.registerEventTrigger("testLog", new InstantCommand(() -> {
            System.out.println("YEET!");
        }));
        return Commands.sequence(
            pathBuilder.build(testPath)
        );
    }
}
