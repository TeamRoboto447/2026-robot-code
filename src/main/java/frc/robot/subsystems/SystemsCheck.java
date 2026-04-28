// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ShipOfTheseus;
import frc.robot.Constants.ClimberSubsystemConstants;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.networking.NetworkedConfig;
import frc.robot.networking.NetworkedTelemetry;
import frc.robot.utils.Elastic;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.NotificationLevel;
import java.util.function.BooleanSupplier;

/**
 * Builds the automated systems-check command that runs during test mode.
 *
 * <p>NetworkTables result entries are owned by
 * {@link NetworkedTelemetry.SystemsCheck}. This class is responsible only for
 * assembling the WPILib command sequence and wiring subsystem calls.
 *
 * <p>Each subsystem check is independently enabled/disabled from the
 * {@code SystemsCheck/config} NetworkTables subtable (see
 * {@link NetworkedConfig.SystemsCheck}) so a specific subsystem can be isolated
 * after maintenance without re-deploying code.
 *
 * <p>Results are published to {@code SystemsCheck/results} and shown on the
 * dedicated Elastic "Systems Check" tab. Any failure also fires an Elastic
 * ERROR notification. If {@code abort_on_failure} is {@code true} the sequence
 * stops at the first failing step.
 */
public class SystemsCheck {

    // Tunable constants

    /** Within ±FLYWHEEL_RPM_TOLERANCE RPM of target is considered "at speed". */
    private static final double FLYWHEEL_RPM_TOLERANCE = 150.0;

    /** Turret angles used for the motion check. */
    private static final double TURRET_FORWARD_TEST_ANGLE_DEG = 45.0;
    private static final double TURRET_REVERSE_TEST_ANGLE_DEG = -45.0;

    /** Hood angle used for the angle-control check (midpoint of usable range). */
    private static final double HOOD_TEST_ANGLE_DEG = 33.5;

    /** Seconds allowed for a position/speed target to be reached. */
    private static final double POSITION_TIMEOUT_S = 2.0;

    /** Seconds allowed for the flywheel to reach target speed. */
    private static final double FLYWHEEL_SPINUP_TIMEOUT_S = 5.0;

    /** How long to hold the intake roller on during its check. */
    private static final double INTAKE_ROLLER_RUN_S = 1.0;

    /** How long to run the indexer during its check. */
    private static final double INDEXER_RUN_S = 1.0;

    /** How long to run the feeder/kicker during its check. */
    private static final double FEEDER_RUN_S = 3.0;

    /** How long to drive in each swerve direction during its check. */
    private static final double SWERVE_MOTION_RUN_S = 2.0;

    // Per-run abort flag

    /**
     * Reset to {@code false} each time {@link #build} is called so successive
     * test-mode activations start clean.
     */
    private static boolean sequenceAborted = false;

    // Public API

    /**
     * Builds and returns the full systems-check {@link Command}.
     *
     * @param robot the {@link ShipOfTheseus} robot container providing subsystem access
     * @return the systems-check command (non-null, safe to schedule)
     */
    public static Command build(ShipOfTheseus robot) {
        // Snapshot the enable flags at the moment build() is called (i.e. when test
        // mode starts).  These same flags are used to selectively reset only the
        // entries that will actually be exercised this run, and to evaluate only
        // those entries in the final pass/fail summary.
        final boolean chkHood     = NetworkedConfig.SystemsCheck.isCheckHood();
        final boolean chkTurret   = NetworkedConfig.SystemsCheck.isCheckTurret();
        final boolean chkFlywheel = NetworkedConfig.SystemsCheck.isCheckFlywheel();
        final boolean chkIntake   = NetworkedConfig.SystemsCheck.isCheckIntake();
        final boolean chkIndexer  = NetworkedConfig.SystemsCheck.isCheckIndexer();
        final boolean chkFeeder   = NetworkedConfig.SystemsCheck.isCheckFeeder();
        final boolean chkClimber  = NetworkedConfig.SystemsCheck.isCheckClimber();
        final boolean chkSwerve   = NetworkedConfig.SystemsCheck.isCheckSwerve();

        return Commands.sequence(
            // Reset NT state only for systems being checked this run, then switch tab.
            // Also put the turret into systems-check mode so periodic() does not
            // override hood/turret positions with trajectory-tracking values.
            Commands.runOnce(() -> {
                sequenceAborted = false;
                robot.turretSubsystem.setSystemsCheckMode(true);
                NetworkedTelemetry.SystemsCheck.resetResults(
                    chkHood, chkTurret, chkFlywheel,
                    chkIntake, chkIndexer, chkFeeder, chkClimber, chkSwerve);
                Elastic.sendNotification(new Notification(
                    NotificationLevel.INFO,
                    "Systems Check",
                    "Systems check started.",
                    4000));
                Elastic.selectTab("Systems Check");
            }),

            // Hood
            step(robot, NetworkedConfig.SystemsCheck::isCheckHood,
                "Hood: Homing",
                robot.turretSubsystem.homeHood()
                    .withTimeout(4.0),
                NetworkedTelemetry.SystemsCheck.resHoodHoming),

            step(robot, NetworkedConfig.SystemsCheck::isCheckHood,
                "Hood: Angle Control",
                Commands.sequence(
                    robot.turretSubsystem.run(
                        () -> robot.turretSubsystem.setHoodAngle(Degrees.of(HOOD_TEST_ANGLE_DEG)))
                        .until(() -> hoodAtTarget(robot, HOOD_TEST_ANGLE_DEG))
                        .withTimeout(POSITION_TIMEOUT_S),
                    robot.turretSubsystem.run(
                        () -> robot.turretSubsystem.setHoodAngle(TurretSubsystemConstants.MIN_HOOD_ANGLE))
                        .until(() -> hoodAtTarget(robot, TurretSubsystemConstants.MIN_HOOD_ANGLE.magnitude()))
                        .withTimeout(POSITION_TIMEOUT_S)),
                NetworkedTelemetry.SystemsCheck.resHoodAngle),

            // Turret Rotation
            step(robot, NetworkedConfig.SystemsCheck::isCheckTurret,
                "Turret: Move Forward",
                robot.turretSubsystem.run(
                    () -> robot.turretSubsystem.turnToAngle(Degrees.of(TURRET_FORWARD_TEST_ANGLE_DEG)))
                    .until(() -> turretAtTarget(robot, TURRET_FORWARD_TEST_ANGLE_DEG))
                    .withTimeout(POSITION_TIMEOUT_S),
                NetworkedTelemetry.SystemsCheck.resTurretFwd),

            
            step(robot, NetworkedConfig.SystemsCheck::isCheckTurret,
                "Turret: Move Reverse",
                robot.turretSubsystem.run(
                    () -> robot.turretSubsystem.turnToAngle(Degrees.of(TURRET_REVERSE_TEST_ANGLE_DEG)))
                    .until(() -> turretAtTarget(robot, TURRET_REVERSE_TEST_ANGLE_DEG))
                    .withTimeout(POSITION_TIMEOUT_S),
                NetworkedTelemetry.SystemsCheck.resTurretRev),

            step(robot, NetworkedConfig.SystemsCheck::isCheckTurret,
                "Turret: Return to Zero",
                robot.turretSubsystem.run(
                    () -> robot.turretSubsystem.turnToAngle(Degrees.of(0.0)))
                    .until(() -> turretAtTarget(robot, 0.0))
                    .withTimeout(POSITION_TIMEOUT_S),
                NetworkedTelemetry.SystemsCheck.resTurretReturn),

            // Flywheel (4 speed targets)
            flywheelStep(robot, 2000, NetworkedTelemetry.SystemsCheck.resFlywheel2k),
            flywheelStep(robot, 3000, NetworkedTelemetry.SystemsCheck.resFlywheel3k),
            flywheelStep(robot, 4000, NetworkedTelemetry.SystemsCheck.resFlywheel4k),
            // flywheelStep(robot, 5000, NetworkedTelemetry.SystemsCheck.resFlywheel5k),

            // Ensure flywheel is stopped before continuing
            robot.turretSubsystem.runOnce(() -> robot.turretSubsystem.stopShooter()),

            // Intake
            step(robot, NetworkedConfig.SystemsCheck::isCheckIntake,
                "Intake: Homing",
                robot.intakeSubsystem.homeLift()
                    .withTimeout(4.0),
                NetworkedTelemetry.SystemsCheck.resIntakeHoming),

            step(robot, NetworkedConfig.SystemsCheck::isCheckIntake,
                "Intake: Drop",
                robot.intakeSubsystem.run(() -> robot.intakeSubsystem.dropIntake())
                    .until(() -> robot.intakeSubsystem.isIntakeDown())
                    .withTimeout(POSITION_TIMEOUT_S),
                NetworkedTelemetry.SystemsCheck.resIntakeDrop),

            step(robot, NetworkedConfig.SystemsCheck::isCheckIntake,
                "Intake: Roller",
                robot.intakeSubsystem.run(() -> robot.intakeSubsystem.intake(1))
                    .withTimeout(INTAKE_ROLLER_RUN_S)
                    .andThen(robot.intakeSubsystem.runOnce(() -> robot.intakeSubsystem.stopIntake())),
                NetworkedTelemetry.SystemsCheck.resIntakeRoller),

            step(robot, NetworkedConfig.SystemsCheck::isCheckIntake,
                "Intake: Lift",
                robot.intakeSubsystem.run(() -> robot.intakeSubsystem.liftIntake())
                    .until(() -> robot.intakeSubsystem.isIntakeUp())
                    .withTimeout(POSITION_TIMEOUT_S),
                NetworkedTelemetry.SystemsCheck.resIntakeLift),

            // Indexer
            step(robot, NetworkedConfig.SystemsCheck::isCheckIndexer,
                "Indexer: Spin",
                robot.indexerSubsystem.run(() -> robot.indexerSubsystem.spin())
                    .withTimeout(INDEXER_RUN_S)
                    .andThen(robot.indexerSubsystem.runOnce(() -> robot.indexerSubsystem.stop())),
                NetworkedTelemetry.SystemsCheck.resIndexer),

            // Feeder
            step(robot, NetworkedConfig.SystemsCheck::isCheckFeeder,
                "Feeder: Run",
                robot.turretSubsystem.run(() -> robot.turretSubsystem.runKickerRaw(0.5))
                    .withTimeout(FEEDER_RUN_S)
                    .andThen(robot.turretSubsystem.runOnce(() -> robot.turretSubsystem.stopKicker())),
                NetworkedTelemetry.SystemsCheck.resFeeder),

            // Climber
            step(robot, NetworkedConfig.SystemsCheck::isCheckClimber,
                "Climber: Homing",
                robot.climberSubsystem.homeClimber()
                    .withTimeout(4.0),
                NetworkedTelemetry.SystemsCheck.resClimberHome),

            step(robot, NetworkedConfig.SystemsCheck::isCheckClimber,
                "Climber: Full Extension",
                Commands.sequence(
                    robot.climberSubsystem.run(() -> robot.climberSubsystem.raise())
                        .until(() -> robot.climberSubsystem.getPositionRotations()
                                    >= ClimberSubsystemConstants.CLIMBER_FULL_EXTENSION_ROTATIONS
                                       - ClimberSubsystemConstants.CLIMBER_HOLD_TOLERANCE_ROTATIONS)
                        .withTimeout(20.0),
                    robot.climberSubsystem.runOnce(() -> robot.climberSubsystem.stopClimber())),
                NetworkedTelemetry.SystemsCheck.resClimberExt),

            step(robot, NetworkedConfig.SystemsCheck::isCheckClimber,
                "Climber: Retract",
                Commands.sequence(
                    robot.climberSubsystem.run(() -> robot.climberSubsystem.lower())
                        .until(() -> robot.climberSubsystem.getPositionRotations()
                                    <= ClimberSubsystemConstants.CLIMBER_HOLD_TOLERANCE_ROTATIONS)
                        .withTimeout(20.0),
                    robot.climberSubsystem.runOnce(() -> robot.climberSubsystem.stopClimber())),
                NetworkedTelemetry.SystemsCheck.resClimberRet),

            // Swerve
            // 1. Hardware fault check (no movement)
            step(robot, NetworkedConfig.SystemsCheck::isCheckSwerve,
                "Swerve: No Faults",
                Commands.sequence(
                    robot.swerveSubsystem.runOnce(() -> { /* trigger subsystem requirement */ }),
                    Commands.either(
                        Commands.none(),
                        Commands.waitUntil(() -> false), // blocks → timeout → interrupted=true → failure
                        () -> {
                            for (int i = 0; i < 4; i++) {
                                var mod = robot.swerveSubsystem.getModule(i);
                                if (mod.getDriveMotor().getFault_Hardware().getValue()
                                 || mod.getSteerMotor().getFault_Hardware().getValue()) {
                                    return false;
                                }
                            }
                            return true;
                        })),
                NetworkedTelemetry.SystemsCheck.resSwerveNoFaults),

            // 2–7. Cardinal translations + rotations (~0.4 m/s or 0.75 rad/s for 1 s)
            swerveMotionStep(robot, "Swerve: Forward",
                new ChassisSpeeds(0.4, 0.0, 0.0),
                NetworkedTelemetry.SystemsCheck.resSwerveForward),

            swerveMotionStep(robot, "Swerve: Backward",
                new ChassisSpeeds(-0.4, 0.0, 0.0),
                NetworkedTelemetry.SystemsCheck.resSwerveBackward),

            swerveMotionStep(robot, "Swerve: Left",
                new ChassisSpeeds(0.0, 0.4, 0.0),
                NetworkedTelemetry.SystemsCheck.resSwerveLeft),

            swerveMotionStep(robot, "Swerve: Right",
                new ChassisSpeeds(0.0, -0.4, 0.0),
                NetworkedTelemetry.SystemsCheck.resSwerveRight),

            swerveMotionStep(robot, "Swerve: Rotate CW",
                new ChassisSpeeds(0.0, 0.0, -0.75),
                NetworkedTelemetry.SystemsCheck.resSwerveRotateCW),

            swerveMotionStep(robot, "Swerve: Rotate CCW",
                new ChassisSpeeds(0.0, 0.0, 0.75),
                NetworkedTelemetry.SystemsCheck.resSwerveRotateCCW),

            // Final summary
            Commands.runOnce(() -> publishFinalResult(
                chkHood, chkTurret, chkFlywheel,
                chkIntake, chkIndexer, chkFeeder, chkClimber, chkSwerve))
        ).finallyDo(() -> robot.turretSubsystem.setSystemsCheckMode(false));
    }

    // Private helpers

    /**
     * Wraps a single check command with pass/fail NT recording, an Elastic ERROR
     * notification on failure, the enable toggle, and optional abort-on-failure.
     */
    private static Command step(
            ShipOfTheseus robot,
            BooleanSupplier enabled,
            String name,
            Command inner,
            BooleanEntry resultEntry) {

        Command tracked = inner.finallyDo(interrupted -> {
            boolean passed = !interrupted;
            resultEntry.set(passed);
            if (!passed) {
                if (!sequenceAborted) {
                    Elastic.sendNotification(new Notification(
                        NotificationLevel.ERROR,
                        "Check Failed: " + name,
                        "Step did not complete within the allowed time."));
                }
                if (NetworkedConfig.SystemsCheck.isAbortOnFailure()) {
                    sequenceAborted = true;
                }
            }
        });

        Command withAbortGuard = Commands.either(
            Commands.runOnce(() -> {}).ignoringDisable(true), // instant no-op when aborting
            tracked,
            () -> sequenceAborted
        );

        return Commands.either(
            withAbortGuard,
            Commands.none(), // skipped — leave result entry at default false
            enabled
        );
    }

    /**
     * Builds a flywheel speed check step — overrides the target RPM, spins up,
     * verifies the flywheel holds within tolerance, then spins down.
     */
    private static Command flywheelStep(ShipOfTheseus robot, double targetRPM, BooleanEntry resultEntry) {
        String name = "Flywheel: " + (int) targetRPM + " RPM";

        Command inner = Commands.sequence(
            // Spin up to the requested RPM directly — no NT mutation needed.
            robot.turretSubsystem.run(() -> robot.turretSubsystem.spinFlywheelAtRPM(targetRPM))
                .until(() -> Math.abs(robot.turretSubsystem.getFlywheelRPM() - targetRPM)
                            < FLYWHEEL_RPM_TOLERANCE)
                .withTimeout(FLYWHEEL_SPINUP_TIMEOUT_S),
            // Hold at speed briefly to confirm it's stable.
            robot.turretSubsystem.run(() -> robot.turretSubsystem.spinFlywheelAtRPM(targetRPM))
                .withTimeout(0.75),
            Commands.waitUntil(() -> robot.turretSubsystem.getFlywheelRPM() < targetRPM * 0.3)
                .withTimeout(5.0)
        ).finallyDo((interrupted -> robot.turretSubsystem.stopShooter()));

        return step(robot, NetworkedConfig.SystemsCheck::isCheckFlywheel, name, inner, resultEntry);
    }

    /**
     * Drives the robot at {@code target} for {@link #SWERVE_MOTION_RUN_S} seconds,
     * verifies the measured chassis speed exceeds a minimum threshold in the expected
     * axis, then commands a full stop. Wrapped in the swerve enable toggle.
     *
     * <p><b>Suspended-robot note:</b> for translation steps the check reads
     * {@code getChassisSpeeds().vx/vy}, which is kinematics-derived from drive-motor
     * encoders and is valid even with no ground contact. For rotation steps the gyro
     * will not turn (the chassis isn't rotating), so {@code omegaRadiansPerSecond}
     * would always read ~0 and cannot be used. Instead, rotation is verified by
     * checking that all four drive motors are producing non-trivial wheel speeds via
     * {@code getModule(i).getCurrentState().speedMetersPerSecond}, which is also
     * encoder-only and is the correct observable for "steer motors have angled the
     * wheels and drive motors are spinning" without needing the gyro.
     */
    private static Command swerveMotionStep(
            ShipOfTheseus robot,
            String name,
            ChassisSpeeds target,
            BooleanEntry resultEntry) {

        final double MIN_TRANSLATION_SPEED_MPS = 0.1;
        final double MIN_MODULE_SPEED_MPS      = 0.1;

        boolean isRotation = (target.vxMetersPerSecond == 0.0 && target.vyMetersPerSecond == 0.0);

        // Shared flag: set to true by the verify step if motion is confirmed.
        // finallyDo on the outer step() wrapper cannot see this flag directly, so we
        // use Commands.either() to turn a false flag into an indefinite waitUntil that
        // the outer timeout will cancel — producing interrupted=true → failure, without
        // any exception being thrown.
        boolean[] motionConfirmed = {false};

        Command inner = Commands.sequence(
            // Drive for SWERVE_MOTION_RUN_S seconds
            robot.swerveSubsystem.run(() -> robot.swerveSubsystem.driveWithChassisSpeeds(target))
                .withTimeout(SWERVE_MOTION_RUN_S),
            // Sample speeds and set the flag — no throw
            robot.swerveSubsystem.runOnce(() -> {
                if (isRotation) {
                    boolean allSpinning = true;
                    for (int i = 0; i < 4; i++) {
                        double speed = Math.abs(
                            robot.swerveSubsystem.getModule(i).getCurrentState().speedMetersPerSecond);
                        if (speed < MIN_MODULE_SPEED_MPS) {
                            allSpinning = false;
                            break;
                        }
                    }
                    motionConfirmed[0] = allSpinning;
                } else {
                    ChassisSpeeds measured = robot.swerveSubsystem.getChassisSpeeds();
                    double mag = Math.abs(target.vxMetersPerSecond) > 0
                        ? Math.abs(measured.vxMetersPerSecond)
                        : Math.abs(measured.vyMetersPerSecond);
                    motionConfirmed[0] = mag >= MIN_TRANSLATION_SPEED_MPS;
                }
            }),
            // Stop the drivetrain
            robot.swerveSubsystem.runOnce(
                () -> robot.swerveSubsystem.driveWithChassisSpeeds(new ChassisSpeeds())),
            // If motion was not confirmed, block forever so the outer step() timeout
            // triggers interrupted=true → failure recorded cleanly via finallyDo
            Commands.either(
                Commands.none(),
                Commands.waitUntil(() -> false),
                () -> motionConfirmed[0])
        );

        return step(robot, NetworkedConfig.SystemsCheck::isCheckSwerve, name, inner, resultEntry);
    }

    // Inline condition helpers (avoids multi-line lambdas in .until())

    private static boolean hoodAtTarget(ShipOfTheseus robot, double targetDeg) {
        return Math.abs(robot.turretSubsystem.getHoodAngleDegrees() - targetDeg)
            < TurretSubsystemConstants.HOOD_ANGLE_TOLERANCE_DEGREES;
    }

    private static boolean turretAtTarget(ShipOfTheseus robot, double targetDeg) {
        return Math.abs(robot.turretSubsystem.getTurretAngleDegrees() - targetDeg)
            < TurretSubsystemConstants.TURRET_ANGLE_TOLERANCE_DEGREES;
    }

    // Final result

    private static void publishFinalResult(
            boolean hood, boolean turret, boolean flywheel,
            boolean intake, boolean indexer, boolean feeder, boolean climber, boolean swerve) {
        boolean allPassed = NetworkedTelemetry.SystemsCheck.computeOverall(
            hood, turret, flywheel, intake, indexer, feeder, climber, swerve);
        if (allPassed) {
            Elastic.sendNotification(new Notification(
                NotificationLevel.INFO,
                "Systems Check Passed",
                "All enabled subsystem checks completed successfully."));
        } else {
            Elastic.sendNotification(new Notification(
                NotificationLevel.ERROR,
                "Systems Check Failed",
                "One or more subsystem checks failed. Review the Systems Check tab.")
                .withNoAutoDismiss());
        }
    }
}
