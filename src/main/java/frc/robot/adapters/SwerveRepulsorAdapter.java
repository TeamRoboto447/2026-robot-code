package frc.robot.adapters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.libraries.Repulsor.DriveRepulsor;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Minimal adapter that exposes our existing swerve drivetrain to Repulsor's
 * DriveRepulsor API.
 */
public final class SwerveRepulsorAdapter extends DriveRepulsor {
  private final CommandSwerveDrivetrain drivetrain;
  private final PIDController omegaPid = new PIDController(4.0, 0.0, 0.1);

  public SwerveRepulsorAdapter(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void runVelocity(ChassisSpeeds speeds) {
    drivetrain.driveWithChassisSpeeds(speeds);
  }

  @Override
  public Pose2d getPose() {
    return drivetrain.getPose();
  }

  @Override
  public PIDController getOmegaPID() {
    return omegaPid;
  }
}
