// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.USE_VISION;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.VisionConstants;
import frc.robot.networking.NetworkedTelemetry;

/**
 * Manages all of the vision cameras on the robot.
 */
public class PoseEstimatorSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain swerveSubsystem;
  private final PhotonRunnable frontCamera;
  private final PhotonRunnable backCamera;

  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem(CommandSwerveDrivetrain swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    if (USE_VISION) {
      this.frontCamera = new PhotonRunnable(new PhotonCamera("FrontCam"), VisionConstants.ROBOT_TO_FRONT_CAM);
      this.backCamera = new PhotonRunnable(new PhotonCamera("BackCam"), VisionConstants.ROBOT_TO_BACK_CAM);
      this.setDefaultCommand(this.createNotifierCommand(this));
    } else {
      this.frontCamera = null;
      this.backCamera = null;
    }
  }

  @Override
  public void periodic() {
    if (VisionConstants.USE_VISION) {
      estimatorChecker(frontCamera);
      estimatorChecker(backCamera);
    }
  }

  private NotifierCommand createNotifierCommand(PoseEstimatorSubsystem peSubsystem) {
    return new NotifierCommand(() -> {
      frontCamera.run();
      backCamera.run();
    }, 0.02, peSubsystem);
  }

  public Pose2d getCurrentPose() {
    return swerveSubsystem.getState().Pose;
  }

  public void setCurrentPose(Pose2d newPose) {
    swerveSubsystem.resetPose(newPose);
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (PhotonTrackedTarget target : estimation.targetsUsed) {
      Transform3d t3d = target.getBestCameraToTarget();
      double distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance)
        smallestDistance = distance;
    }

    double poseAmbiguityFactor = estimation.targetsUsed.size() != 1 ? 1
        : Math.max(1, estimation.targetsUsed.get(0).getPoseAmbiguity() + VisionConstants.POSE_AMBIGUITY_SHIFTER
            * VisionConstants.POSE_AMBIGUITY_MULTIPLIER);

    double confidenceMultiplier = Math.max(
        1,
        (Math.max(
            1,
            Math.max(0, smallestDistance - VisionConstants.NOISY_DISTANCE_METERS)
                * VisionConstants.DISTANCE_WEIGHT)
            * poseAmbiguityFactor)
            / (1
                + ((estimation.targetsUsed.size() - 1)
                    * VisionConstants.TAG_PRESENCE_WEIGHT)));
    return VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
  }

  public void estimatorChecker(PhotonRunnable estimator) {
    EstimatedRobotPose cameraPose = estimator.grabLatestEstimatedPose();
    if (cameraPose == null) {
      NetworkedTelemetry.Vision.setHasValidAprilTags(false);
      return;
    }
    
    NetworkedTelemetry.Vision.setHasValidAprilTags(true);
    Pose2d pose2d = cameraPose.estimatedPose.toPose2d();
    if (RobotState.isDisabled()) {
      System.out.println("Setting position");
      swerveSubsystem.resetPose(pose2d);
    } else {
      swerveSubsystem.addVisionMeasurement(pose2d, cameraPose.timestampSeconds,
          confidenceCalculator(cameraPose));
    }
  }
}