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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
            this.frontCamera = new PhotonRunnable(new PhotonCamera("FrontCam"), VisionConstants.ROBOT_TO_BACK_LEFT_CAM);
            this.backCamera = new PhotonRunnable(new PhotonCamera("BackCam"), VisionConstants.ROBOT_TO_BACK_RIGHT_CAM);
            this.setDefaultCommand(this.createNotifierCommand(this));
        } else {
            this.frontCamera = null;
            this.backCamera = null;
        }
    }

    /**
     * Updates the subsystem's various elements.
     */
    @Override
    public void periodic() {
        if (VisionConstants.USE_VISION) {
            estimatorChecker(frontCamera);
            estimatorChecker(backCamera);
        }
    }

    /**
     * Creates a new notifier command that continues running while the robot is disabled,
     * ensuring vision pose estimation remains active in all robot states.
     */
    private Command createNotifierCommand(PoseEstimatorSubsystem peSubsystem) {
        return new NotifierCommand(() -> {
            frontCamera.run();
            backCamera.run();
        }, 0.02, peSubsystem).ignoringDisable(true);
    }

    /**
     * Gets the robot's current pose.
     * 
     * @return The current robot pose
     */
    public Pose2d getCurrentPose() {
        return swerveSubsystem.getState().Pose;
    }

    /**
     * Sets a new robot pose.
     * 
     * @param newPose The pose to set
     */
    public void setCurrentPose(Pose2d newPose) {
        swerveSubsystem.resetPose(newPose);
    }

    /**
     * Resets where the robot thinks it is on the field.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    /**
     * Gets the confidence of an estimated robot pose.
     * 
     * @param estimation The estimated position of the robot
     * @return A matrix representing the confidence of the position
     */
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

    /**
     * Checks if a camera can see any tags and, if so, adds its current measurement to the swerve subsystem.
     * 
     * @param estimator The camera to check
     */
    public void estimatorChecker(PhotonRunnable estimator) {
        EstimatedRobotPose cameraPose = estimator.grabLatestEstimatedPose();
        if (cameraPose == null) {
            NetworkedTelemetry.Vision.setHasValidAprilTags(false);
            return;
        }

        NetworkedTelemetry.Vision.setHasValidAprilTags(true);
        Pose2d pose2d = cameraPose.estimatedPose.toPose2d();
        swerveSubsystem.addVisionMeasurement(pose2d, cameraPose.timestampSeconds,
            confidenceCalculator(cameraPose));
    }
}