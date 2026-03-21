// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.USE_VISION;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final PhotonRunnable backLeftCamera;
    private final PhotonRunnable backCamera;
    private final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /** Creates a new PoseEstimatorSubsystem. */
    public PoseEstimatorSubsystem(CommandSwerveDrivetrain swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        if (USE_VISION) {
            this.backLeftCamera = new PhotonRunnable(new PhotonCamera("FrontCam"), VisionConstants.ROBOT_TO_BACK_LEFT_CAM);
            this.backCamera = new PhotonRunnable(new PhotonCamera("BackCam"), VisionConstants.ROBOT_TO_BACK_RIGHT_CAM);
            this.setDefaultCommand(this.createNotifierCommand(this));
        } else {
            this.backLeftCamera = null;
            this.backCamera = null;
        }
    }

    /**
     * Updates the subsystem's various elements.
     */
    @Override
    public void periodic() {
        if (VisionConstants.USE_VISION) {
            boolean anyValid = false;
            anyValid |= estimatorChecker(backLeftCamera);
            anyValid |= estimatorChecker(backCamera);
            NetworkedTelemetry.Vision.setHasValidAprilTags(anyValid);

            List<PhotonTrackedTarget> allDetectedTags = List.of();
            allDetectedTags.addAll(backLeftCamera.grabDetectedTags());
            allDetectedTags.addAll(backCamera.grabDetectedTags());

            List<Pose3d> detectedTagPositions = List.of();
            allDetectedTags.forEach((PhotonTrackedTarget tag) -> {
                Optional<Pose3d> optionalPose = aprilTagLayout.getTagPose(tag.getFiducialId());
                if (optionalPose.isPresent()) detectedTagPositions.add(optionalPose.get());
            });

            NetworkedTelemetry.Vision.setDetectedTagPostions(detectedTagPositions);
        }
    }

    /**
     * Creates a new notifier command that continues running while the robot is disabled,
     * ensuring vision pose estimation remains active in all robot states.
     */
    private Command createNotifierCommand(PoseEstimatorSubsystem peSubsystem) {
        return new NotifierCommand(() -> {
            backLeftCamera.run();
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

        // Scale std devs up linearly with distance beyond the threshold.
        double distanceBeyondThreshold = Math.max(0, smallestDistance - VisionConstants.VISION_STD_DEV_SCALE_DISTANCE);
        double distanceMultiplier = 1.0 + distanceBeyondThreshold * VisionConstants.VISION_STD_DEV_SCALE_FACTOR;

        // For single-tag estimates, also penalize high pose ambiguity.
        // Multi-tag estimates are inherently more reliable so no extra penalty.
        double ambiguityMultiplier = 1.0;
        if (estimation.targetsUsed.size() == 1) {
            double ambiguity = estimation.targetsUsed.get(0).getPoseAmbiguity();
            // Linearly scale from 1x at 0 ambiguity to 3x at the rejection threshold.
            ambiguityMultiplier = 1.0 + (ambiguity / VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD) * 2.0;
        }

        double confidenceMultiplier = distanceMultiplier * ambiguityMultiplier;
        return VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }

    /**
     * Checks if a camera can see any tags and, if so, adds its current measurement to the swerve subsystem.
     *
     * <p>While the robot is disabled, vision measurements are trusted implicitly by
     * passing near-zero standard deviations, allowing the pose estimator to fully
     * snap to the vision-derived pose. This ensures the robot's odometry is
     * accurately seeded before autonomous begins.</p>
     *
     * <p>During enabled operation, the normal {@link #confidenceCalculator} is used
     * so that vision only gently nudges the wheel-odometry estimate.</p>
     * 
     * @param estimator The camera to check
     * @return true if the camera had a valid pose estimate this cycle
     */
    public boolean estimatorChecker(PhotonRunnable estimator) {
        EstimatedRobotPose cameraPose = estimator.grabLatestEstimatedPose();
        if (cameraPose == null) {
            return false;
        }

        Pose2d pose2d = cameraPose.estimatedPose.toPose2d();

        // While disabled, trust vision completely so the pose is fully seeded
        // from AprilTags before autonomous starts.
        Matrix<N3, N1> stdDevs = DriverStation.isDisabled()
            ? VisionConstants.VISION_DISABLED_STANDARD_DEVIATIONS
            : confidenceCalculator(cameraPose);

        swerveSubsystem.addVisionMeasurement(pose2d, cameraPose.timestampSeconds, stdDevs);
        return true;
    }
}