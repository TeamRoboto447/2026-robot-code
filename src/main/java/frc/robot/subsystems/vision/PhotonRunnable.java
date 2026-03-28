// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.FieldConstants;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import frc.robot.utils.MathUtils;

/**
 * A single camera instance, used in the {@link PoseEstimatorSubsystem}.
 */
public class PhotonRunnable implements Runnable {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
    private ArrayList<PhotonTrackedTarget> detectedTags = new ArrayList<>();

    /**
     * Creates a new PhotonRunnable.
     * @param camera The camera to get data from.
     * @param robotToCamera The positional offset of the camera from the center of the robot in the form of a {@link Transform3d} object.
     */
    public PhotonRunnable(PhotonCamera camera, Transform3d robotToCamera) {
        this.photonCamera = camera;
        PhotonPoseEstimator poseEstimator = null;
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        if (this.photonCamera != null) {
            poseEstimator = new PhotonPoseEstimator(layout, robotToCamera);
        }
        this.photonPoseEstimator = poseEstimator;
    }

    /**
     * Contains the code to run every iteration.
     */
    @Override
    public void run() {
        if (this.photonPoseEstimator != null && this.photonCamera != null) {
            List<PhotonPipelineResult> photonResults = this.photonCamera.getAllUnreadResults();
            for (PhotonPipelineResult result : photonResults) {
                detectedTags.clear();

                if (!result.hasTargets()) continue;

                // Prefer coprocessor multi-tag; fall back to lowest-ambiguity single-tag.
                // Both methods are the non-deprecated direct estimation API in photonlib 2026.
                var estimation = photonPoseEstimator.estimateCoprocMultiTagPose(result);
                if (estimation.isEmpty()) {
                    if (result.targets.size() == 1
                            && result.targets.get(0).getPoseAmbiguity() <= APRILTAG_AMBIGUITY_THRESHOLD) {
                        estimation = photonPoseEstimator.estimateLowestAmbiguityPose(result);
                    }
                }

                estimation.ifPresent(estimatedRobotPose -> {
                    Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
                    if (MathUtils.withinRange(estimatedPose.getX(), 0, FieldConstants.FIELD_LENGTH_METERS)
                            && MathUtils.withinRange(estimatedPose.getY(), 0, FieldConstants.FIELD_WIDTH_METERS)) {
                        atomicEstimatedRobotPose.set(estimatedRobotPose);
                    }
                });

                detectedTags.addAll(result.getTargets());
            }
        }
    }

    /**
     * Gets the latest estimated pose of the robot.
     * @return The estimated pose.
     */
    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }

    public List<PhotonTrackedTarget> grabDetectedTags() {
        return detectedTags;
    }
}