// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.FieldConstants;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import frc.robot.utils.MathUtils;

/** Add your docs here. */
public class PhotonRunnable implements Runnable {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

    public PhotonRunnable(PhotonCamera camera, Transform3d robotToCamera) {
        this.photonCamera = camera;
        PhotonPoseEstimator poseEstimator = null;
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        if (this.photonCamera != null) {
            poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
        this.photonPoseEstimator = poseEstimator;
    }

    @Override
    public void run() {
        if (this.photonPoseEstimator != null && this.photonCamera != null) {
            List<PhotonPipelineResult> photonResults = this.photonCamera.getAllUnreadResults();
            for(PhotonPipelineResult result : photonResults) {
                if (result.hasTargets() && (result.targets.size() > 1 || result.targets.get(0).getPoseAmbiguity() > APRILTAG_AMBIGUITY_THRESHOLD)) {
                    this.photonPoseEstimator.update(result).ifPresent(estimatedRobotPose -> {
                        Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
                        if (MathUtils.withinRange(estimatedPose.getX(), 0, FieldConstants.FIELD_LENGTH_METERS) && MathUtils.withinRange(estimatedPose.getY(), 0, FieldConstants.FIELD_WIDTH_METERS)) {
                            atomicEstimatedRobotPose.set(estimatedRobotPose);
                        }
                    });
                }
            }
        }
    }

    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }
}