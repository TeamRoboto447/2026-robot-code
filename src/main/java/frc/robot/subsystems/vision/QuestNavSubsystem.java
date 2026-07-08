package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.networking.NetworkedTelemetry.QuestNavNT;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class QuestNavSubsystem extends SubsystemBase {
    private final QuestNav questNav;

    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final CommandSwerveDrivetrain swerveSubsystem;
    private Pose3d lastRobotPose = new Pose3d();

    public QuestNavSubsystem(CommandSwerveDrivetrain swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.questNav = new QuestNav();
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();

        for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
            if (frame.isTracking()) {
                Pose3d robotPose = frame.questPose3d()
                        .transformBy(VisionConstants.ROBOT_TO_QUEST.inverse());
                // Feed to your pose estimator:
                // driveSubsystem.addVisionMeasurement(
                // robotPose.toPose2d(), frame.dataTimestamp(), stdDevs);
                // if (fieldLayout.getFieldLength() >= robotPose.getX()) && (robotPose.getX() >= 0)
                swerveSubsystem.addVisionMeasurement(robotPose.toPose2d(), frame.dataTimestamp(),
                        VisionConstants.VISION_DISABLED_STANDARD_DEVIATIONS);

                lastRobotPose = robotPose;
            }
        }

        publishNetworkTableData();
    }

    public void setPose(Pose3d newPose) {
        questNav.setPose(newPose.transformBy(VisionConstants.ROBOT_TO_QUEST));
    }

    public void resetPose() {
        questNav.setPose((new Pose3d()).transformBy(VisionConstants.ROBOT_TO_QUEST));
    }

    private void publishNetworkTableData() {
        QuestNavNT.set3dPose(lastRobotPose);

        QuestNavNT.setConnected(questNav.isConnected());
        QuestNavNT.setTracking(questNav.isTracking());
        QuestNavNT.setLatency(questNav.getLatency());
        questNav.getBatteryPercent().ifPresent(
            b -> QuestNavNT.setBattery(b)
        );
        questNav.getTrackingLostCounter().ifPresent(
            c -> QuestNavNT.setTrackingLostCount(c)
        );

    }
}