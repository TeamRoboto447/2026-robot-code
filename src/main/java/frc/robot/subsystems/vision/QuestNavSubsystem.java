package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class QuestNavSubsystem extends SubsystemBase {
    private final QuestNav questNav;

    private final CommandSwerveDrivetrain swerveSubsystem;

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
                swerveSubsystem.addVisionMeasurement(robotPose.toPose2d(), frame.dataTimestamp(),
                        VisionConstants.VISION_DISABLED_STANDARD_DEVIATIONS);
            }
        }
    }

    public void setPose(Pose3d newPose) {
        questNav.setPose(newPose.plus(VisionConstants.ROBOT_TO_QUEST));
    }

    public void resetPose() {
        questNav.setPose((new Pose3d()).plus(VisionConstants.ROBOT_TO_QUEST));
    }
}