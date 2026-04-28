package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class QuestNavSubsystem extends SubsystemBase {
    private final QuestNav questNav = new QuestNav();

    // Offset from robot center to the Quest headset
    // Example: Quest is 0.3m forward, 0.0m left, 0.5m up from robot center
    private static final Transform3d ROBOT_TO_QUEST = new Transform3d(0.3, 0.0, 0.5, new Rotation3d());

    private final CommandSwerveDrivetrain swerveSubsystem;

    public QuestNavSubsystem(CommandSwerveDrivetrain swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();

        for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
            if (frame.isTracking()) {
                Pose3d robotPose = frame.questPose3d()
                        .transformBy(ROBOT_TO_QUEST.inverse());
                // Feed to your pose estimator:
                // driveSubsystem.addVisionMeasurement(
                // robotPose.toPose2d(), frame.dataTimestamp(), stdDevs);
                swerveSubsystem.addVisionMeasurement(robotPose.toPose2d(), frame.dataTimestamp(),
                        VisionConstants.VISION_DISABLED_STANDARD_DEVIATIONS);
            }
        }
    }
}