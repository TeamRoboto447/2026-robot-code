package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;
import frc.robot.networking.NetworkedTelemetry;
import frc.robot.utils.RollingAveragePose2d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class TheseusQuestNav {

    private final QuestNav questNav;

    // Pose of the Quest when the pose was reset
    private Pose2d resetPoseOculus = new Pose2d();

    // Pose of the robot when the pose was reset
    private Pose2d resetPoseRobot = new Pose2d();

    private final RollingAveragePose2d rollingAvg;
    
    private NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("QuestNavTelemetry");
    private BooleanPublisher connectedPublisher = telemetryTable.getBooleanTopic("Connected").publish();
    private DoublePublisher batteryPublisher = telemetryTable.getDoubleTopic("Battery %").publish();

    private final Transform2d robotToQuest =
        new Transform2d(
            VisionConstants.ROBOT_TO_QUEST.getTranslation().toTranslation2d(),
            VisionConstants.ROBOT_TO_QUEST.getRotation().toRotation2d()
        );

    public TheseusQuestNav(int windowSize) {
        questNav = new QuestNav();

        
        // Zero the absolute 3D position of the robot (similar to long-pressing the
        // quest logo)
        // if (questMiso.get() != 99) {
        //     questMosi.set(1);
        // }

        rollingAvg = new RollingAveragePose2d(windowSize);
    }

    public TheseusQuestNav() {
        this(2);
    }

    public void updateAverageRobotPose() {
        rollingAvg.addPose(getRobotPose());
    }

    /**
     * Gets the pose of the robot on the field
     *
     * @return pose of the robot
     */
    public Pose2d getRobotPose() {
        // The robot is the Quest's pose transformed back by the quest->robot offset
        return getQuestPose().transformBy(robotToQuest);
    }

    public Pose2d getQuestPose() {
        var rawPose = getUncorrectedOculusPose();
        var poseRelativeToReset = rawPose.minus(resetPoseOculus);
        // Transform from "reset quest pose" to "current quest pose"

        return resetPoseRobot // the robot's field pose at reset
                .transformBy(robotToQuest) // offset to get the Quest's field pose at reset
                .transformBy(poseRelativeToReset);
    }

    public Pose2d getAverageRobotPose() {
        return rollingAvg.getAveragePose();
    }

    public void resetPose(Pose2d newPose) {
        rollingAvg.reset();
        resetPoseOculus = getUncorrectedOculusPose();
        resetPoseRobot = newPose;
    }    

    public void updateTelemetry() {
        questNav.commandPeriodic();
        batteryPublisher.set(questNav.getBatteryPercent().orElse(0));
        connectedPublisher.set(questNav.isConnected());
    }

    private Pose2d getUncorrectedOculusPose() {
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        Pose3d questPose = null;

        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();
            }
        }
        NetworkedTelemetry.QuestNavNT.set3dPose(questPose);

        if (questPose != null) {
            return new Pose2d(
                questPose.getTranslation().toTranslation2d(),
                questPose.getRotation().toRotation2d()
            );
        } else {
            return rollingAvg.getAveragePose().transformBy(robotToQuest);
        }
    }
    /**
     * Check if the Quest is connected to the robot.
     * @return The connection state as a boolean
     */
    public boolean isConnected() {
        return questNav.isConnected();
    }

    /**
     * Cleans up the messages from the Quest.
     * 
     * Currently does nothing, just here for swapping
     */
    public void cleanUpQuestNavMessages() {
    }
}
