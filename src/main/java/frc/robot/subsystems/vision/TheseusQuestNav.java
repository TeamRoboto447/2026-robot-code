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
import frc.robot.networking.NetworkedTelemetry.QuestNavNT;
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

    /**
     * Create an instance of this object.
     * @param windowSize The size of the window for the rolling pose average
     */
    public TheseusQuestNav(int windowSize) {
        questNav = new QuestNav();

        
        // Zero the absolute 3D position of the robot (similar to long-pressing the
        // quest logo)
        // if (questMiso.get() != 99) {
        //     questMosi.set(1);
        // }

        rollingAvg = new RollingAveragePose2d(windowSize);
    }

    /**
     * Create an instance of this object with a rolling average window size of 2.
     */
    public TheseusQuestNav() {
        this(2);
    }

    /**
     * Update the rolling pose average with the latest pose.
     */
    public void updateAverageRobotPose() {
        rollingAvg.addPose(getRobotPose());
    }

    /**
     * Gets the pose of the robot on the field.
     *
     * @return pose of the robot
     */
    public Pose2d getRobotPose() {
        // The robot is the Quest's pose transformed back by the quest->robot offset
        return getQuestPose().transformBy(robotToQuest.inverse());
    }

    public Pose2d getQuestPose() {
        var rawPose = getUncorrectedOculusPose();
        var poseRelativeToReset = rawPose.minus(resetPoseOculus);
        // Transform from "reset quest pose" to "current quest pose"

        return resetPoseRobot // the robot's field pose at reset
                .transformBy(robotToQuest) // offset to get the Quest's field pose at reset
                .transformBy(poseRelativeToReset);
    }

    /**
     * Gets the average pose from the rolling pose average.
     */
    public Pose2d getAverageRobotPose() {
        return rollingAvg.getAveragePose();
    }

    /**
     * Resets the quest pose a new position.
     * @param newPose The new pose to use
     */
    public void resetPose(Pose2d newPose) {
        rollingAvg.reset();
        resetPoseOculus = getUncorrectedOculusPose();
        resetPoseRobot = newPose;
    }

    private Pose2d latestRawQuestPose = null;

    // FPGA timestamp of latestRawQuestPose, and whether it arrived during the most recent update
    private double latestFrameTimestamp = 0.0;
    private boolean hasNewFrame = false;

    /**
     * Updates the NetworkTable telemetry and the internal values (similar to a subsystem's periodic function)
     */
    public void updateTelemetry() {
        questNav.commandPeriodic();
        hasNewFrame = false;
        for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
            if (frame.isTracking()) {
                latestRawQuestPose = frame.questPose3d().toPose2d();
                latestFrameTimestamp = frame.dataTimestamp();
                hasNewFrame = true;
            }
        }
        if (latestRawQuestPose != null) QuestNavNT.setRawQuestPose(new Pose3d(latestRawQuestPose));
        QuestNavNT.setBattery(questNav.getBatteryPercent().orElse(0));
        QuestNavNT.setLatency(questNav.getLatency());
        QuestNavNT.setConnected(questNav.isConnected());
        QuestNavNT.setTracking(questNav.isTracking());
        QuestNavNT.setTrackingLostCount(questNav.getTrackingLostCounter().orElse(999));
        QuestNavNT.setCorrectedQuestPose(new Pose3d(getQuestPose()));
    }

    /**
     * Whether a new tracking frame was received during the last {@link #updateTelemetry()}.
     */
    public boolean hasNewFrame() {
        return hasNewFrame;
    }

    /**
     * FPGA timestamp (seconds) of the most recent tracking frame.
     */
    public double getLatestFrameTimestamp() {
        return latestFrameTimestamp;
    }

    /**
     * Re
     * 
     * @return The raw Quest pose
     */
    private Pose2d getUncorrectedOculusPose() {
        return latestRawQuestPose != null ? latestRawQuestPose : new Pose2d();
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
