package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.RollingAveragePose2d;
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

    public Pose2d getAverageRobotPose() {
        return rollingAvg.getAveragePose();
    }

    public void resetPose(Pose2d robotPose) {
        Pose3d questPose = new Pose3d(robotPose).transformBy(VisionConstants.ROBOT_TO_QUEST);
        questNav.setPose(questPose);
    }    

    public void updateTelemetry() {
        questNav.commandPeriodic();
        batteryPublisher.set(questNav.getBatteryPercent().orElse(0));
        connectedPublisher.set(questNav.isConnected());
    }

    
}
