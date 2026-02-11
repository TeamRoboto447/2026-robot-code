package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants.FieldZone;

public interface PoseProvider {
    /**
     * Gets the current estimated pose of the robot on the field.
     * 
     * @return The robot's current Pose2d (position and rotation)
     */
    Pose2d getPose();
    
    /**
     * Gets the current field zone the robot is located in.
     * 
     * @return The FieldZone enum representing the robot's current zone
     */
    FieldZone getFieldZone();
}
