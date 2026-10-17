package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldConstants.FieldZone;

/**
 * Abstact interface to make it easier to implement pose retreival.
 */
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

    /**
     * Gets the current chassis speeds of the robot (robot-relative, as CTRE's swerve state
     * reports them). Convert with {@code ChassisSpeeds.fromRobotRelativeSpeeds} for field use.
     *
     * @return The robot's current {@link ChassisSpeeds}
     */
    ChassisSpeeds getChassisSpeeds();

    /**
     * How long ago the pose and speeds returned by this provider were measured.
     * Used to project the pose forward to "now" for shoot-on-the-fly aiming.
     *
     * @return The age of the current pose estimate, in seconds
     */
    double getPoseAgeSeconds();
}
