package frc.robot.networking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.Constants.FieldConstants.FieldZone;
import frc.robot.Constants.FieldConstants.FieldZoneAreas;

/**
 * NetworkedTelemetry centralizes generic robot telemetry publishing to NetworkTables.
 * This class handles telemetry that is not directly related to subsystem configurations,
 * such as robot pose and vision system status.
 */
public class NetworkedTelemetry {
    private static final NetworkTableInstance defaultNTInstance = NetworkTableInstance.getDefault();
    
    /**
     * Robot pose telemetry for field positioning.
     */
    public static class Pose {
        private static final NetworkTable poseTable = defaultNTInstance.getTable("Pose");
        
        private static final DoubleArrayPublisher robotPosePub = poseTable
            .getDoubleArrayTopic("robotPose").publish();
        private static final StringPublisher fieldTypePub = poseTable
            .getStringTopic(".type").publish();
        
        // Field zone rectangle visualization
        private static final DoubleArrayPublisher fieldZoneRectPub = poseTable
            .getDoubleArrayTopic("FieldZone").publish();
        
        // Target position circle visualization
        private static final DoubleArrayPublisher targetCirclePub = poseTable
            .getDoubleArrayTopic("Target").publish();
        
        private static final double[] poseArray = new double[3];
        
        /**
         * Publishes the robot's pose to NetworkTables for field visualization.
         * 
         * @param pose The robot's current Pose2d
         */
        public static void publishRobotPose(Pose2d pose) {
            fieldTypePub.set("Field2d");
            poseArray[0] = pose.getX();
            poseArray[1] = pose.getY();
            poseArray[2] = pose.getRotation().getDegrees();
            robotPosePub.set(poseArray);
        }
        
        /**
         * Publishes a rectangle around the current field zone for visualization.
         * The rectangle is drawn using 5 corner points (4 corners + closing point).
         * 
         * @param zone The current field zone
         */
        public static void publishFieldZoneRectangle(FieldZone zone) {
            Rectangle2d rect = getZoneRectangle(zone);
            if (rect == null) {
                // Clear the rectangle if zone is out of field or unknown
                fieldZoneRectPub.set(new double[0]);
                return;
            }
            
            // Rectangle2d uses center and dimensions, so calculate corners
            Pose2d centerPose = rect.getCenter();
            Translation2d center = centerPose.getTranslation();
            double halfWidth = rect.getXWidth() / 2.0;
            double halfHeight = rect.getYWidth() / 2.0;
            
            // Calculate the four corners
            double minX = center.getX() - halfWidth;
            double maxX = center.getX() + halfWidth;
            double minY = center.getY() - halfHeight;
            double maxY = center.getY() + halfHeight;
            
            // Flatten to array: [x1, y1, rot1, x2, y2, rot2, ...]
            double[] rectArray = new double[15]; // 5 poses * 3 values each
            
            // Bottom left
            rectArray[0] = minX;
            rectArray[1] = minY;
            rectArray[2] = 0;
            
            // Top left
            rectArray[3] = minX;
            rectArray[4] = maxY;
            rectArray[5] = 0;
            
            // Top right
            rectArray[6] = maxX;
            rectArray[7] = maxY;
            rectArray[8] = 0;
            
            // Bottom right
            rectArray[9] = maxX;
            rectArray[10] = minY;
            rectArray[11] = 0;
            
            // Close the rectangle back to bottom left
            rectArray[12] = minX;
            rectArray[13] = minY;
            rectArray[14] = 0;
            
            fieldZoneRectPub.set(rectArray);
        }
        
        /**
         * Publishes a circle at the target position for visualization.
         * The circle is approximated using 16 points.
         * 
         * @param target The target position (uses only X and Y, ignoring Z)
         * @param radiusMeters The radius of the circle in meters
         */
        public static void publishTargetCircle(Translation3d target, double radiusMeters) {
            if (target == null) {
                // Clear the circle if no target
                targetCirclePub.set(new double[0]);
                return;
            }
            
            // Create circle with 16 points
            int numPoints = 16;
            double[] circleArray = new double[numPoints * 3]; // 16 poses * 3 values each
            
            for (int i = 0; i < numPoints; i++) {
                double angle = 2 * Math.PI * i / numPoints;
                circleArray[i * 3] = target.getX() + radiusMeters * Math.cos(angle);
                circleArray[i * 3 + 1] = target.getY() + radiusMeters * Math.sin(angle);
                circleArray[i * 3 + 2] = 0; // rotation
            }
            
            targetCirclePub.set(circleArray);
        }
        
        /**
         * Helper method to get the Rectangle2d for a given field zone.
         * 
         * @param zone The field zone
         * @return The Rectangle2d representing the zone boundaries, or null if OUT_OF_FIELD
         */
        private static Rectangle2d getZoneRectangle(FieldZone zone) {
            switch (zone) {
                case RED_ALLIANCE_AUDIENCE_SIDE:
                    return FieldZoneAreas.RED_ALLIANCE_AUDIENCE_SIDE_AREA;
                case RED_ALLIANCE_SCORING_SIDE:
                    return FieldZoneAreas.RED_ALLIANCE_SCORING_SIDE_AREA;
                case BLUE_ALLIANCE_AUDIENCE_SIDE:
                    return FieldZoneAreas.BLUE_ALLIANCE_AUDIENCE_SIDE_AREA;
                case BLUE_ALLIANCE_SCORING_SIDE:
                    return FieldZoneAreas.BLUE_ALLIANCE_SCORING_SIDE_AREA;
                case AUDIENCE_NEUTRAL_ZONE:
                    return FieldZoneAreas.AUDIENCE_NEUTRAL_ZONE_AREA;
                case SCORING_NEUTRAL_ZONE:
                    return FieldZoneAreas.SCORING_NEUTRAL_ZONE_AREA;
                case OUT_OF_FIELD:
                default:
                    return null;
            }
        }
    }
    
    /**
     * Vision system telemetry.
     */
    public static class Vision {
        private static final NetworkTable visionTable = defaultNTInstance.getTable("Vision");
        
        private static final BooleanEntry hasValidAprilTags = visionTable
            .getBooleanTopic("Has Valid AprilTags").getEntry(false);
        
        /**
         * Sets whether the vision system currently has valid AprilTag detections.
         * 
         * @param hasValid True if valid AprilTags are detected, false otherwise
         */
        public static void setHasValidAprilTags(boolean hasValid) {
            hasValidAprilTags.set(hasValid);
        }
        
        /**
         * Gets whether the vision system currently has valid AprilTag detections.
         * 
         * @return True if valid AprilTags are detected, false otherwise
         */
        public static boolean hasValidAprilTags() {
            return hasValidAprilTags.get();
        }
    }
}
