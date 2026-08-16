package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AdvScopeConstants;
import frc.robot.Constants.FieldConstants;

public class SimPhysics {
    public SimPhysics() {}

    public Pose3d applyBumpAngle(Pose2d basePose) {
        double robotYaw = basePose.getRotation().getRadians();

        double projectedWheelbaseLength = 
            AdvScopeConstants.ROBOT_WHEELBASE_LENGTH * Math.abs(Math.cos(robotYaw))
            + AdvScopeConstants.ROBOT_WHEELBASE_WIDTH * Math.abs(Math.sin(robotYaw));
        
        double bumpHalfWidth = Units.inchesToMeters(22.2);
        double bumpAngle = Units.degreesToRadians(15.0);

        double halfRobotLength = projectedWheelbaseLength / 2.0;

        Pose3d currentRobotPose = new Pose3d(
            new Translation3d(
                basePose.getX(),
                basePose.getY(),
                0.0
            ),
            new Rotation3d(
                0.0,
                0.0,
                basePose.getRotation().getRadians()
            )
        );

        Translation2d baseTranslation = basePose.getTranslation();

        double rawDistFromBump;

        if (FieldConstants.BumpZones.RED_DEPOT_BUMP_ZONE.contains(baseTranslation) ||
            FieldConstants.BumpZones.RED_OUTPOST_BUMP_ZONE.contains(baseTranslation)) {
            rawDistFromBump = baseTranslation.getX() - Units.inchesToMeters(469.11);
        } else if (FieldConstants.BumpZones.BLUE_DEPOT_BUMP_ZONE.contains(baseTranslation) ||
            FieldConstants.BumpZones.BLUE_OUTPOST_BUMP_ZONE.contains(baseTranslation)) {

            rawDistFromBump = baseTranslation.getX() - Units.inchesToMeters(182.11);
        } else {
            return currentRobotPose;
        }

        double side = Math.signum(rawDistFromBump);
        double distanceFromBumpCenter = Math.abs(rawDistFromBump);

        if (side == 0.0) {
            side = 1.0;
        }

        double angle;
        double height;

        double maxAngleDistance =
                bumpHalfWidth
                - halfRobotLength * Math.cos(bumpAngle);

        double offBumpDistance =
                bumpHalfWidth
                + halfRobotLength;


        // ------------------------------------------------------------
        // Robot is completely off the bump
        // ------------------------------------------------------------

        if (distanceFromBumpCenter >= offBumpDistance) {

            angle = 0.0;
            height = 0.0;
        }


        // ------------------------------------------------------------
        // Robot is sitting on the 15 degree slope
        // ------------------------------------------------------------

        else if (distanceFromBumpCenter <= maxAngleDistance) {

            angle = bumpAngle * side;

            height =
                    Math.tan(bumpAngle)
                    * (bumpHalfWidth - distanceFromBumpCenter);
        }


        // ------------------------------------------------------------
        // Robot is transitioning onto/off of the bump
        // ------------------------------------------------------------

        else {

            double lowAngle = 0.0;
            double highAngle = bumpAngle;

            for (int i = 0; i < 50; i++) {

                double testAngle =
                        (lowAngle + highAngle) / 2.0;

                double horizontalHalfLength =
                        halfRobotLength
                        * Math.cos(testAngle);

                double centerHeight =
                        halfRobotLength
                        * Math.sin(testAngle);

                double bumpEndX =
                        distanceFromBumpCenter
                        - horizontalHalfLength;

                double bumpHeight =
                        Math.tan(bumpAngle)
                        * (bumpHalfWidth - bumpEndX);

                double robotEndHeight =
                        centerHeight
                        + halfRobotLength
                        * Math.sin(testAngle);

                double error =
                        robotEndHeight - bumpHeight;

                if (error > 0.0) {
                    highAngle = testAngle;
                } else {
                    lowAngle = testAngle;
                }
            }

            angle =
                    ((lowAngle + highAngle) / 2.0)
                    * side;

            height =
                    halfRobotLength
                    * Math.sin(Math.abs(angle));
        }

        Translation3d bumpTranslation = new Translation3d(0, 0, height);
        Rotation3d bumpRotation = new Rotation3d(0.0, angle, 0.0);

        return new Pose3d(
            currentRobotPose.getTranslation().plus(bumpTranslation),
            currentRobotPose.getRotation().plus(bumpRotation)
        );
    }
}
