package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AdvScopeConstants;
import frc.robot.Constants.FieldConstants;

public class SimPhysics {
    private final Rectangle2d fieldRectangle;

    public SimPhysics() {
        this.fieldRectangle = new Rectangle2d(
            new Translation2d(), 
            new Translation2d(FieldConstants.FIELD_LENGTH_METERS, FieldConstants.FIELD_WIDTH_METERS));
    }

    public Pose3d applyBumpAngle(Pose2d basePose) {
        double robotYaw = basePose.getRotation().getRadians();

        double projectedWheelbaseLength = 
            AdvScopeConstants.ROBOT_WHEELBASE_LENGTH * Math.abs(Math.cos(robotYaw))
            + AdvScopeConstants.ROBOT_WHEELBASE_WIDTH * Math.abs(Math.sin(robotYaw));
        
        double bumpHalfWidth = Units.inchesToMeters(22.2);
        double bumpAngle = Units.degreesToRadians(15.0);

        double halfRobotLength = projectedWheelbaseLength / 2.0;

        Pose3d currentRobotPose = new Pose3d(
            new Translation3d(basePose.getTranslation()),
            new Rotation3d(basePose.getRotation()));

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

    public Pose3d checkForElement(Pose3d basePose, Rectangle2d fieldElement) {

        double halfLength = Units.inchesToMeters(36.68) / 2.0;
        double halfWidth  = Units.inchesToMeters(33.68) / 2.0;

        Translation2d center =
                basePose.getTranslation().toTranslation2d();

        Rotation2d rotation =
                new Rotation2d(basePose.getRotation().getZ());

        double cos = rotation.getCos();
        double sin = rotation.getSin();

        // ------------------------------------------------------------
        // Robot corners
        // ------------------------------------------------------------

        Translation2d[] robotCorners = {
            transform(
                    new Translation2d( halfLength,  halfWidth),
                    center, cos, sin),

            transform(
                    new Translation2d( halfLength, -halfWidth),
                    center, cos, sin),

            transform(
                    new Translation2d(-halfLength, -halfWidth),
                    center, cos, sin),

            transform(
                    new Translation2d(-halfLength,  halfWidth),
                    center, cos, sin)
        };

        // ------------------------------------------------------------
        // Field element corners
        // Rectangle2d is axis-aligned
        // ------------------------------------------------------------

        Translation2d elementCenter =
                fieldElement.getCenter().getTranslation();

        double halfElementX =
                fieldElement.getXWidth() / 2.0;

        double halfElementY =
                fieldElement.getYWidth() / 2.0;

        Translation2d[] elementCorners = {
            new Translation2d(
                    elementCenter.getX() - halfElementX,
                    elementCenter.getY() - halfElementY),

            new Translation2d(
                    elementCenter.getX() + halfElementX,
                    elementCenter.getY() - halfElementY),

            new Translation2d(
                    elementCenter.getX() + halfElementX,
                    elementCenter.getY() + halfElementY),

            new Translation2d(
                    elementCenter.getX() - halfElementX,
                    elementCenter.getY() + halfElementY)
        };

        // ------------------------------------------------------------
        // Test for overlap
        // ------------------------------------------------------------

        Translation2d[] axes = {
            getNormal(robotCorners[0], robotCorners[1]),
            getNormal(robotCorners[1], robotCorners[2]),
            getNormal(elementCorners[0], elementCorners[1]),
            getNormal(elementCorners[1], elementCorners[2])
        };

        double smallestPush = Double.POSITIVE_INFINITY;
        Translation2d smallestAxis = null;

        for (Translation2d axis : axes) {

            double[] robotProjection =
                    project(robotCorners, axis);

            double[] elementProjection =
                    project(elementCorners, axis);

            // No overlap on this axis means the rectangles
            // aren't touching.
            if (robotProjection[1] < elementProjection[0] ||
                elementProjection[1] < robotProjection[0]) {

                return basePose;
            }

            // Calculate penetration in both directions
            double pushPositive =
                    elementProjection[1] - robotProjection[0];

            double pushNegative =
                    robotProjection[1] - elementProjection[0];

            double push = Math.min(pushPositive, pushNegative);

            if (push < smallestPush) {
                smallestPush = push;

                // Determine which direction moves the robot
                // AWAY from the element.
                double robotCenterProjection =
                        center.getX() * axis.getX()
                        + center.getY() * axis.getY();

                double elementCenterProjection =
                        elementCenter.getX() * axis.getX()
                        + elementCenter.getY() * axis.getY();

                if (robotCenterProjection < elementCenterProjection) {
                    smallestAxis = new Translation2d(
                            -axis.getX(),
                            -axis.getY());
                } else {
                    smallestAxis = axis;
                }
            }
        }

        // ------------------------------------------------------------
        // Push robot out of the element
        // ------------------------------------------------------------

        Translation2d correctedCenter =
                center.plus(
                        smallestAxis.times(smallestPush));

        return new Pose3d(
                new Translation3d(
                    correctedCenter.getX(),
                    correctedCenter.getY(),
                    basePose.getZ()
                ),
                basePose.getRotation());
    }

    public Pose3d checkFieldBoundries(Pose3d basePose) {

        double halfLength = Units.inchesToMeters(36.68) / 2.0;
        double halfWidth  = Units.inchesToMeters(33.68) / 2.0;

        Translation2d center = basePose.getTranslation().toTranslation2d();
        Rotation2d rotation = basePose.getRotation().toRotation2d();

        double cos = rotation.getCos();
        double sin = rotation.getSin();

        // Calculate the robot's four corners
        Translation2d[] corners = {
            transform(
                    new Translation2d( halfLength,  halfWidth),
                    center, cos, sin),

            transform(
                    new Translation2d( halfLength, -halfWidth),
                    center, cos, sin),

            transform(
                    new Translation2d(-halfLength, -halfWidth),
                    center, cos, sin),

            transform(
                    new Translation2d(-halfLength,  halfWidth),
                    center, cos, sin)
        };

        // Find robot extents
        double robotMinX = Double.POSITIVE_INFINITY;
        double robotMaxX = Double.NEGATIVE_INFINITY;
        double robotMinY = Double.POSITIVE_INFINITY;
        double robotMaxY = Double.NEGATIVE_INFINITY;

        for (Translation2d corner : corners) {
            robotMinX = Math.min(robotMinX, corner.getX());
            robotMaxX = Math.max(robotMaxX, corner.getX());
            robotMinY = Math.min(robotMinY, corner.getY());
            robotMaxY = Math.max(robotMaxY, corner.getY());
        }

        // Field bounds
        Translation2d boundsCenter =
                fieldRectangle.getCenter().getTranslation();

        double fieldMinX =
                boundsCenter.getX()
                - fieldRectangle.getXWidth() / 2.0;

        double fieldMaxX =
                boundsCenter.getX()
                + fieldRectangle.getXWidth() / 2.0;

        double fieldMinY =
                boundsCenter.getY()
                - fieldRectangle.getYWidth() / 2.0;

        double fieldMaxY =
                boundsCenter.getY()
                + fieldRectangle.getYWidth() / 2.0;

        double x = basePose.getX();
        double y = basePose.getY();

        /*
        * Push the robot back inside the field.
        */

        if (robotMinX < fieldMinX) {
            x += fieldMinX - robotMinX;
        }

        if (robotMaxX > fieldMaxX) {
            x -= robotMaxX - fieldMaxX;
        }

        if (robotMinY < fieldMinY) {
            y += fieldMinY - robotMinY;
        }

        if (robotMaxY > fieldMaxY) {
            y -= robotMaxY - fieldMaxY;
        }

        return new Pose3d(
                x,
                y,
                basePose.getZ(),
                basePose.getRotation());
    }

    private Translation2d transform(
        Translation2d local,
        Translation2d center,
        double cos,
        double sin) {

        double x =
                local.getX() * cos -
                local.getY() * sin;

        double y =
                local.getX() * sin +
                local.getY() * cos;

        return new Translation2d(
                center.getX() + x,
                center.getY() + y);
    }

    private Translation2d getNormal(
            Translation2d a,
            Translation2d b) {

        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();

        double length = Math.hypot(dx, dy);

        return new Translation2d(
                -dy / length,
                dx / length);
    }

    private double[] project(
            Translation2d[] points,
            Translation2d axis) {

        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;

        for (Translation2d point : points) {

            double projection =
                    point.getX() * axis.getX()
                    + point.getY() * axis.getY();

            min = Math.min(min, projection);
            max = Math.max(max, projection);
        }

        return new double[] { min, max };
    }
}
