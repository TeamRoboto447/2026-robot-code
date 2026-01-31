package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

    public static class VisionConstants {
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
        public static final double POSE_AMBIGUITY_SHIFTER = 0;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 0;
        public static final double NOISY_DISTANCE_METERS = 0;
        public static final double DISTANCE_WEIGHT = 0;
        public static final int TAG_PRESENCE_WEIGHT = 0;

        public static final boolean USE_VISION = true;

        public static final Transform3d ROBOT_TO_FRONT_CAM = new Transform3d(
        new Translation3d(Units.inchesToMeters(12.375), Units.inchesToMeters(-1.5), Units.inchesToMeters(5)),
        new Rotation3d(0, Units.degreesToRadians(-5), 0));
        public static final Transform3d ROBOT_TO_BACK_CAM = new Transform3d(
        new Translation3d(Units.inchesToMeters(-12.875), Units.inchesToMeters(5.25), Units.inchesToMeters(4.875)),
        new Rotation3d(0, Units.degreesToRadians(5), Units.degreesToRadians(180)));

        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(), 1,
        1, 1 * Math.PI);

    }

    public static class FieldConstants {
        public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.22);
        public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.69);

        public enum FieldZone {
            RED_ALLIANCE_ZONE,
            BLUE_ALLIANCE_ZONE,
            AUDIENCE_NEUTRAL_ZONE,
            SCORING_NEUTRAL_ZONE,
            OUT_OF_FIELD
        };

        /*
        ALL AREA MEASUREMENTS ARE IN METERS!!!

        Also, Neutral Zone names are from blue side.
        */
        public static class FieldZoneAreas {
            public static final Rectangle2d RED_ALLIANCE_ZONE_AREA = new Rectangle2d(
                new Translation2d(11.91, 0),
                new Translation2d(16.54, 8.07));    // Field length and width
            public static final Rectangle2d BLUE_ALLIANCE_ZONE_AREA = new Rectangle2d(
                new Translation2d(0, 0),
                new Translation2d(4.63, 8.07));

            public static final Rectangle2d AUDIENCE_NEUTRAL_ZONE_AREA = new Rectangle2d(
                new Translation2d(4.63, 0), 
                new Translation2d(11.91, 4.035));
            public static final Rectangle2d SCORING_NEUTRAL_ZONE_AREA = new Rectangle2d(
                new Translation2d(4.63, 4.035),
                new Translation2d(11.91, 8.07));
        }
    }
}
