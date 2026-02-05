package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

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
import edu.wpi.first.units.measure.Angle;

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

        public enum TurretTarget {
            RED_HUB,
            BLUE_HUB,
            AUDIENCE_CORNER,
            SCORING_CORNER,
            NONE
        }

        /*
        ALL TRANSLATION MEASUREMENTS ARE IN METERS!!!
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

        public static class TurretTargetPoints {
            public static final Translation3d RED_HUB = new Translation3d(0, 4.035, 1.83);
            public static final Translation3d RED_LEFT_CORNER = new Translation3d(15.54, 7.07, 0);
            public static final Translation3d RED_RIGHT_CORNER = new Translation3d(15.54, 1, 0);
            
            public static final Translation3d BLUE_HUB = new Translation3d(0, 4.035, 1.83);
            public static final Translation3d BLUE_LEFT_CORNER = new Translation3d(1, 7.07, 0);
            public static final Translation3d BLUE_RIGHT_CORNER = new Translation3d(1, 1, 0);
        }
    }

    public static class TurretSubsystemConstants {
        public static final int LEFT_SHOOTER_MOTOR_ID = 40;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 41;
        public static final int HOOD_MOTOR_ID = 42;
        public static final int ANGLE_MOTOR_ID = -1;            //
        public static final int KICKER_MOTOR_ID = -1;           //  TODO: SET PROPER IDS FOR THESE MOTORS!!!

        public static final double SHOOTER_KP = 0;
        public static final double SHOOTER_KI = 0;
        public static final double SHOOTER_KD = 0;

        public static final int LOOKUP_TABLE_VEL_STEP = 1;
        public static final int LOOKUP_TABLE_DIST_STEP = 1;

        public static final Angle MIN_HOOD_ANGLE = Degrees.of(17);
        public static final Angle MAX_HOOD_ANGLE = Degrees.of(45);
        public static final Angle HOOD_DEGREES_ROTATION_RATIO = Degrees.of(19.5);
    }

    public static class IntakeSubsystemConstants {
        public static final int LIFT_MOTOR_ID = -1;
        public static final int INTAKE_MOTOR_ID = -1;
    }

    public static class IndexerSubsystemConstants {
        public static final int SPINNER_MOTOR_ID = -1;
    }
}
