package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import frc.robot.libraries.Repulsor.Fields.FieldDefinition;
import frc.robot.libraries.Repulsor.Fields.Rebuilt2026;

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

    public static class RepulsorConstants {
    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    /**
     * Field length in meters (full field X dimension). Sourced from the
     * AprilTag field layout; keep units in meters.
     */
    public static final double FIELD_LENGTH = 16.540988;

    /**
     * Field width in meters (full field Y dimension). Sourced from the
     * AprilTag field layout; keep units in meters.
     */
    public static final double FIELD_WIDTH = aprilTagLayout.getFieldWidth();
    public static final FieldDefinition FIELD = new Rebuilt2026();

    /**
     * Robot external footprint (meters). These are the full robot
     * dimensions (length = X, width = Y) and SHOULD include bumpers and
     * any permanent protrusions. Use these values for collision checks
     * and pathing where the code expects full extents.
     */
    public static final double ROBOT_X = Units.inchesToMeters(33.583);
    public static final double ROBOT_Y = Units.inchesToMeters(36.583000);
    }

    /**
     * Holds constants related to the vision subsystem.
     */
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
        new Translation3d(Units.inchesToMeters(-11.875), Units.inchesToMeters(-10.375), Units.inchesToMeters(7.95)),
        new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(230.24)));

        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(), 1,
        1, 1 * Math.PI);

    }

    /**
     * Holds constants related to the field.
     */
    public static class FieldConstants {
        public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.22);
        public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.69);

        public enum FieldZone {
            RED_ALLIANCE_AUDIENCE_SIDE,
            RED_ALLIANCE_SCORING_SIDE,
            BLUE_ALLIANCE_AUDIENCE_SIDE,
            BLUE_ALLIANCE_SCORING_SIDE,
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
        TODO: consider using Units.inchesToMeters() for readability
        */
        /**
         * Holds areas considered to be part of different "field zones".
         */
        public static class FieldZoneAreas {
            // Red alliance zones (audience side is y=0 to y=4.035, scoring side is y=4.035 to y=8.07)
            public static final Rectangle2d RED_ALLIANCE_AUDIENCE_SIDE_AREA = new Rectangle2d(
                new Translation2d(11.91, 0),
                new Translation2d(16.54, 4.035));
            public static final Rectangle2d RED_ALLIANCE_SCORING_SIDE_AREA = new Rectangle2d(
                new Translation2d(11.91, 4.035),
                new Translation2d(16.54, 8.07));
            
            // Blue alliance zones (audience side is y=0 to y=4.035, scoring side is y=4.035 to y=8.07)
            public static final Rectangle2d BLUE_ALLIANCE_AUDIENCE_SIDE_AREA = new Rectangle2d(
                new Translation2d(0, 0),
                new Translation2d(4.63, 4.035));
            public static final Rectangle2d BLUE_ALLIANCE_SCORING_SIDE_AREA = new Rectangle2d(
                new Translation2d(0, 4.035),
                new Translation2d(4.63, 8.07));

            // Neutral zones
            public static final Rectangle2d AUDIENCE_NEUTRAL_ZONE_AREA = new Rectangle2d(
                new Translation2d(4.63, 0), 
                new Translation2d(11.91, 4.035));
            public static final Rectangle2d SCORING_NEUTRAL_ZONE_AREA = new Rectangle2d(
                new Translation2d(4.63, 4.035),
                new Translation2d(11.91, 8.07));
        }

        public static class TurretTargetPoints {
            public static final Translation3d RED_HUB = new Translation3d(11.92, 4.035, 1.83);
            public static final Translation3d RED_LEFT_CORNER = new Translation3d(15.54, 7.07, 0);
            public static final Translation3d RED_RIGHT_CORNER = new Translation3d(15.54, 1, 0);
            
            public static final Translation3d BLUE_HUB = new Translation3d(4.623, 4.035, 1.83);
            public static final Translation3d BLUE_LEFT_CORNER = new Translation3d(1, 7.07, 0);
            public static final Translation3d BLUE_RIGHT_CORNER = new Translation3d(1, 1, 0);
        }
    }

    public static class TurretSubsystemConstants {
        public static final int LEFT_SHOOTER_MOTOR_ID = 40;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 41;
        public static final int HOOD_MOTOR_ID = 42;
        public static final int ANGLE_MOTOR_ID = 43;
        public static final int KICKER_MOTOR_ID = 44;

        public static final double SHOOTER_KP = 0.5;
        public static final double SHOOTER_KI = 0;
        public static final double SHOOTER_KD = 0;
        public static final double SHOOTER_KV = 0.11;

        public static final double HOOD_KP = 0.05;
        public static final double HOOD_KI = 0;
        public static final double HOOD_KD = 0;

        public static final double TURRET_KP = 40;
        public static final double TURRET_KD = 0;
        public static final double TURRET_KI = 0;
        public static final double TURRET_KS = 0.5;

        public static final int LOOKUP_TABLE_VEL_STEP = 1;
        public static final int LOOKUP_TABLE_DIST_STEP = 1;

        public static final Angle MIN_HOOD_ANGLE = Degrees.of(23);
        public static final Angle MAX_HOOD_ANGLE = Degrees.of(44);
        public static final Angle HOOD_DEGREES_ROTATION_RATIO = Degrees.of(1);

        public static final Angle MIN_TURRET_ANGLE = Degrees.of(-90);
        public static final Angle MAX_TURRET_ANGLE = Degrees.of(80);

        public static final double TURRET_DEGREES_PER_ROTATION = 29.17;
        public static final double TURRET_ANGLE_TOLERANCE_DEGREES = 0.5;

        
        /** Open-loop output used while homing toward the lower hard stop. Negative = lower. */
        public static final double HOOD_HOMING_SPEED = -0.1;
        /**
         * Output-current threshold (amps) above which the NEO 550 is considered stalled.
         * NEO 550 free current ≈ 1 A; stall ≈ 8 A. 4 A gives comfortable headroom.
         */
        public static final double HOOD_HOMING_STALL_AMPS = 4.0;
        /** How long (seconds) current must exceed the threshold before homing is accepted. */
        public static final double HOOD_HOMING_STALL_DURATION_S = 0.1;

        public static final Transform3d TURRET_TO_ROBOT = new Transform3d(
            new Translation3d(Units.inchesToMeters(-7.5), Units.inchesToMeters(-6.5), Units.inchesToMeters(20)),
            new Rotation3d(0, 0, 0));
    }
    
    public static class IntakeSubsystemConstants {
        public static final int LIFT_MOTOR_ID = 47;
        public static final int INTAKE_MOTOR_ID = 46;

        public static final double LIFT_GEARBOX_RATIO = 4.0*4*4;

        public static final double LIFT_KP = 0.5;
        public static final double LIFT_KI = 0;
        public static final double LIFT_KD = 0;
    }

    public static class IndexerSubsystemConstants {
        public static final int SPINNER_MOTOR_ID = 50;
        
        public static final double SPINNER_KP = 0;
        public static final double SPINNER_KI = 0;
        public static final double SPINNER_KD = 0;
        public static final double SPINNER_KV = 0;
    }

    public static class ClimberSubsystemConstants {
        public static final int CLIMBER_MOTOR_ID = 45;

        public static final double CLIMBER_HOLD_KP = 1.0;
        public static final double CLIMBER_HOLD_KI = 0.0;
        public static final double CLIMBER_HOLD_KD = 0.0;
        public static final double CLIMBER_HOLD_KS = 0.0;

        public static final double CLIMBER_HOLD_TOLERANCE_ROTATIONS = 0.5;

        // Climber homing constants (TalonFX / Kraken X60, 64:1 gearbox, ~7 in travel).
        // Homing drives slowly downward to the lower hard stop and zeros when stall is detected.
        /** Open-loop output while homing toward the lower hard stop. Negative = retract/lower. */
        public static final double CLIMBER_HOMING_SPEED = -1;
        /**
         * Stator-current threshold (amps) above which the climber is considered stalled.
         * Kraken X60 stall current is ~200 A; 20 A provides a conservative safe threshold
         * before the mechanical stop applies significant force.
         */
        public static final double CLIMBER_HOMING_STALL_AMPS = 15.0;
        /** How long (seconds) current must exceed the threshold before homing is accepted. */
        public static final double CLIMBER_HOMING_STALL_DURATION_S = 0.1;
    }
}
