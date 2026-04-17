package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        // How many meters away a tag needs to be before its std dev starts scaling up.
        // At this distance the multiplier is 1x; beyond it, it grows linearly.
        public static final double VISION_STD_DEV_SCALE_DISTANCE = 1.0; // meters
        // How quickly std devs grow with distance beyond VISION_STD_DEV_SCALE_DISTANCE.
        // e.g. 0.3 means +0.3 to the multiplier per extra meter.
        public static final double VISION_STD_DEV_SCALE_FACTOR = 0.5;

        public static final Transform3d ROBOT_TO_CLIMBER_CAM = new Transform3d(
        new Translation3d(Units.inchesToMeters(-10.625), Units.inchesToMeters(13.375), Units.inchesToMeters(9.25)),
        new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(142)));
        public static final Transform3d ROBOT_TO_TURRET_CAM = new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.875), Units.inchesToMeters(-10.375), Units.inchesToMeters(7.95)),
        new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(210.24)));

        // Base std devs for vision measurements. Higher = trust odometry more over vision.
        // [x (m), y (m), theta (rad)] — lower = trust vision more, higher = trust swerve more.
        // At 0.9/0.9/1.5, vision gently nudges the pose estimate rather than overriding wheel odometry.
        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(),
                0.5, 0.5, .15);

        // Near-zero std devs used while the robot is disabled so vision measurements
        // are trusted implicitly, fully seeding the pose estimator from AprilTags
        // before autonomous begins. Values are non-zero to avoid numerical issues
        // in the Kalman filter.
        public static final Matrix<N3, N1> VISION_DISABLED_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(),
                1e-4, 1e-4, 1e-4);

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
            public static final Translation3d RED_HUB = new Translation3d(11.9, 4.035, 1.83);
            public static final Translation3d RED_RIGHT_CORNER = new Translation3d(15.04, 6.57, 0);
            public static final Translation3d RED_LEFT_CORNER = new Translation3d(15.04, 1.5, 0);
            
            public static final Translation3d BLUE_HUB = new Translation3d(4.595, 4.035, 1.83);
            public static final Translation3d BLUE_RIGHT_CORNER = new Translation3d(1.5, 6.57, 0);
            public static final Translation3d BLUE_LEFT_CORNER = new Translation3d(1.5, 1.5, 0);
        }

        /**
         * Target poses the robot drives to during an automated climb, one per
         * alliance-side combination.
         *
         * <p>All coordinates use the blue-alliance origin (standard WPILib convention).
         * X/Y are in meters; rotation is the heading the robot should face when it
         * arrives so that the side-mounted hooks engage the bar correctly.
         *
         * <p>Each alliance+side has two poses:
         * <ul>
         *   <li><b>Staging</b> — where the robot stops to align before slotting onto
         *       the tower. It should be in front of (but not touching) the bar,
         *       facing the correct direction, with the climber already raised.</li>
         *   <li><b>Final</b>   — the precise position where the hooks engage the bar.
         *       The robot approaches this from the staging pose at reduced speed.</li>
         * </ul>
         *
         * <p><b>TODO: Tune all eight poses to match the actual bar locations on your field.</b>
         */
        public static class ClimbPositions {
            /**
             * Maximum drive speed (m/s) while approaching the bar.
             * Applied to the entire climb path so the robot slows down before
             * slotting onto the tower.
             */
            public static final double APPROACH_SPEED_MPS = .25;

            /**
             * How close the robot must get to the staging waypoint (meters) before
             * BLine hands off to the final bar pose. Smaller = tighter alignment
             */
            public static final double STAGING_HANDOFF_RADIUS_METERS = Units.inchesToMeters(.25);

            /**
             * How close the robot must get to the final waypoint (meters) before
             * BLine considers the path complete. Smaller = tighter alignment
             */
            public static final double FINAL_APPROACH_RADIUS_METERS = Units.inchesToMeters(.25);


            // private static final double BLUE_DEPOT_X = 1.133;
            // private static final double BLUE_OUTPOST_X = 0.9;
            private static final double BLUE_DEPOT_X = 1.1061;
            private static final double BLUE_OUTPOST_X = 1;

            // ── Final (hook-engagement) positions ──────────────────────────────
            public static final Pose2d BLUE_OUTPOST_SIDE = new Pose2d(
                BLUE_OUTPOST_X, // placeholder — tune to your bar
                3.090,  // audience side: y < field midpoint
                Rotation2d.fromDegrees(270)   // placeholder — tune to face the bar
            );
            public static final Pose2d BLUE_DEPOT_SIDE = new Pose2d(
                BLUE_DEPOT_X, // placeholder — tune to your bar
                4.30, // scoring side: y > field midpoint
                Rotation2d.fromDegrees(90)  // placeholder — tune to face the bar
            );

            // private static final double RED_DEPOT_X = 15.3021;
            private static final double RED_DEPOT_X = 15.449;
            // private static final double RED_OUTPOST_X = 15.61;
            private static final double RED_OUTPOST_X = 15.510;

            public static final Pose2d RED_DEPOT_SIDE = new Pose2d(
                RED_DEPOT_X, // placeholder — tune to your bar
                3.726,   // audience side: y < field midpoint
                Rotation2d.fromDegrees(270)    // placeholder — tune to face the bar
            );
            public static final Pose2d RED_OUTPOST_SIDE = new Pose2d(
                RED_OUTPOST_X, // placeholder — tune to your bar
                4.90,  // scoring side: y > field midpoint
                Rotation2d.fromDegrees(90)   // placeholder — tune to face the bar
            );

            
            // ── Staging (pre-alignment) positions ──────────────────────────────
            // These should be positioned a short distance back from the final poses,
            // with the same heading, so the robot can align before slotting in.
            public static final Pose2d BLUE_OUTPOST_SIDE_STAGING = new Pose2d(
                BLUE_OUTPOST_X, // TODO: tune — offset from final pose
                2,
                Rotation2d.fromDegrees(270)
            );
            public static final Pose2d BLUE_DEPOT_SIDE_STAGING = new Pose2d(
                BLUE_DEPOT_X, // TODO: tune — offset from final pose
                5.5,
                Rotation2d.fromDegrees(90)
            );
            public static final Pose2d RED_DEPOT_SIDE_STAGING = new Pose2d(
                RED_DEPOT_X, // TODO: tune — offset from final pose
                2.8,
                Rotation2d.fromDegrees(270)
            );
            public static final Pose2d RED_OUTPOST_SIDE_STAGING = new Pose2d(
                RED_OUTPOST_X, // TODO: tune — offset from final pose
                5.75,
                Rotation2d.fromDegrees(90)
            );
        }

        public static final Translation2d BLUE_TOWER_CENTER = new Translation2d(
            1,
            3.75
        );
        
        public static final Translation2d RED_TOWER_CENTER = new Translation2d(
            15.55,
            4.3
        );

        /**
         * Rectangles in which Theseus drops the hood to protect the turret from breaking off.
         * 
         * <p><b>TODO: Values possibly need tuning to allow us to get closer to the trench without colliding.</b>
         */
        public static class TurretSafety {

            public static final Rectangle2d BLUE_DEPOT_SIDE_TRENCH = new Rectangle2d(
                new Translation2d(Units.inchesToMeters(158.06), Units.inchesToMeters(0)),
                new Translation2d(Units.inchesToMeters(205.06), Units.inchesToMeters(49.84))
            );

            public static final Rectangle2d BLUE_OUTPOST_SIDE_TRENCH = new Rectangle2d(
                new Translation2d(Units.inchesToMeters(158.06), Units.inchesToMeters(266.68)),
                new Translation2d(Units.inchesToMeters(205.06), FIELD_WIDTH_METERS)
            );

            public static final Rectangle2d RED_DEPOT_SIDE_TRENCH = new Rectangle2d(
                new Translation2d(Units.inchesToMeters(445.06), Units.inchesToMeters(266.68)),
                new Translation2d(Units.inchesToMeters(492.06), FIELD_WIDTH_METERS) //492.06
            );

            public static final Rectangle2d RED_OUTPOST_SIDE_TRENCH = new Rectangle2d(
                new Translation2d(Units.inchesToMeters(445.06), Units.inchesToMeters(0)),
                new Translation2d(Units.inchesToMeters(492.06), Units.inchesToMeters(49.84))
            );
        }
        
        /**
         * Tolerance for aligning robot to autonomous starting position.
         * Used during disabled mode to show if robot is positioned correctly for auto.
         */
        public static final double AUTO_POSE_DISTANCE_TOLERANCE_M = 0.5;
        public static final double AUTO_POSE_HEADING_TOLERANCE_DEG = 10.0;
    }

    public static class TurretSubsystemConstants {
        public static final int LEFT_SHOOTER_MOTOR_ID = 40;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 41;
        public static final int HOOD_MOTOR_ID = 42;
        public static final int ANGLE_MOTOR_ID = 43;
        public static final int KICKER_MOTOR_ID = 44;

        public static final double SHOOTER_KP = 2;
        public static final double SHOOTER_KI = 0;
        public static final double SHOOTER_KD = 0;
        public static final double SHOOTER_KV = 0;

        public static final double HOOD_KP = 0.05;
        public static final double HOOD_KI = 0;
        public static final double HOOD_KD = 0;
        public static final int HOOD_CURRENT_LIMIT_AMPS = 30;
        
        public static final double TURRET_KP = 80;
        public static final double TURRET_KD = 0.5;
        public static final double TURRET_KI = 0;
        public static final double TURRET_KS = 0.15;

        /**
         * Feed-forward gain for robot-rotation compensation (degrees of turret offset
         * per degree-per-second of robot yaw rate). When the robot rotates, the turret
         * target angle is shifted by {@code omega_deg_per_s * TURRET_ROTATION_FF} so
         * the turret leads the motion rather than lagging behind.
         * Start at 1.0 (one-to-one compensation) and tune by watching turret lag
         * during a spin: increase if still lagging, decrease if it overshoots.
         */
        public static final double TURRET_ROTATION_FF = 0.0;

        /**
         * How close (in RPS) the flywheel must be to its target speed before the
         * kicker is allowed to run. At 50 RPS (~3000 RPM) this is ±3%, which is
         * tight enough to ensure a consistent shot without being unreachably precise.
         * Increase if the kicker rarely fires; decrease if shot consistency is poor.
         */
        public static final double FLYWHEEL_READY_TOLERANCE_RPS = 250.0 / 60.0; // 50 RPM tolerance / 60 seconds = RPS

        public static final int LOOKUP_TABLE_VEL_STEP = 1;
        public static final int LOOKUP_TABLE_DIST_STEP = 1;

        public static final Angle MIN_HOOD_ANGLE = Degrees.of(23);
        public static final Angle MAX_HOOD_ANGLE = Degrees.of(40);
        public static final Angle HOOD_DEGREES_ROTATION_RATIO = Degrees.of(1);

        public static final Angle MIN_TURRET_ANGLE = Degrees.of(-190);
        public static final Angle MAX_TURRET_ANGLE = Degrees.of(170);

        public static final double TURRET_GEAR_RATIO = 29.17;
        public static final double TURRET_ANGLE_TOLERANCE_DEGREES = 0.25;
        public static final double HOOD_ANGLE_TOLERANCE_DEGREES = 0.25;

        
        /** Open-loop output used while homing toward the lower hard stop. Negative = lower. */
        public static final double HOOD_HOMING_SPEED = -0.5;
        /**
         * Output-current threshold (amps) above which the NEO 550 is considered stalled.
         * NEO 550 free current ≈ 1 A; stall ≈ 8 A. 4 A gives comfortable headroom.
         */
        public static final double HOOD_HOMING_STALL_AMPS = 2.0;
        /** How long (seconds) current must exceed the threshold before homing is accepted. */
        public static final double HOOD_HOMING_STALL_DURATION_S = 0.1;

        public static final Transform3d TURRET_TO_ROBOT = new Transform3d(
            new Translation3d(Units.inchesToMeters(-7.5), Units.inchesToMeters(-6.5), Units.inchesToMeters(20)),
            new Rotation3d(0, 0, 0));

        /**
         * Maximum drive speed (m/s) enforced while the robot is actively attempting
         * a shoot-on-the-fly shot.
         *
         * <p>At higher speeds the Newton TOF-recursion correction and latency
         * compensation become less reliable and the flywheel slip factor increases,
         * so shots become inconsistent.  When the shoot button (or {@code autoShootTrigger})
         * is active, the drive default command caps the requested velocity magnitude
         * at this value so the driver can still manoeuvre but cannot accidentally
         * outrun the shooter's reliable operating envelope.
         *
         * <p>Set to {@code MaxSpeed} (the robot's normal top speed) to disable the cap.
         * Testing showed 1.5 m/s as the upper bound for reliable SOTF shots.
         */
        public static final double SOTF_MAX_DRIVE_SPEED_MPS = .8;

        /**
         * Maximum angular rate (rad/s) enforced while the robot is actively
         * attempting a shoot-on-the-fly shot.
         *
         * <p>This limits spin speed during shooting so turret/shot compensation
         * remains stable under driver rotation input.
         */
        public static final double SOTF_MAX_ANGULAR_RATE_RAD_PER_SEC = Units.rotationsToRadians(0.25);

        /**
         * Duration (seconds) over which the drive speed ramps back to full speed
         * after the shoot button is released.
         *
         * <p>When shooting stops, the speed cap is linearly interpolated from
         * {@link #SOTF_MAX_DRIVE_SPEED_MPS} back to the robot's maximum speed
         * over this duration. This provides a smooth transition that prevents
         * the driver from suddenly regaining control and losing handling authority.
         */
        public static final double SOTF_SPEED_RAMP_TIME_S = 0.5;

        /**
         * Latency compensation for shoot-on-the-fly (seconds).
         *
         * <p>The aim-point calculation uses the robot's current pose and velocity, but
         * by the time the ball actually leaves the robot, the robot has moved.  This
         * constant projects the turret-pivot position forward in time so the virtual
         * target is computed from where the robot <em>will be</em> when the ball departs,
         * not where it <em>was</em> when the sensor frame was captured.
         *
         * <p>It accounts for the sum of all pipeline delays:
         * <ul>
         *   <li>Camera frame capture → coprocessor processing (PhotonVision): ~10–20 ms</li>
         *   <li>NT round-trip (coprocessor → RoboRIO): ~5–10 ms</li>
         *   <li>Robot loop period: up to 20 ms</li>
         *   <li>Turret PID settling + motor response: ~20–40 ms</li>
         *   <li>Ball travel through the indexer/shooter: ~20–40 ms</li>
         * </ul>
         * Total is typically 75–130 ms.  Starting value of 0.1 s (100 ms) is a
         * reasonable first guess.
         *
         * <p><b>Tuning procedure:</b> Drive in a straight line perpendicular to the
         * target at constant speed and watch where shots land.  If shots land
         * consistently in the direction you came from (behind your path), increase
         * this value.  If they land consistently ahead of your path, decrease it.
         */
        public static final double SOTF_LATENCY_COMPENSATION_S = -0.02;

        /**
         * Additional radial (toward/away) latency compensation for shoot-on-the-fly
         * distance solving (seconds).
         *
         * <p>{@link #SOTF_LATENCY_COMPENSATION_S} projects the full turret position for
         * lateral/bearing lead. This constant independently adjusts only the
         * <em>range</em> component (distance to target) so toward/away misses can be
         * tuned without disturbing the perpendicular lead that is already correct.
         *
         * <p><b>Tuning guide:</b>
         * <ul>
         *   <li>If driving toward the target shoots long and driving away shoots short,
         *       decrease this value.</li>
         *   <li>If driving toward shoots short and driving away shoots long,
         *       increase this value.</li>
         * </ul>
         */
        public static final double SOTF_RANGE_LATENCY_COMPENSATION_S = -0.02;

        /** Stator current limit (A) for each flywheel motor (Kraken X60). */
        public static final double SHOOTER_STATOR_CURRENT_LIMIT_A = 80.0;
        /** Supply current limit (A) for each flywheel motor. */
        public static final double SHOOTER_SUPPLY_CURRENT_LIMIT_A = 60.0;

        /** Stator current limit (A) for the turret angle motor (Kraken X60). */
        public static final double TURRET_STATOR_CURRENT_LIMIT_A = 40.0;
        /** Supply current limit (A) for the turret angle motor. */
        public static final double TURRET_SUPPLY_CURRENT_LIMIT_A = 30.0;

        // 12 inches of distance offset
        public static final double RPM_OFFSET_WHILE_CLIMBED = -200;
    }
    
    public static class IntakeSubsystemConstants {
        public static final int LIFT_MOTOR_ID = 47;
        public static final int INTAKE_MOTOR_ID = 46;

        public static final double LIFT_GEARBOX_RATIO = 4.0*4*4;

        public static final double LIFT_KP = 3;
        public static final double LIFT_KI = 0;
        public static final double LIFT_KD = 0;

        /**
         * Rotations (output shaft) the lift travels from the stowed position (0)
         * to the fully-lowered intake position. Negative because the motor must
         * turn in the negative direction to lower.
         */
        public static final double LIFT_LOWERED_ROTATIONS = -34;

        /**
         * Tolerance (rotations) used by {@code isIntakeDown()} and {@code isIntakeUp()}
         * when comparing the lift's actual position against the target endpoints.
         */
        public static final double LIFT_POSITION_TOLERANCE_ROTATIONS = .5;

        /** Open-loop output used while homing toward the upper hard stop. Positive = raise. */
        public static final double LIFT_HOMING_SPEED = 0.1;
        /**
         * Stator-current threshold (amps) above which the lift motor is considered stalled.
         * Kraken/Falcon stall is ~200 A; 15 A gives a conservative threshold against the
         * upper hard stop at the slow homing speed.
         */
        public static final double LIFT_HOMING_STALL_AMPS = 3.0;
        /** How long (seconds) current must exceed the threshold before homing is accepted. */
        public static final double LIFT_HOMING_STALL_DURATION_S = 0.1;

        /** Stator current limit (A) for the intake roller motor (Kraken X60). */
        public static final double INTAKE_STATOR_CURRENT_LIMIT_A = 60.0;
        /** Supply current limit (A) for the intake roller motor. */
        public static final double INTAKE_SUPPLY_CURRENT_LIMIT_A = 40.0;

        /** Stator current limit (A) for the lift motor (Kraken X60). */
        public static final double LIFT_STATOR_CURRENT_LIMIT_A = 40.0;
        /** Supply current limit (A) for the lift motor. */
        public static final double LIFT_SUPPLY_CURRENT_LIMIT_A = 30.0;
    }

    public static class IndexerSubsystemConstants {
        public static final int SPINNER_MOTOR_ID = 50;
        
        public static final double SPINNER_KP = 0;
        public static final double SPINNER_KI = 0;
        public static final double SPINNER_KD = 0;
        public static final double SPINNER_KV = 0;

        /** Stator current limit (A) for the indexer spinner motor (Kraken X60). */
        public static final double SPINNER_STATOR_CURRENT_LIMIT_A = 160.0;
        /** Supply current limit (A) for the indexer spinner motor. */
        public static final double SPINNER_SUPPLY_CURRENT_LIMIT_A = 80.0;
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
         * Winch drum radius (inches). Used to convert linear travel to motor rotations.
         * Full extension = CLIMBER_TRAVEL_INCHES / (2π × CLIMBER_DRUM_RADIUS_INCHES) × CLIMBER_GEARBOX_RATIO
         * Tune this if the climber overshoots or undershoots during the systems check.
         */
        public static final double CLIMBER_DRUM_RADIUS_INCHES = 0.25;
        public static final double CLIMBER_GEARBOX_RATIO = 64.0;
        public static final double CLIMBER_TRAVEL_INCHES = 7.0;

        /**
         * Motor rotations required to reach full extension.
         * = travel / circumference × gear ratio
         */
        public static final double CLIMBER_FULL_EXTENSION_ROTATIONS = 212.38;

        /** Stator current limit (A) for the climber motor (Kraken X60). */
        public static final double CLIMBER_STATOR_CURRENT_LIMIT_A = 120.0;
        /** Supply current limit (A) for the climber motor. */
        public static final double CLIMBER_SUPPLY_CURRENT_LIMIT_A = 80.0;
    }

    public static class NeopixelConstants {
        public enum PixelStates {
            DISABLED_NO_CAMERA,
            DISABLED_NO_TAGS,
            DISABLED_HAS_TAGS,
            DISABLED_CORRECT_POSITION,
            ENABLED_DEFAULT            
        }
    }
}
