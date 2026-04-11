package frc.robot.networking;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.Constants.FieldConstants.FieldZone;
import frc.robot.Constants.FieldConstants.FieldZoneAreas;
import org.photonvision.PhotonCamera;

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
        private static final DoubleArrayPublisher startingCirclePub = poseTable
            .getDoubleArrayTopic("Starting Position").publish();
        
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
         * Publishes a circle at the starting position for visualization.
         * The circle is approximated using 17 points, with the last closing the circle.
         * 
         * @param pose The starting position
         * @param radiusMeters The radius of the circle in meters
         */
        public static void publishStartingCircle(Pose2d pose, double radiusMeters) {
            if (pose == null) {
                // Clear the circle if no target
                startingCirclePub.set(new double[0]);
                return;
            }
            
            // Create circle with 16 points
            int numPoints = 16;
            double[] circleArray = new double[(numPoints + 1) * 3]; // 16 poses * 3 values each
            
            for (int i = 0; i <= numPoints; i++) {
                double angle = 2 * Math.PI * i / numPoints;
                circleArray[i * 3] = pose.getX() + radiusMeters * Math.cos(angle);
                circleArray[i * 3 + 1] = pose.getY() + radiusMeters * Math.sin(angle);
                circleArray[i * 3 + 2] = 0; // rotation
            }
            
            startingCirclePub.set(circleArray);
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
        private static final StructArrayPublisher<Pose3d> detectedTagPositions = visionTable
            .getStructArrayTopic("Detected Tags", Pose3d.struct).publish();
        

        private static final PhotonCamera turretCamera = new PhotonCamera("TurretCam");
        private static final PhotonCamera climberCamera = new PhotonCamera("ClimberCam");

        /**
         * Sets whether the vision system currently has valid AprilTag detections.
         * 
         * @param hasValid True if valid AprilTags are detected, false otherwise
         */
        public static void setHasValidAprilTags(boolean hasValid) {
            hasValidAprilTags.set(hasValid);
        }

        public static void setDetectedTagPostions(List<Pose3d> positions) {
            Pose3d[] posArray = positions.toArray(new Pose3d[0]);
            detectedTagPositions.set(posArray);            
        }
        
        /**
         * Gets whether the vision system currently has valid AprilTag detections.
         * 
         * @return True if valid AprilTags are detected, false otherwise
         */
        public static boolean hasValidAprilTags() {
            return hasValidAprilTags.get();
        }
        
        /**
         * Checks if both FrontCam and BackCam are currently active.
         * Verifies both cameras by checking if their heartbeat values are changing
         * (indicating fresh frames are being processed).
         * 
         * @return True if both cameras are active and publishing new frames, false otherwise
         */
        public static boolean bothCamerasActive() {
            try {
                return turretCamera.isConnected() && climberCamera.isConnected();
            } catch (Exception e) {
                return false;
            }
        }
    }

    /**
     * Repulsor-related telemetry (operator-supplied "has piece" flag, etc.).
     */
    public static class Repulsor {
        private static final NetworkTable repTable = defaultNTInstance.getTable("Repulsor");

        private static final BooleanEntry hasPiece = repTable
            .getBooleanTopic("Has Piece").getEntry(false);

        public static void setHasPiece(boolean v) {
            hasPiece.set(v);
        }

        public static boolean hasPiece() {
            return hasPiece.get();
        }
    }

    public static class Turret {
        private static final NetworkTable turretTable = defaultNTInstance.getTable("Turret");

        private static final DoubleEntry controlTargetHoodAngle = turretTable.getDoubleTopic("Control Target Hood Angle").getEntry(0);
        private static final DoubleEntry controlTargetFlywheelRPM = turretTable.getDoubleTopic("Control Target Flywheel RPM").getEntry(0);
        private static final BooleanEntry controlTargetValidTrajectory = turretTable.getBooleanTopic("Control Target Valid Trajectory").getEntry(false);
        private static final BooleanEntry motorCommStatus = turretTable.getBooleanTopic("Motor Comm Status").getEntry(true);
        private static final StringEntry selectedSolver = turretTable.getStringTopic("Selected Solver").getEntry("NONE");
        private static final DoubleEntry lutModelHoodDeltaDeg = turretTable.getDoubleTopic("LUT-Model Hood Delta Deg").getEntry(0);
        private static final DoubleEntry lutModelRPMDelta = turretTable.getDoubleTopic("LUT-Model RPM Delta").getEntry(0);
        private static final StructArrayPublisher<Pose3d> shotArc =
            turretTable.getStructArrayTopic("Shot Arc", Pose3d.struct).publish();

        public static void setCTHoodAngle(double angle) {
            controlTargetHoodAngle.set(angle);
        }

        public static void setCTFlywheelRPM(double rpm) {
            controlTargetFlywheelRPM.set(rpm);
        }

        public static void setCTValidTrajectory(boolean valid) {
            controlTargetValidTrajectory.set(valid);
        }

        public static void setMotorCommStatus(boolean status) {
            motorCommStatus.set(status);
        }

        public static void setSelectedSolver(String solver) {
            selectedSolver.set(solver);
        }

        public static void setLutModelHoodDeltaDeg(double deltaDeg) {
            lutModelHoodDeltaDeg.set(deltaDeg);
        }

        public static void setLutModelRPMDelta(double deltaRpm) {
            lutModelRPMDelta.set(deltaRpm);
        }

        public static void setShotArc(Pose3d[] arcPoints) {
            shotArc.set(arcPoints);
        }
    }

    /**
     * Systems-check telemetry — owns all NetworkTables result entries for the
     * automated test-mode sequence. Command-building logic lives in
     * {@code frc.robot.SystemsCheck}; this class only handles NT reads/writes
     * so that the networking layer stays free of subsystem dependencies.
     */
    public static class SystemsCheck {
        private static final NetworkTable resultsTable =
            defaultNTInstance.getTable("SystemsCheck").getSubTable("results");

        public static final BooleanEntry resHoodHoming   = resultsTable.getBooleanTopic("Hood: Homing").getEntry(false);
        public static final BooleanEntry resHoodAngle    = resultsTable.getBooleanTopic("Hood: Angle Control").getEntry(false);
        public static final BooleanEntry resTurretFwd    = resultsTable.getBooleanTopic("Turret: Move Forward").getEntry(false);
        public static final BooleanEntry resTurretRev    = resultsTable.getBooleanTopic("Turret: Move Reverse").getEntry(false);
        public static final BooleanEntry resTurretReturn = resultsTable.getBooleanTopic("Turret: Return to Zero").getEntry(false);
        public static final BooleanEntry resFlywheel2k   = resultsTable.getBooleanTopic("Flywheel: 2000 RPM").getEntry(false);
        public static final BooleanEntry resFlywheel3k   = resultsTable.getBooleanTopic("Flywheel: 3000 RPM").getEntry(false);
        public static final BooleanEntry resFlywheel4k   = resultsTable.getBooleanTopic("Flywheel: 4000 RPM").getEntry(false);
        public static final BooleanEntry resFlywheel5k   = resultsTable.getBooleanTopic("Flywheel: 5000 RPM").getEntry(false);
        public static final BooleanEntry resIntakeHoming = resultsTable.getBooleanTopic("Intake: Homing").getEntry(false);
        public static final BooleanEntry resIntakeDrop   = resultsTable.getBooleanTopic("Intake: Drop").getEntry(false);
        public static final BooleanEntry resIntakeRoller = resultsTable.getBooleanTopic("Intake: Roller").getEntry(false);
        public static final BooleanEntry resIntakeLift   = resultsTable.getBooleanTopic("Intake: Lift").getEntry(false);
        public static final BooleanEntry resIndexer      = resultsTable.getBooleanTopic("Indexer: Spin").getEntry(false);
        public static final BooleanEntry resFeeder       = resultsTable.getBooleanTopic("Feeder: Run").getEntry(false);
        public static final BooleanEntry resClimberHome  = resultsTable.getBooleanTopic("Climber: Homing").getEntry(false);
        public static final BooleanEntry resClimberExt   = resultsTable.getBooleanTopic("Climber: Full Extension").getEntry(false);
        public static final BooleanEntry resClimberRet   = resultsTable.getBooleanTopic("Climber: Retract").getEntry(false);
        public static final BooleanEntry resSwerveNoFaults    = resultsTable.getBooleanTopic("Swerve: No Faults").getEntry(false);
        public static final BooleanEntry resSwerveForward     = resultsTable.getBooleanTopic("Swerve: Forward").getEntry(false);
        public static final BooleanEntry resSwerveBackward    = resultsTable.getBooleanTopic("Swerve: Backward").getEntry(false);
        public static final BooleanEntry resSwerveLeft        = resultsTable.getBooleanTopic("Swerve: Left").getEntry(false);
        public static final BooleanEntry resSwerveRight       = resultsTable.getBooleanTopic("Swerve: Right").getEntry(false);
        public static final BooleanEntry resSwerveRotateCW    = resultsTable.getBooleanTopic("Swerve: Rotate CW").getEntry(false);
        public static final BooleanEntry resSwerveRotateCCW   = resultsTable.getBooleanTopic("Swerve: Rotate CCW").getEntry(false);
        public static final StringEntry  resOverall      = resultsTable.getStringTopic("Overall").getEntry("NOT RUN");

        /** Resets result entries to {@code false} only for <em>enabled</em> systems,
         *  leaving results for skipped systems unchanged. Also sets overall to
         *  {@code "RUNNING"}.
         *
         * @param hood     whether the hood check is enabled this run
         * @param turret   whether the turret check is enabled this run
         * @param flywheel whether the flywheel check is enabled this run
         * @param intake   whether the intake check is enabled this run
         * @param indexer  whether the indexer check is enabled this run
         * @param climber  whether the climber check is enabled this run
         * @param swerve   whether the swerve check is enabled this run
         */
        public static void resetResults(
                boolean hood, boolean turret, boolean flywheel,
                boolean intake, boolean indexer, boolean feeder, boolean climber, boolean swerve) {
            if (hood)     { resHoodHoming.set(false);  resHoodAngle.set(false); }
            if (turret)   { resTurretFwd.set(false);   resTurretRev.set(false); resTurretReturn.set(false); }
            if (flywheel) { resFlywheel2k.set(false);  resFlywheel3k.set(false);
                            resFlywheel4k.set(false);  resFlywheel5k.set(false); }
            if (intake)   { resIntakeHoming.set(false); resIntakeDrop.set(false);  resIntakeRoller.set(false); resIntakeLift.set(false); }
            if (indexer)  { resIndexer.set(false); }
            if (feeder)   { resFeeder.set(false); }
            if (climber)  { resClimberHome.set(false); resClimberExt.set(false); resClimberRet.set(false); }
            if (swerve)   { resSwerveNoFaults.set(false);  resSwerveForward.set(false);
                            resSwerveBackward.set(false);   resSwerveLeft.set(false);
                            resSwerveRight.set(false);      resSwerveRotateCW.set(false);
                            resSwerveRotateCCW.set(false); }
            resOverall.set("RUNNING");
        }

        /**
         * Evaluates result entries <em>only for enabled systems</em> and sets
         * {@code Overall} to {@code "PASSED"} or {@code "FAILED"}.
         *
         * <p>Systems that were not checked in this run are excluded from the
         * evaluation entirely — a skipped system never causes a failure.
         *
         * @return {@code true} if every checked entry passed
         */
        public static boolean computeOverall(
                boolean hood, boolean turret, boolean flywheel,
                boolean intake, boolean indexer, boolean feeder, boolean climber, boolean swerve) {
            boolean allPassed = true;
            if (hood)     allPassed &= resHoodHoming.get()    && resHoodAngle.get();
            if (turret)   allPassed &= resTurretFwd.get()     && resTurretRev.get() && resTurretReturn.get();
            if (flywheel) allPassed &= resFlywheel2k.get()    && resFlywheel3k.get()
                                    && resFlywheel4k.get()    && resFlywheel5k.get();
            if (intake)   allPassed &= resIntakeHoming.get() && resIntakeDrop.get() && resIntakeRoller.get() && resIntakeLift.get();
            if (indexer)  allPassed &= resIndexer.get();
            if (feeder)   allPassed &= resFeeder.get();
            if (climber)  allPassed &= resClimberHome.get()   && resClimberExt.get() && resClimberRet.get();
            if (swerve)   allPassed &= resSwerveNoFaults.get() && resSwerveForward.get()
                                    && resSwerveBackward.get() && resSwerveLeft.get()
                                    && resSwerveRight.get()    && resSwerveRotateCW.get()
                                    && resSwerveRotateCCW.get();
            resOverall.set(allPassed ? "PASSED" : "FAILED");
            return allPassed;
        }
    }

    /**
     * Game-state telemetry — publishes match time and hub active status
     * derived from {@code GameState} via {@code StateManager}.
     *
     * <p>NT path: {@code GameState/}</p>
     */
    public static class GameState {
        private static final NetworkTable gameStateTable =
            defaultNTInstance.getTable("GameState");

        /** Remaining match time in seconds (as reported by DriverStation). */
        private static final DoubleEntry matchTime =
            gameStateTable.getDoubleTopic("Match Time").getEntry(-1.0);

        /**
         * Whether the hub is currently active (accepting fuel) for this alliance.
         * False if the game data hasn't arrived yet.
         */
        private static final BooleanEntry hubActive =
            gameStateTable.getBooleanTopic("Hub Active").getEntry(false);

        /**
         * Seconds until the hub next becomes active for this alliance.
         * 0.0 when the hub is already active or game data is unavailable.
         */
        private static final DoubleEntry hubActiveCountdown =
            gameStateTable.getDoubleTopic("Hub Active Countdown").getEntry(0.0);

        /**
         * Publishes game-state values to NetworkTables.
         *
         * @param matchTimeSecs        Remaining match time in seconds
         * @param isHubActive          Whether the hub is active for this alliance
         * @param hubActiveCountdownSecs Seconds until the hub next becomes active; 0 if already active
         */
        public static void publish(double matchTimeSecs, boolean isHubActive, double hubActiveCountdownSecs) {
            matchTime.set(matchTimeSecs);
            hubActive.set(isHubActive);
            hubActiveCountdown.set(hubActiveCountdownSecs);
        }
    }

    public static class NeoPixels {
        private static final NetworkTable neopixelTable = 
            defaultNTInstance.getTable("Neopixels");
        
        private static final StringEntry controlMode = 
            neopixelTable.getStringTopic("Control Mode").getEntry("DISABLED_NO_CAMERA");
        private static final StringEntry controlTrigger = 
            neopixelTable.getStringTopic("Control Trigger").getEntry("");
                
        public static void setControlMode(String mode)          { controlMode.set(mode); }
        public static void setControlTrigger(String trigger)    { controlTrigger.set(trigger); }
    }
}
