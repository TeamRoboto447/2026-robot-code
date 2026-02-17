package frc.robot.networking;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import frc.robot.Constants.IndexerSubsystemConstants;
import frc.robot.Constants.TurretSubsystemConstants;

/**
 * NetworkedConfig centralizes all NetworkTables-based configurations for the robot.
 * This class provides a single location for managing tunable parameters that can be
 * adjusted via the dashboard during development and testing.
 * 
 * Uses NetworkTables directly for better performance and organization compared to SmartDashboard.
 */
public class NetworkedConfig {
    private static final NetworkTableInstance defaultNTInstance = NetworkTableInstance.getDefault();
    
    /**
     * Turret configuration values accessible via NetworkTables.
     */
    public static class Turret {
        private static final NetworkTable turretTable = defaultNTInstance.getTable("TurretAssembly");
        private static final NetworkTable turretTargettingTable = turretTable.getSubTable("targetting");
        private static final NetworkTable hoodTable = turretTable.getSubTable("hood");
        private static final NetworkTable flywheelTable = turretTable.getSubTable("flywheel");
        private static final NetworkTable rotationTable = turretTable.getSubTable("rotation");
        
        // Shooter PID
        private static final DoubleEntry shooterKP = flywheelTable
            .getDoubleTopic("kP").getEntry(TurretSubsystemConstants.SHOOTER_KP);
        private static final DoubleEntry shooterKI = flywheelTable
            .getDoubleTopic("kI").getEntry(TurretSubsystemConstants.SHOOTER_KI);
        private static final DoubleEntry shooterKD = flywheelTable
            .getDoubleTopic("kD").getEntry(TurretSubsystemConstants.SHOOTER_KD);
        private static final DoubleEntry shooterKV = flywheelTable
            .getDoubleTopic("kV").getEntry(TurretSubsystemConstants.SHOOTER_KV);
        private static final DoubleEntry targetRPM = flywheelTable
            .getDoubleTopic("Target Flywheel RPM").getEntry(3400);
        
        // Hood PID
        private static final DoubleEntry hoodKP = hoodTable
            .getDoubleTopic("kP").getEntry(TurretSubsystemConstants.HOOD_KP);
        private static final DoubleEntry hoodKI = hoodTable
            .getDoubleTopic("kI").getEntry(TurretSubsystemConstants.HOOD_KI);
        private static final DoubleEntry hoodKD = hoodTable
            .getDoubleTopic("kD").getEntry(TurretSubsystemConstants.HOOD_KD);
        private static final DoubleEntry targetHoodAngle = hoodTable
            .getDoubleTopic("Target Hood Angle").getEntry(17);
        
        // Turret PID
        private static final DoubleEntry turretKP = rotationTable
            .getDoubleTopic("kP").getEntry(TurretSubsystemConstants.TURRET_KP);
        private static final DoubleEntry turretKI = rotationTable
            .getDoubleTopic("kI").getEntry(TurretSubsystemConstants.TURRET_KI);
        private static final DoubleEntry turretKD = rotationTable
            .getDoubleTopic("kD").getEntry(TurretSubsystemConstants.TURRET_KD);
        private static final DoubleEntry targetTurretAngle = rotationTable
            .getDoubleTopic("Target Turret Angle").getEntry(0);

        // Turret Targeting
        private static final BooleanEntry validTarget = turretTargettingTable
            .getBooleanTopic("has_valid_shot").getEntry(false);
        
        // Telemetry
        private static final DoubleEntry turretAngle = rotationTable
            .getDoubleTopic("Turret Angle").getEntry(0);
        private static final DoubleEntry hoodAngle = hoodTable
            .getDoubleTopic("Hood Angle").getEntry(0);
        private static final DoubleEntry flywheelSpeed = flywheelTable
            .getDoubleTopic("Flywheel Speed").getEntry(0);
        private static final StringEntry turretTarget = turretTable
            .getStringTopic("Turret Target").getEntry("");
        private static final StringEntry debugFieldZone = turretTable
            .getStringTopic("Debug Field Zone").getEntry("");
        

        /**
         * Initializes the NetworkTables entries for the turret subsystem.
         */
        public static void initializeDefaults() {
            // Entries have defaults declared already, simply verify they are created on initialization
            shooterKP.get();
            shooterKI.get();
            shooterKD.get();
            shooterKV.get();
            targetRPM.set(3400);
            hoodKP.get();
            hoodKI.get();
            hoodKD.get();
            targetHoodAngle.set(17);
            turretKP.get();
            turretKI.get();
            turretKD.get();
            targetTurretAngle.set(0);
            validTarget.get();
        }
        
        // Launcher
        
        /** Gets the kP for the shooter's PID. */
        public static double getShooterKP() {
            return shooterKP.get();
        }
        
        /** Gets the kI for the shooter's PID. */
        public static double getShooterKI() {
            return shooterKI.get();
        }
        
        /** Gets the kD for the shooter's PID. */
        public static double getShooterKD() {
            return shooterKD.get();
        }
        
        /** Gets the kV for the shooter's Feed-Forward. */
        public static double getShooterKV() {
            return shooterKV.get();
        }
        
        /** Gets the target RPM for the shooter. */
        public static double getTargetRPM() {
            return targetRPM.get();
        }
        
        // Hood
        /** Gets the kP for the hood's PID. */
        public static double getHoodKP() {
            return hoodKP.get();
        }
        
        /** Gets the kI for the hood's PID. */
        public static double getHoodKI() {
            return hoodKI.get();
        }
        
        /** Gets the kD for the hood's PID. */
        public static double getHoodKD() {
            return hoodKD.get();
        }
        
        /** Gets the target angle for the hood. */
        public static double getTargetHoodAngle() {
            return targetHoodAngle.get();
        }

        // Turret
        /** Gets the kP for the turret's PID. */
        public static double getTurretKP() {
            return turretKP.get();
        }

        /** Gets the kI for the turret's PID. */
        public static double getTurretKI() {
            return turretKI.get();
        }

        /** Gets the kD for the turret's PID. */
        public static double getTurretKD() {
            return turretKD.get();
        }

        /** Gets the target angle for the turret. */
        public static double getTargetTurretAngle() {
            return targetTurretAngle.get();
        };
        
        public static boolean hasValidTrajectory() {
            return validTarget.get();
        }

        // Telemetry

        /** Sends a value as the current turret angle. */
        public static void setTurretAngle(double angle) {
            turretAngle.set(angle);
        }
        
        /** Sends a value as the current hood angle. */
        public static void setHoodAngle(double angle) {
            hoodAngle.set(angle);
        }
        
        /** Sends a value as the current shooter speed. */
        public static void setFlywheelSpeed(double speed) {
            flywheelSpeed.set(speed);
        }
        
        /** Sends a value as the current turret target. */
        public static void setTurretTarget(String target) {
            turretTarget.set(target);
        }
        
        /** Sends a value as the current field zone. */
        public static void setDebugFieldZone(String zone) {
            debugFieldZone.set(zone);
        }
    }
    
    /**
     * Indexer configuration values accessible via NetworkTables.
     */
    public static class Indexer {
        private static final NetworkTable indexerTable = defaultNTInstance.getTable("Indexer");
        
        // Spinner PID
        private static final DoubleEntry spinnerKP = indexerTable
            .getDoubleTopic("Spinner kP").getEntry(IndexerSubsystemConstants.SPINNER_KP);
        private static final DoubleEntry spinnerKI = indexerTable
            .getDoubleTopic("Spinner kI").getEntry(IndexerSubsystemConstants.SPINNER_KI);
        private static final DoubleEntry spinnerKD = indexerTable
            .getDoubleTopic("Spinner kD").getEntry(IndexerSubsystemConstants.SPINNER_KD);
        private static final DoubleEntry spinnerKV = indexerTable
            .getDoubleTopic("Spinner kV").getEntry(IndexerSubsystemConstants.SPINNER_KV);
        private static final DoubleEntry targetSpeed = indexerTable
            .getDoubleTopic("Target Speed").getEntry(0.43);
        
        // Telemetry
        private static final DoubleEntry spinnerSpeed = indexerTable
            .getDoubleTopic("Spinner Speed").getEntry(0);
        
        /**
         * Initializes the NetworkTables entries for the indexer subsystem.
         */
        public static void initializeDefaults() {
            // Entries have defaults declared already, simply verify they are created on initialization
            spinnerKP.get();
            spinnerKI.get();
            spinnerKD.get();
            spinnerKV.get();
            targetSpeed.get();
        }
        
        // Spinner
        
        /** Gets the kP for the spindexer's PID. */
        public static double getSpinnerKP() {
            return spinnerKP.get();
        }

        /** Gets the kI for the spindexer's PID. */
        public static double getSpinnerKI() {
            return spinnerKI.get();
        }

        /** Gets the kD for the spindexer's PID. */
        public static double getSpinnerKD() {
            return spinnerKD.get();
        }
        
        /** Gets the kV for the spindexer's Feed-Forward. */
        public static double getSpinnerKV() {
            return spinnerKV.get();
        }
        
        /** Gets the target speed for the spindexer. */
        public static double getTargetSpeed() {
            return targetSpeed.get();
        }
        
        // Telemetry
        /** Sends a value as the current spindexer speed. */
        public static void setSpinnerSpeed(double speed) {
            spinnerSpeed.set(speed);
        }
    }
    
    /**
     * Intake configuration values accessible via NetworkTables.
     */
    public static class Intake {
        private static final NetworkTable intakeTable = defaultNTInstance.getTable("Intake");
        
        // Lift PID
        private static final DoubleEntry liftKP = intakeTable
            .getDoubleTopic("Lift kP").getEntry(0);
        private static final DoubleEntry liftKI = intakeTable
            .getDoubleTopic("Lift kI").getEntry(0);
        private static final DoubleEntry liftKD = intakeTable
            .getDoubleTopic("Lift kD").getEntry(0);
        
        // Telemetry entries
        private static final DoubleEntry intakeSpeed = intakeTable
            .getDoubleTopic("Intake Speed").getEntry(0);
        
        /**
         * Initializes the NetworkTables entries for the intake subsystem.
         */
        public static void initializeDefaults() {
            // Entries have defaults declared already, simply verify they are created on initialization
            liftKP.get();
            liftKI.get();
            liftKD.get();
        }
        
        // Lift
        
        /** Gets the kP for the lift's PID. */
        public static double getLiftKP() {
            return liftKP.get();
        }
        
        /** Gets the kI for the lift's PID. */
        public static double getLiftKI() {
            return liftKI.get();
        }
        
        /** Gets the kD for the lift's PID. */
        public static double getLiftKD() {
            return liftKD.get();
        }
        
        // Telemetry
        /** Sends a value as the current intake speed. */
        public static void setIntakeSpeed(double speed) {
            intakeSpeed.set(speed);
        }
    }

    /**
     * Climber configuration values accessible via NetworkTables.
     */
    public static class Climber {
        private static final NetworkTable climberTable = defaultNTInstance.getTable("Climber");

        // Spark MAX closed-loop PID (prepared for future implementation)
        private static final DoubleEntry climberKP = climberTable
            .getDoubleTopic("kP").getEntry(0.0);
        private static final DoubleEntry climberKI = climberTable
            .getDoubleTopic("kI").getEntry(0.0);
        private static final DoubleEntry climberKD = climberTable
            .getDoubleTopic("kD").getEntry(0.0);
        private static final DoubleEntry climberKFF = climberTable
            .getDoubleTopic("kFF").getEntry(0.0);
        private static final DoubleEntry climberIZone = climberTable
            .getDoubleTopic("iZone").getEntry(0.0);
        private static final DoubleEntry climberMaxOutput = climberTable
            .getDoubleTopic("Max Output").getEntry(1.0);

        // Target / telemetry
        private static final DoubleEntry targetPosition = climberTable
            .getDoubleTopic("Target Position").getEntry(0.0);
        private static final DoubleEntry currentPosition = climberTable
            .getDoubleTopic("Current Position").getEntry(0.0);
        // Direct open-loop output for testing (-1.0 .. 1.0)
        private static final DoubleEntry openLoopOutput = climberTable
            .getDoubleTopic("Open Loop Speed").getEntry(0.0);

        public static void initializeDefaults() {
            climberKP.get();
            climberKI.get();
            climberKD.get();
            climberKFF.get();
            climberIZone.get();
            climberMaxOutput.get();
            targetPosition.get();
            currentPosition.get();
            openLoopOutput.get();
        }

        // PID accessors (for future closed-loop control)
        public static double getClimberKP() { return climberKP.get(); }
        public static double getClimberKI() { return climberKI.get(); }
        public static double getClimberKD() { return climberKD.get(); }
        public static double getClimberKFF() { return climberKFF.get(); }
        public static double getClimberIZone() { return climberIZone.get(); }
        public static double getClimberMaxOutput() { return climberMaxOutput.get(); }

        // Target / telemetry
        public static double getTargetPosition() { return targetPosition.get(); }
        public static void setCurrentPosition(double pos) { currentPosition.set(pos); }
        public static double getOpenLoopOutput() { return openLoopOutput.get(); }
        public static void setOpenLoopOutput(double out) { openLoopOutput.set(out); }
    }

    public static class Debug {
        private static final NetworkTable debugTable = defaultNTInstance.getTable("Debug");

        private static final DoubleEntry newPoseX = debugTable.getDoubleTopic("New Pose X").getEntry(0);
        private static final DoubleEntry newPoseY = debugTable.getDoubleTopic("New Pose Y").getEntry(0);
        private static final DoubleEntry newPoseRotation = debugTable.getDoubleTopic("New Pose Rotation").getEntry(0);

        public static void initializeDefaults() {
            newPoseX.get();
            newPoseY.get();
            newPoseRotation.get();
        }
        
        public static double getNewPoseX() {
            return newPoseX.get();
        }
        
        public static double getNewPoseY() {
            return newPoseY.get();
        }
        
        public static double getNewPoseRotation() {
            return newPoseRotation.get();
        }
        
    }
    
    /**
     * Initialize all default values in NetworkTables.
     * Should be called once during robot initialization.
     */
    public static void initializeAllDefaults() {
        Turret.initializeDefaults();
        Indexer.initializeDefaults();
        Intake.initializeDefaults();
        Climber.initializeDefaults();
        Debug.initializeDefaults();
    }
}
