package frc.robot.networking;

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
        private static final NetworkTable turretTable = defaultNTInstance.getTable("Turret");
        
        // Shooter PID
        private static final DoubleEntry shooterKP = turretTable
            .getDoubleTopic("Flywheel kP").getEntry(TurretSubsystemConstants.SHOOTER_KP);
        private static final DoubleEntry shooterKI = turretTable
            .getDoubleTopic("Flywheel kI").getEntry(TurretSubsystemConstants.SHOOTER_KI);
        private static final DoubleEntry shooterKD = turretTable
            .getDoubleTopic("Flywheel kD").getEntry(TurretSubsystemConstants.SHOOTER_KD);
        private static final DoubleEntry shooterKV = turretTable
            .getDoubleTopic("Flywheel kV").getEntry(TurretSubsystemConstants.SHOOTER_KV);
        private static final DoubleEntry targetRPM = turretTable
            .getDoubleTopic("Target Flywheel RPM").getEntry(3400);
        
        // Hood PID
        private static final DoubleEntry hoodKP = turretTable
            .getDoubleTopic("Hood kP").getEntry(TurretSubsystemConstants.HOOD_KP);
        private static final DoubleEntry hoodKI = turretTable
            .getDoubleTopic("Hood kI").getEntry(TurretSubsystemConstants.HOOD_KI);
        private static final DoubleEntry hoodKD = turretTable
            .getDoubleTopic("Hood kD").getEntry(TurretSubsystemConstants.HOOD_KD);
        private static final DoubleEntry targetHoodAngle = turretTable
            .getDoubleTopic("Target Hood Angle").getEntry(17);
        
        // Telemetry
        private static final DoubleEntry turretAngle = turretTable
            .getDoubleTopic("Turret Angle").getEntry(0);
        private static final DoubleEntry hoodAngle = turretTable
            .getDoubleTopic("Hood Angle").getEntry(0);
        private static final DoubleEntry turretSpeed = turretTable
            .getDoubleTopic("Turret Speed").getEntry(0);
        private static final StringEntry turretTarget = turretTable
            .getStringTopic("Turret Target").getEntry("");
        private static final StringEntry debugFieldZone = turretTable
            .getStringTopic("Debug Field Zone").getEntry("");
        

        public static void initializeDefaults() {
            // Entries have defaults declared already, simply verify they are created on initialization
            shooterKP.get();
            shooterKI.get();
            shooterKD.get();
            shooterKV.get();
            targetRPM.get();
            hoodKP.get();
            hoodKI.get();
            hoodKD.get();
            targetHoodAngle.get();
        }
        
        // Launcher
        public static double getShooterKP() {
            return shooterKP.get();
        }
        
        public static double getShooterKI() {
            return shooterKI.get();
        }
        
        public static double getShooterKD() {
            return shooterKD.get();
        }
        
        public static double getShooterKV() {
            return shooterKV.get();
        }
        
        public static double getTargetRPM() {
            return targetRPM.get();
        }
        
        // Hood
        public static double getHoodKP() {
            return hoodKP.get();
        }
        
        public static double getHoodKI() {
            return hoodKI.get();
        }
        
        public static double getHoodKD() {
            return hoodKD.get();
        }
        
        public static double getTargetHoodAngle() {
            return targetHoodAngle.get();
        }
        
        // Telemetry
        public static void setTurretAngle(double angle) {
            turretAngle.set(angle);
        }
        
        public static void setHoodAngle(double angle) {
            hoodAngle.set(angle);
        }
        
        public static void setTurretSpeed(double speed) {
            turretSpeed.set(speed);
        }
        
        public static void setTurretTarget(String target) {
            turretTarget.set(target);
        }
        
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
            .getDoubleTopic("Target Speed").getEntry(0.28);
        
        // Telemetry
        private static final DoubleEntry spinnerSpeed = indexerTable
            .getDoubleTopic("Spinner Speed").getEntry(0);
        
        public static void initializeDefaults() {
            // Entries have defaults declared already, simply verify they are created on initialization
            spinnerKP.get();
            spinnerKI.get();
            spinnerKD.get();
            spinnerKV.get();
            targetSpeed.get();
        }
        
        // Spinner
        public static double getSpinnerKP() {
            return spinnerKP.get();
        }
        
        public static double getSpinnerKI() {
            return spinnerKI.get();
        }
        
        public static double getSpinnerKD() {
            return spinnerKD.get();
        }
        
        public static double getSpinnerKV() {
            return spinnerKV.get();
        }
        
        public static double getTargetSpeed() {
            return targetSpeed.get();
        }
        
        // Telemetry
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
        
        public static void initializeDefaults() {
            // Entries have defaults declared already, simply verify they are created on initialization
            liftKP.get();
            liftKI.get();
            liftKD.get();
        }
        
        // Lift
        public static double getLiftKP() {
            return liftKP.get();
        }
        
        public static double getLiftKI() {
            return liftKI.get();
        }
        
        public static double getLiftKD() {
            return liftKD.get();
        }
        
        // Telemetry
        public static void setIntakeSpeed(double speed) {
            intakeSpeed.set(speed);
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
    }
}
