// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Objects;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.MappingJsonFactory;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.Constants.FieldConstants.FieldZone;
import frc.robot.Constants.FieldConstants.TurretTarget;
import frc.robot.Constants.FieldConstants.TurretTargetPoints;
import frc.robot.networking.NetworkedConfig;
import frc.robot.networking.NetworkedTelemetry;
import frc.robot.utils.TargettingUtils.ControlTarget;

/**
 * A subsystem that allows control of the flywheel, the hood, 
 * the turret's angle, and the kicker.
 */
public class TurretSubsystem extends SubsystemBase {
    private final PoseProvider poseProvider;
    
    private final File lookupTable;
    private double prevReading = Double.NaN;
    private double prevReadingTimestamp = Double.NaN;
    private double currentVelocityToTarget = 0;
    private ControlTarget currentControlTarget = new ControlTarget();
    @SuppressWarnings("unused")
    private boolean hoodLimitSet = false;
    private final Trigger hoodLowerLimitTrigger;
    private final Trigger feedTrigger;

    private final TalonFX rightShooterMotor;
    private final TalonFX leftShooterMotor;
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkClosedLoopController hoodController;
    private final TalonFX angleMotor;
    private final SparkMax kickerMotor;

    /** The current turret target as an enum. */
    public TurretTarget turretTarget = TurretTarget.NONE;
    private TalonFXConfiguration ShooterFxConfigs = new TalonFXConfiguration();
    private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0);

    private TalonFXConfiguration AngleFxConfigs = new TalonFXConfiguration();
    private final PositionVoltage anglePositionReq = new PositionVoltage(0).withSlot(0);
    
    /**
     * Get the trigger that indicates when the shooter is at target velocity.
     * This can be used to coordinate feeding game pieces when the shooter is ready.
     * 
     * @return Trigger that activates when shooter is at target speed
     */
    public Trigger getFeedTrigger() {
        return feedTrigger;
    }
    
    /** 
     * Creates a new TurretSubsystem.
     * 
     * @param poseProvider An object that can fetch the current robot pose
     */
    public TurretSubsystem(PoseProvider poseProvider) {
        this.poseProvider = poseProvider;
        this.lookupTable = new File(Filesystem.getDeployDirectory(), "lookup_table.json");

        this.rightShooterMotor = new TalonFX(TurretSubsystemConstants.RIGHT_SHOOTER_MOTOR_ID);
        var shooterSlot0config = ShooterFxConfigs.Slot0;
        shooterSlot0config.kP = TurretSubsystemConstants.SHOOTER_KP;
        shooterSlot0config.kI = TurretSubsystemConstants.SHOOTER_KI;
        shooterSlot0config.kD = TurretSubsystemConstants.SHOOTER_KD;
        shooterSlot0config.kV = TurretSubsystemConstants.SHOOTER_KV;
        shooterSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;
        
        this.rightShooterMotor.getConfigurator().apply(ShooterFxConfigs);

        this.leftShooterMotor = new TalonFX(TurretSubsystemConstants.LEFT_SHOOTER_MOTOR_ID);

        this.leftShooterMotor.setControl(new Follower(TurretSubsystemConstants.RIGHT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));

        this.hoodMotor = new SparkMax(TurretSubsystemConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.inverted(true);
        hoodConfig.closedLoop
            .p(TurretSubsystemConstants.HOOD_KP)
            .i(TurretSubsystemConstants.HOOD_KI)
            .d(TurretSubsystemConstants.HOOD_KD);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.hoodEncoder = hoodMotor.getEncoder();
        hoodController = this.hoodMotor.getClosedLoopController();

        hoodLowerLimitTrigger = new Trigger(() -> this.hoodMotor.getForwardLimitSwitch().isPressed());
        hoodLowerLimitTrigger.onTrue(Commands.runOnce((() -> {
            this.hoodEncoder.setPosition(0);
        }), this));

        this.angleMotor = new TalonFX(TurretSubsystemConstants.ANGLE_MOTOR_ID);

        var angleSlot0config = AngleFxConfigs.Slot0;
        angleSlot0config.kP = 0;
        angleSlot0config.kI = 0;
        angleSlot0config.kD = 0;
        angleSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        this.angleMotor.getConfigurator().apply(AngleFxConfigs);

        this.kickerMotor = new SparkMax(TurretSubsystemConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

        this.feedTrigger = new Trigger(() -> this.rightShooterMotor.getVelocity().isNear(currentControlTarget.getRPS(), 0.8));

    }
    
    /**
     * Get the current alliance from DriverStation.
     * 
     * @return The current alliance, or Alliance.Red as default if alliance is not set
     */
    private Alliance getCurrentAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }

    /**
     * Updates the subsystem's various elements, 
     * including its networktables, the current target, and the turret control data.
     */
    @Override
    public void periodic() {
        Translation2d targetFlatTranslation = getTargetFromEnum(turretTarget).toTranslation2d();
        double targetDist = targetFlatTranslation.getDistance(poseProvider.getPose().getTranslation());
        double targetDistTimestamp = Timer.getFPGATimestamp();

        if (!Double.isNaN(prevReading)) {
            double deltaDist = targetDist - prevReading;
            double deltaTime = targetDistTimestamp - prevReadingTimestamp;
            currentVelocityToTarget = deltaDist / deltaTime;
        }

        // currentControlTarget = getControlTarget();

        

        prevReading = targetDist;
        prevReadingTimestamp = targetDistTimestamp;

        updateTurretTarget();
        updateNetworkTables();
    }

    /**
     * Tells the flywheel to spin at a specific speed, taken from the control target.
     */
    public void shoot() {
        // double targetRPS = currentControlTarget.getRPS();
        double targetRPS = NetworkedConfig.Turret.getTargetRPM()/60;
        rightShooterMotor.setControl(velocityReq.withVelocity(targetRPS));
    }

    /**
     * Stops the shooter. Does not immediately stop due to flywheel momentum.
     * 
     * @return A {@link Command} that stops the shooter.
     */
    public Command stopShooter() {
        // return this.run(() -> this.rightShooterMotor.set(0));
        return this.run(() -> this.rightShooterMotor.setControl(velocityReq.withVelocity(0)));
    }

    /**
     * Stops the hood.
     * 
     * @return A {@link Command} that stops the hood.
     */
    public Command stopHood() {
        return this.run(() -> this.hoodMotor.set(0));
    }

    /**
     * Stops the kicker.
     * 
     * @return A {@link Command} that stops the kicker.
     */
    public Command stopKicker() {
        return this.run(() -> this.kickerMotor.set(0));
    }

    public Command stopTurret() {
        return this.run(() -> this.angleMotor.set(0));
    }

    /**
     * Starts the turret moving towards a new angle.
     * 
     * @param newAngle The angle to move toward.
     */
    public void turnToAngle(Angle newAngle) {
        if ((newAngle.compareTo(TurretSubsystemConstants.MAX_TURRET_ANGLE) > 0) || (newAngle.compareTo(TurretSubsystemConstants.MIN_TURRET_ANGLE) < 0)) {
            return;
        } else {
            double rotationsToAngle = newAngle
                .minus(TurretSubsystemConstants.MIN_TURRET_ANGLE)
                .div(TurretSubsystemConstants.TURRET_DEGREES_ROTATION_RATIO)
                .magnitude();
        
            angleMotor.setControl(anglePositionReq.withPosition(rotationsToAngle));
        }
    }

    /**
     * Runs the kicker motor if the flywheel is close to its target speed.
     * 
     * @param strength The strength to run the motor at, on a scale of -1 (full reverse) to 1 (full forward).
     */
    public void kick(double strength) {
        // if (rightShooterMotor.getVelocity().isNear(NetworkedConfig.Turret.getTargetRPM()/60, 0.8)) {
            kickerMotor.set(strength);
        // } else {
        //     kickerMotor.set(0);
        // }
    }

    /**
     * Starts the hood moving towards a new angle.
     * 
     * @param newAngle The new angle for the hood to move to.
     */
    public void setHoodAngle(Angle newAngle) {
        if (/*(!hoodLimitSet) || */
                (newAngle.compareTo(TurretSubsystemConstants.MAX_HOOD_ANGLE) > 0) ||
                (newAngle.compareTo(TurretSubsystemConstants.MIN_HOOD_ANGLE) < 0)) {
            return;
        } else {
            double angleToRotations = newAngle
                .minus(TurretSubsystemConstants.MIN_HOOD_ANGLE)
                .div(TurretSubsystemConstants.HOOD_DEGREES_ROTATION_RATIO)
                .magnitude();
            hoodController.setSetpoint(angleToRotations, ControlType.kPosition);
            
        }
    }

    /**
     * Updates the data posted to the NetworkTables. 
     */
    private void updateNetworkTables() {
        NetworkedConfig.Turret.setTurretAngle(TurretSubsystemConstants.MIN_TURRET_ANGLE.plus(TurretSubsystemConstants.TURRET_DEGREES_ROTATION_RATIO.times(this.angleMotor.getPosition().getValueAsDouble())).magnitude());
        NetworkedConfig.Turret.setHoodAngle(TurretSubsystemConstants.MIN_HOOD_ANGLE.plus(TurretSubsystemConstants.HOOD_DEGREES_ROTATION_RATIO.times(this.hoodEncoder.getPosition())).magnitude());
        NetworkedConfig.Turret.setTurretSpeed(this.rightShooterMotor.getVelocity().getValueAsDouble()*60);

        NetworkedConfig.Turret.setTurretTarget(this.turretTarget.toString());
        
        Translation3d targetPosition = getTargetFromEnum(this.turretTarget);
        NetworkedTelemetry.Pose.publishTargetCircle(targetPosition, Units.inchesToMeters(12));
    }

    /**
     * Updates the current turret target.
     */
    public void updateTurretTarget() {
        FieldZone currentFieldZone = this.poseProvider.getFieldZone();
        Alliance currentAlliance = getCurrentAlliance();

        NetworkedConfig.Turret.setDebugFieldZone(currentFieldZone.toString());

        switch(currentFieldZone) {
            case RED_ALLIANCE_AUDIENCE_SIDE:
            case RED_ALLIANCE_SCORING_SIDE: {
                // If in red zone and we're red alliance, aim for hub
                if (currentAlliance == Alliance.Red) {
                    this.turretTarget = TurretTarget.RED_HUB;
                } else {
                    // Blue alliance in opponent zone
                    this.turretTarget = (currentFieldZone == FieldZone.RED_ALLIANCE_AUDIENCE_SIDE) 
                        ? TurretTarget.AUDIENCE_CORNER 
                        : TurretTarget.SCORING_CORNER;
                }
                break;
            }
            case BLUE_ALLIANCE_AUDIENCE_SIDE:
            case BLUE_ALLIANCE_SCORING_SIDE: {
                // If in blue zone and we're blue alliance, aim for hub
                if (currentAlliance == Alliance.Blue) {
                    this.turretTarget = TurretTarget.BLUE_HUB;
                } else {
                    // Blue alliance in opponent zone
                    this.turretTarget = (currentFieldZone == FieldZone.BLUE_ALLIANCE_AUDIENCE_SIDE) 
                        ? TurretTarget.AUDIENCE_CORNER 
                        : TurretTarget.SCORING_CORNER;
                }
                break;
            }
            case SCORING_NEUTRAL_ZONE: {
                // In neutral zone, aim for scoring corner
                this.turretTarget = TurretTarget.SCORING_CORNER;
                break;
            }
            case AUDIENCE_NEUTRAL_ZONE: {
                // In neutral zone, aim for audience corner
                this.turretTarget = TurretTarget.AUDIENCE_CORNER;
                break;
            }
            case OUT_OF_FIELD: {
                this.turretTarget = TurretTarget.NONE;
                break;
            }
        }
    }

    /**
     * Pulls data from the NetworkTables.
     */
    public void pullNetworkTableData() {
        var shooterSlot0config = ShooterFxConfigs.Slot0;
        
        shooterSlot0config.kP = NetworkedConfig.Turret.getShooterKP();
        shooterSlot0config.kI = NetworkedConfig.Turret.getShooterKI();
        shooterSlot0config.kD = NetworkedConfig.Turret.getShooterKD();
        shooterSlot0config.kV = NetworkedConfig.Turret.getShooterKV();
        shooterSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        this.rightShooterMotor.getConfigurator().apply(ShooterFxConfigs);

        var angleSlot0config = AngleFxConfigs.Slot0;

        angleSlot0config.kP = NetworkedConfig.Turret.getTurretKP();
        angleSlot0config.kI = NetworkedConfig.Turret.getTurretKP();
        angleSlot0config.kD = NetworkedConfig.Turret.getTurretKP();
        angleSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        this.angleMotor.getConfigurator().apply(AngleFxConfigs);
        
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.inverted(true);
        hoodConfig.closedLoop
            .p(NetworkedConfig.Turret.getHoodKP())
            .i(NetworkedConfig.Turret.getHoodKI())
            .d(NetworkedConfig.Turret.getHoodKD());
        
        this.hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        System.out.println("Updated Turret PIDs.");
        
    }

    /**
     * Gets the current control target - namely the target RPM and hood angle.
     * 
     * @return A {@link ControlTarget} with the appropiate data.
     */
    public ControlTarget getControlTarget() {
        JsonFactory factory = new MappingJsonFactory();
        try (JsonParser parser = factory.createParser(lookupTable)) {
            Translation3d targetTranslation = getTargetFromEnum(turretTarget);
            int targetHeight = (int) targetTranslation.getZ();

            int steppedTargetDist = ((int) Math.round(prevReading / TurretSubsystemConstants.LOOKUP_TABLE_DIST_STEP)) * TurretSubsystemConstants.LOOKUP_TABLE_DIST_STEP;
            int steppedVelocity = ((int) Math.round(currentVelocityToTarget / TurretSubsystemConstants.LOOKUP_TABLE_VEL_STEP)) * TurretSubsystemConstants.LOOKUP_TABLE_VEL_STEP;

            JsonNode velocityNode = null;
            parser.nextToken();
            while (parser.nextToken() != null) {
                String heightName = parser.currentName();
                if (!heightName.equals(String.valueOf(targetHeight))) {
                    parser.nextToken();
                    parser.skipChildren();
                } else {
                    while (parser.nextToken() != null) {
                        String distName = parser.currentName();
                        if(!distName.equals(String.valueOf(steppedTargetDist))) {
                            parser.nextToken();
                            parser.skipChildren();
                        } else {
                            velocityNode = parser.readValueAsTree();
                        }
                    }
                }
            }
            if (Objects.isNull(velocityNode)) {
                return new ControlTarget();
            }
            JsonNode dataNode = velocityNode
                .get(String.valueOf(steppedVelocity));

            if (dataNode.isNull()) {
                return new ControlTarget();
            } else {
                int rpm = dataNode.get("rpm").asInt();
                double hoodAngle = dataNode.get("angle_deg").asDouble();
                return new ControlTarget(rpm, hoodAngle);
            }
        } catch (IOException e) {
            DriverStation.reportError("Error reading lookup table JSON: " + e.getMessage(), e.getStackTrace());
            return new ControlTarget();
        }
    }

    /**
     * Gets the coordinates of the current target from an enum.
     * 
     * @param target A {@link TurretTarget} enum to retrieve the postion from.
     * @return The target coordinates in the form of a {@link Translation3d} object.
     */
    private Translation3d getTargetFromEnum(TurretTarget target) {
        Alliance currentAlliance = getCurrentAlliance();
        
        switch (target) {
            case RED_HUB: {
                return TurretTargetPoints.RED_HUB;
            }
            case BLUE_HUB: {
                return TurretTargetPoints.BLUE_HUB;
            }
            case AUDIENCE_CORNER: {
                if (currentAlliance == Alliance.Red) {
                    return TurretTargetPoints.RED_LEFT_CORNER;
                } else {
                    return TurretTargetPoints.BLUE_RIGHT_CORNER;
                }
            }
            case SCORING_CORNER: {
                if (currentAlliance == Alliance.Red) {
                    return TurretTargetPoints.RED_RIGHT_CORNER;
                } else {
                    return TurretTargetPoints.BLUE_LEFT_CORNER;
                }
            }
            default: return new Translation3d();
        }    
    }
}
