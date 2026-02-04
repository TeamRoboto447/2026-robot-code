// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DigitalInput;
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
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.Constants.FieldConstants.FieldZone;
import frc.robot.Constants.FieldConstants.TurretTarget;
import frc.robot.Constants.FieldConstants.TurretTargetPoints;
import frc.robot.utils.TargettingUtils.ControlTarget;

public class TurretSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain swerveSubsystem;
    
    private final File lookupTable;
    private double prevReading = Double.NaN;
    private double prevReadingTimestamp = Double.NaN;
    private double currentVelocityToTarget = 0;
    private ControlTarget currentControlTarget = new ControlTarget();
    private MutAngle currentHoodAngle;
    private boolean hoodLimitSet = false;
    private final Trigger hoodLowerLimitTrigger;

    // private final Field2d field;

    private final TalonFX rightShooterMotor;
    private final TalonFX leftShooterMotor;
    private final TalonFX hoodMotor;
    private final DigitalInput hoodLowerLimitSwitch; 
    // private final TalonFX angleMotor;
    private final TalonFX kickerMotor;

    public TurretTarget turretTarget = TurretTarget.NONE;
    private TalonFXConfiguration ShooterFxConfigs = new TalonFXConfiguration();
    private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0);

    private TalonFXConfiguration HoodFxConfigs = new TalonFXConfiguration();
    private final PositionVoltage positionReq = new PositionVoltage(0).withSlot(0);

    // private 
    
    /** Creates a new TurretSubsystem. */
    public TurretSubsystem(Field2d fieldImport, CommandSwerveDrivetrain sSubsystem) {
        this.swerveSubsystem = sSubsystem;
        this.lookupTable = new File(Filesystem.getDeployDirectory(), "lookup_table.json");

        SmartDashboard.putNumber("Turret/Turret kP", 0);
        SmartDashboard.putNumber("Turret/Turret kI", 0);
        SmartDashboard.putNumber("Turret/Turret kD", 0);
        SmartDashboard.putNumber("Turret/Turret kV", 0);
        SmartDashboard.putNumber("Turret/Target RPM", 3000);

        SmartDashboard.putNumber("Turret/Hood kP", 0);
        SmartDashboard.putNumber("Turret/Hood kI", 0);
        SmartDashboard.putNumber("Turret/Hood kD", 0);

        this.rightShooterMotor = new TalonFX(TurretSubsystemConstants.RIGHT_SHOOTER_MOTOR_ID);
        var shooterSlot0config = ShooterFxConfigs.Slot0;
        shooterSlot0config.kP = 0;
        shooterSlot0config.kI = 0;
        shooterSlot0config.kD = 0;
        shooterSlot0config.kV = 0;
        shooterSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;
        
        this.rightShooterMotor.getConfigurator().apply(ShooterFxConfigs);

        this.leftShooterMotor = new TalonFX(TurretSubsystemConstants.LEFT_SHOOTER_MOTOR_ID);

        this.leftShooterMotor.setControl(new Follower(TurretSubsystemConstants.RIGHT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
        
        this.hoodMotor = new TalonFX(TurretSubsystemConstants.HOOD_MOTOR_ID);
        var hoodSlot0config = HoodFxConfigs.Slot0;
        hoodSlot0config.kP = 0;
        hoodSlot0config.kI = 0;
        hoodSlot0config.kD = 0;
        hoodSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        this.hoodMotor.getConfigurator().apply(HoodFxConfigs);

        this.hoodLowerLimitSwitch = new DigitalInput(0);

        hoodLowerLimitTrigger = new Trigger(() -> hoodLowerLimitSwitch.get());
        hoodLowerLimitTrigger.onTrue(Commands.runOnce((() -> {
            this.currentHoodAngle = Degrees.mutable(17);
            this.hoodMotor.setPosition(0);
        }), this));

        // this.angleMotor = new TalonFX(TurretSubsystemConstants.ANGLE_MOTOR_ID);

        this.kickerMotor = new TalonFX(TurretSubsystemConstants.KICKER_MOTOR_ID);

    }
    public void periodic() {
        Translation2d targetFlatTranslation = getTargetFromEnum(turretTarget).toTranslation2d();
        double targetDist = targetFlatTranslation.getDistance(swerveSubsystem.getPose().getTranslation());
        double targetDistTimestamp = Timer.getFPGATimestamp();

        if (prevReading != Double.NaN) {
            double deltaDist = targetDist - prevReading;
            double deltaTime = targetDistTimestamp - prevReadingTimestamp;
            currentVelocityToTarget = deltaDist / deltaTime;
        }

        currentControlTarget = getControlTarget();

        

        prevReading = targetDist;
        prevReadingTimestamp = targetDistTimestamp;

        updateTurretTarget();
        updateSmartDashboard();
    }

    public void shoot() {
        double targetRPS = currentControlTarget.getRPS();
        rightShooterMotor.setControl(velocityReq.withVelocity(targetRPS));
    }

    public Command stop() {
        return this.run(() -> this.rightShooterMotor.set(0));
    }

    // private void turnRaw(double speed) {
    //     this.angleMotor.set(speed);
    // }

    public void kick(double strength) {
        if (rightShooterMotor.getVelocity().isNear(currentControlTarget.getRPS(), 0.8)) {
            kickerMotor.set(strength);
        } else {
            kickerMotor.set(0);
        }
    }

    public void setHoodAngle(Angle newAngle) {
        if ((!hoodLimitSet) ||
                (newAngle.compareTo(TurretSubsystemConstants.MAX_HOOD_ANGLE) > 0) ||
                (newAngle.compareTo(TurretSubsystemConstants.MIN_HOOD_ANGLE) < 0)) {
            return;
        } else {
            double rotationsToAngle = newAngle
                .minus(TurretSubsystemConstants.MIN_HOOD_ANGLE)
                .div(TurretSubsystemConstants.HOOD_DEGREES_ROTATION_RATIO)
                .magnitude();
            hoodMotor.setControl(positionReq.withPosition(rotationsToAngle));
        }
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Turret/Turret Angle", 0);
        SmartDashboard.putNumber("Turret/Hood Angle", currentHoodAngle.magnitude());
        SmartDashboard.putNumber("Turret/Turret Speed", this.rightShooterMotor.getVelocity().getValueAsDouble()*60);

        SmartDashboard.putString("Turret/Turret Target", this.turretTarget.toString());        
    }

    public void updateTurretTarget() {
        FieldZone currentFieldZone = this.swerveSubsystem.getFieldZone();

        SmartDashboard.putString("Turret/Debug Field Zone", currentFieldZone.toString());

        switch(currentFieldZone) {
            case RED_ALLIANCE_ZONE: {
                this.turretTarget = TurretTarget.RED_HUB;
            }
            case BLUE_ALLIANCE_ZONE: {
                this.turretTarget = TurretTarget.BLUE_HUB;
            }
            case SCORING_NEUTRAL_ZONE: {
                this.turretTarget = TurretTarget.SCORING_CORNER;
            }
            case AUDIENCE_NEUTRAL_ZONE: {
                this.turretTarget = TurretTarget.AUDIENCE_CORNER;
            }
            case OUT_OF_FIELD: {
                this.turretTarget = TurretTarget.NONE;
            }
        }
    }

    public void pullSmartDashboardData() {
        var shooterSlot0config = ShooterFxConfigs.Slot0;
        
        shooterSlot0config.kP = SmartDashboard.getNumber("Turret/Turret kP", 0);
        shooterSlot0config.kI = SmartDashboard.getNumber("Turret/Turret kI", 0);
        shooterSlot0config.kD = SmartDashboard.getNumber("Turret/Turret kD", 0);
        shooterSlot0config.kV = SmartDashboard.getNumber("Turret/Turret kV", 0);
        shooterSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        System.out.println(SmartDashboard.getNumber("Turret/Turret kP", 0));
        this.rightShooterMotor.getConfigurator().apply(ShooterFxConfigs);
        
        var hoodSlot0config = HoodFxConfigs.Slot0;
        hoodSlot0config.kP = SmartDashboard.getNumber("Turret/Hood kP", 0);
        hoodSlot0config.kI = SmartDashboard.getNumber("Turret/Hood kI", 0);
        hoodSlot0config.kD = SmartDashboard.getNumber("Turret/Hood kD", 0);
        hoodSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        this.hoodMotor.getConfigurator().apply(hoodSlot0config);
        
    }

    public ControlTarget getControlTarget() {
        ObjectMapper mapper = new ObjectMapper();
        try {
            JsonNode lookupNode = mapper.readTree(lookupTable);
            
            Translation3d targetTranslation = getTargetFromEnum(turretTarget);
            int targetHeight = (int) targetTranslation.getZ();

            int steppedTargetDist = ((int) Math.round(prevReading / TurretSubsystemConstants.LOOKUP_TABLE_DIST_STEP)) * TurretSubsystemConstants.LOOKUP_TABLE_DIST_STEP;
            int steppedVelocity = ((int) Math.round(currentVelocityToTarget / TurretSubsystemConstants.LOOKUP_TABLE_VEL_STEP)) * TurretSubsystemConstants.LOOKUP_TABLE_VEL_STEP;

            JsonNode dataNode = lookupNode
                .get(String.valueOf(targetHeight))
                .get(String.valueOf(steppedTargetDist))
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

    private Translation3d getTargetFromEnum(TurretTarget target) {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        Alliance currentAlliance;
        if (optionalAlliance.isPresent()) {
            currentAlliance = optionalAlliance.get();
        } else {
            currentAlliance = Alliance.Red;
        }
        
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
