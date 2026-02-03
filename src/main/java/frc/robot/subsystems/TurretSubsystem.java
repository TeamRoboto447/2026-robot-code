// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    // private final Field2d field;

    private final TalonFX rightShooterMotor;
    private final TalonFX leftShooterMotor;
    // private final TalonFX hoodMotor;
    // private final TalonFX angleMotor;

    public TurretTarget turretTarget = TurretTarget.NONE;
    private TalonFXConfiguration fxConfigs = new TalonFXConfiguration();
    
    /** Creates a new TurretSubsystem. */
    public TurretSubsystem(Field2d fieldImport, CommandSwerveDrivetrain sSubsystem) {
        this.swerveSubsystem = sSubsystem;
        this.lookupTable = new File(Filesystem.getDeployDirectory(), "lookup_table.json");

        SmartDashboard.putNumber("Turret/Turret kP", 0);
        SmartDashboard.putNumber("Turret/Turret kI", 0);
        SmartDashboard.putNumber("Turret/Turret kD", 0);
        SmartDashboard.putNumber("Turret/Turret kV", 0);
        SmartDashboard.putNumber("Turret/Target RPM", 3000);

        this.rightShooterMotor = new TalonFX(TurretSubsystemConstants.RIGHT_SHOOTER_MOTOR_ID);
        var slot0config = fxConfigs.Slot0;
        slot0config.kP = 0;
        slot0config.kI = 0;
        slot0config.kD = 0;
        slot0config.kV = 0;
        slot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;
        
        this.rightShooterMotor.getConfigurator().apply(fxConfigs);

        this.leftShooterMotor = new TalonFX(TurretSubsystemConstants.LEFT_SHOOTER_MOTOR_ID);

        this.leftShooterMotor.setControl(new Follower(TurretSubsystemConstants.RIGHT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
        
        // this.hoodMotor = new TalonFX(TurretSubsystemConstants.HOOD_MOTOR_ID);

        // this.angleMotor = new TalonFX(TurretSubsystemConstants.ANGLE_MOTOR_ID);

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

        prevReading = targetDist;
        prevReadingTimestamp = targetDistTimestamp;

        updateTurretTarget();
        updateSmartDashboard();
    }

    private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0);

    public void shoot() {
        double targetRPS = SmartDashboard.getNumber("Turret/Target RPM", 0) / 60;
        rightShooterMotor.setControl(velocityReq.withVelocity(targetRPS));
    }

    public Command stop() {
        return this.run(() -> this.rightShooterMotor.set(0));
    }

    // private void turnRaw(double speed) {
    //     this.angleMotor.set(speed);
    // }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Turret/Turret Angle", 0);
        SmartDashboard.putNumber("Turret/Hood Angle", 0);
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
        var slot0config = fxConfigs.Slot0;
        
        slot0config.kP = SmartDashboard.getNumber("Turret/Turret kP", 0);
        slot0config.kI = SmartDashboard.getNumber("Turret/Turret kI", 0);
        slot0config.kD = SmartDashboard.getNumber("Turret/Turret kD", 0);
        slot0config.kV = SmartDashboard.getNumber("Turret/Turret kV", 0);
        slot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        System.out.println(SmartDashboard.getNumber("Turret/Turret kP", 0));
        this.rightShooterMotor.getConfigurator().apply(fxConfigs);
    }

    public ControlTarget getControlTarget() {
        ObjectMapper mapper = new ObjectMapper();
        try {
            JsonNode lookupNode = mapper.readTree(lookupTable);
            
            Translation3d targetTranslation = getTargetFromEnum(turretTarget);
            int targetHeight = (int) targetTranslation.getZ();
            JsonNode dataNode = lookupNode
                .get(String.valueOf(targetHeight))
                .get(String.valueOf(prevReading))
                .get(String.valueOf(currentVelocityToTarget));

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
