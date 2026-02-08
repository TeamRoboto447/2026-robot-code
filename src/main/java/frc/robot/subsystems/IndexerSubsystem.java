// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerSubsystemConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX spinnerMotor;

    private TalonFXConfiguration SpinnerFxConfigs = new TalonFXConfiguration();
    // private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0);
    /** Creates a new IndexerSubsystem. */
    public IndexerSubsystem() {
        spinnerMotor = new TalonFX(IndexerSubsystemConstants.SPINNER_MOTOR_ID);

        var spinnerSlot0config = SpinnerFxConfigs.Slot0;
        spinnerSlot0config.kP = IndexerSubsystemConstants.SPINNER_KP;
        spinnerSlot0config.kI = IndexerSubsystemConstants.SPINNER_KI;
        spinnerSlot0config.kD = IndexerSubsystemConstants.SPINNER_KD;
        spinnerSlot0config.kV = IndexerSubsystemConstants.SPINNER_KV;
        spinnerSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;
        
        this.spinnerMotor.getConfigurator().apply(SpinnerFxConfigs);

        SmartDashboard.putNumber("Indexer/Spinner kP", IndexerSubsystemConstants.SPINNER_KP);
        SmartDashboard.putNumber("Indexer/Spinner kI", IndexerSubsystemConstants.SPINNER_KI);
        SmartDashboard.putNumber("Indexer/Spinner kD", IndexerSubsystemConstants.SPINNER_KD);
        SmartDashboard.putNumber("Indexer/Spinner kV", IndexerSubsystemConstants.SPINNER_KV);
        SmartDashboard.putNumber("Indexer/Target Speed", 1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer/Spinner Speed", this.spinnerMotor.getVelocity().getValueAsDouble()*60);
    }

    public void spin() {
        // spinnerMotor.setControl(velocityReq.withVelocity(-SmartDashboard.getNumber("Indexer/Target RPM", 1000)/60));
        spinnerMotor.set(-SmartDashboard.getNumber("Indexer/Target Speed", 0));
    }

    public Command stop() {
        return this.run(() -> spinnerMotor.set(0));
    }

    public void pullSmartDashboardData() {
        var spinnerSlot0config = SpinnerFxConfigs.Slot0;
        
        spinnerSlot0config.kP = SmartDashboard.getNumber("Indexer/Spinner kP", 0);
        spinnerSlot0config.kI = SmartDashboard.getNumber("Indexer/Spinner kI", 0);
        spinnerSlot0config.kD = SmartDashboard.getNumber("Indexer/Spinner kD", 0);
        spinnerSlot0config.kV = SmartDashboard.getNumber("Indexer/Spinner kV", 0);
        spinnerSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        this.spinnerMotor.getConfigurator().apply(spinnerSlot0config);        
    }
}
