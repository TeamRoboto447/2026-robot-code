// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerSubsystemConstants;
import frc.robot.networking.NetworkedConfig;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX spinnerMotor;

    private TalonFXConfiguration SpinnerFxConfigs = new TalonFXConfiguration();
    // private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0);

    /**
     * Creates a new IndexerSubsystem.
     */
    public IndexerSubsystem() {
        spinnerMotor = new TalonFX(IndexerSubsystemConstants.SPINNER_MOTOR_ID);

        var spinnerSlot0config = SpinnerFxConfigs.Slot0;
        spinnerSlot0config.kP = IndexerSubsystemConstants.SPINNER_KP;
        spinnerSlot0config.kI = IndexerSubsystemConstants.SPINNER_KI;
        spinnerSlot0config.kD = IndexerSubsystemConstants.SPINNER_KD;
        spinnerSlot0config.kV = IndexerSubsystemConstants.SPINNER_KV;
        spinnerSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;
        
        this.spinnerMotor.getConfigurator().apply(SpinnerFxConfigs);
    }

    /**
     * Updates the subsystem's various elements. 
     * Currently only updates the spinner velocity on the NetworkTables.
     */
    @Override
    public void periodic() {
        NetworkedConfig.Indexer.setSpinnerSpeed(this.spinnerMotor.getVelocity().getValueAsDouble()*60);
    }

    /**
     * Spins the motor in the hopper at the speed specified on the NetworkTables.
     */
    public void spin() {
        // spinnerMotor.setControl(velocityReq.withVelocity(-NetworkedConfig.Indexer.getTargetSpeed()/60));
        spinnerMotor.set(-NetworkedConfig.Indexer.getTargetSpeed());
    }

    /**
     * Stops the motor in the hopper.
     * @return A {@link Command} that stops the motor.
     */
    public Command stop() {
        return this.run(() -> spinnerMotor.set(0));
    }

    /**
     * Pulls data from the NetworkTables.
     */
    public void pullNetworkTableData() {
        var spinnerSlot0config = SpinnerFxConfigs.Slot0;
        
        spinnerSlot0config.kP = NetworkedConfig.Indexer.getSpinnerKP();
        spinnerSlot0config.kI = NetworkedConfig.Indexer.getSpinnerKI();
        spinnerSlot0config.kD = NetworkedConfig.Indexer.getSpinnerKD();
        spinnerSlot0config.kV = NetworkedConfig.Indexer.getSpinnerKV();
        spinnerSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        this.spinnerMotor.getConfigurator().apply(spinnerSlot0config);        
    }
}
