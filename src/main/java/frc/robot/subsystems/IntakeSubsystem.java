// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.networking.NetworkedConfig;

/**
 * A subsystem that allows control of lifting and running the intake.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX liftMotor;
    private final TalonFX intakeMotor;
    
    TalonFXConfiguration liftFXConfigs = new TalonFXConfiguration();

    private boolean isIntakeOut = false;

    // Cached signal — avoids creating a new StatusSignal object every periodic() call.
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> intakeVelocitySignal;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        liftMotor = new TalonFX(IntakeSubsystemConstants.LIFT_MOTOR_ID);
        intakeMotor = new TalonFX(IntakeSubsystemConstants.INTAKE_MOTOR_ID);

        liftFXConfigs.Slot0
            .withKP(IntakeSubsystemConstants.LIFT_KP)
            .withKI(IntakeSubsystemConstants.LIFT_KI)
            .withKD(IntakeSubsystemConstants.LIFT_KD);
        
        liftMotor.getConfigurator().apply(liftFXConfigs);
        liftMotor.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(IntakeSubsystemConstants.LIFT_GEARBOX_RATIO)
        );
        liftMotor.setPosition(0);

        // Cache and configure the intake velocity signal at 20 Hz (telemetry only).
        this.intakeVelocitySignal = intakeMotor.getVelocity();
        this.intakeVelocitySignal.setUpdateFrequency(20);
        // Suppress all other status frames on motors whose other signals are not needed.
        this.intakeMotor.optimizeBusUtilization();
        this.liftMotor.optimizeBusUtilization();

    }

    /**
     * Updates the subsystem's various elements.
     * Currently only updates the intake speed on the NetworkTables.
     */
    @Override
    public void periodic() {
        // Refresh the cached signal once per loop — single CAN read for all signal data.
        BaseStatusSignal.refreshAll(intakeVelocitySignal);
        updateNetworkTables();
    }

    /**
     * Runs the intake.
     * 
     * @param speed The speed to intake at
     */
    public void intake(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * Runs the intake in reverse.
     * 
     * @param speed The speed to reverse at
     */
    public void reverseIntake(double speed) {
        intakeMotor.set(-speed);
    }

    /**
     * Stops the intake.
     */
    public void stopIntake() {
        intakeMotor.set(0);
    }

    /**
     * Stops the lifter.
     */
    public void stopLifter() {
        liftMotor.set(0);
    }

    /**
     * Drops the intake to the extended position.
     */
    public void dropIntake() {
            this.liftMotor.setControl(new PositionVoltage(NetworkedConfig.Intake.getLiftPosition())); // TODO: Fine-tune value
    }

    /**
     * Lifts the intake to the starting position.
     */
    public void liftIntake() {
        this.liftMotor.setControl(new PositionVoltage(0));
    }

    /**
     * Updates the data on the NetworkTables.
     */
    private void updateNetworkTables() {
        NetworkedConfig.Intake.setIntakeSpeed(intakeVelocitySignal.getValueAsDouble());
    }

    /**
     * Pulls data from the NetworkTables.
     */
    public void pullNetworkTableData() {
        liftFXConfigs.Slot0
            .withKP(NetworkedConfig.Intake.getLiftKP())
            .withKI(NetworkedConfig.Intake.getLiftKI())
            .withKD(NetworkedConfig.Intake.getLiftKD());
        
        this.liftMotor.getConfigurator().apply(liftFXConfigs);
    }
}
