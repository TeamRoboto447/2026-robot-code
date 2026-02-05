// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX liftMotor;
    private final TalonFX intakeMotor;

    private boolean isIntakeOut = false;

    private TalonFXConfiguration LiftFxConfigs = new TalonFXConfiguration();
    private final PositionVoltage positionReq = new PositionVoltage(0).withSlot(0);

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        liftMotor = new TalonFX(IntakeSubsystemConstants.LIFT_MOTOR_ID);
        intakeMotor = new TalonFX(IntakeSubsystemConstants.INTAKE_MOTOR_ID);

        var liftSlot0config = LiftFxConfigs.Slot0;
        liftSlot0config.kP = 0;
        liftSlot0config.kI = 0;
        liftSlot0config.kD = 0;
        liftSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        liftMotor.getConfigurator().apply(LiftFxConfigs);

        SmartDashboard.putNumber("Intake/Lift kP", 0);
        SmartDashboard.putNumber("Intake/Lift kI", 0);
        SmartDashboard.putNumber("Intake/Lift kD", 0);
    }

    @Override
    public void periodic() {
        updateSmartDashboard();
    }

    public void intake() {
        intakeMotor.set(1);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public void dropIntake() {
        if (!isIntakeOut) {
            liftMotor.setControl(positionReq.withPosition(.25));        // TODO: Fine-tune value
        }
    }

    public void liftIntake() {
        if (isIntakeOut) {
            liftMotor.setControl(positionReq.withPosition(0));
        }
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Intake/Intake Speed", this.intakeMotor.getVelocity().getValueAsDouble()*60);
    }

    public void pullSmartDashboardData() {
        var liftSlot0config = LiftFxConfigs.Slot0;
        liftSlot0config.kP = SmartDashboard.getNumber("Intake/Lift kP", 0);
        liftSlot0config.kI = SmartDashboard.getNumber("Intake/Lift kI", 0);
        liftSlot0config.kD = SmartDashboard.getNumber("Intake/Lift kD", 0);
        liftSlot0config.GainSchedBehavior = GainSchedBehaviorValue.UseSlot0;

        this.liftMotor.getConfigurator().apply(LiftFxConfigs);
    }
}
