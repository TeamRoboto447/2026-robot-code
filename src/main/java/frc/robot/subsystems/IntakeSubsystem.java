// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.networking.NetworkedConfig;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax liftMotor;
    private final SparkMax intakeMotor;
    @SuppressWarnings("unused")
    private final RelativeEncoder liftEncoder;
    private final SparkClosedLoopController liftController;

    private boolean isIntakeOut = false;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        liftMotor = new SparkMax(IntakeSubsystemConstants.LIFT_MOTOR_ID, MotorType.kBrushless);
        intakeMotor = new SparkMax(IntakeSubsystemConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        liftEncoder = liftMotor.getEncoder();
        liftController = liftMotor.getClosedLoopController();

        SparkMaxConfig liftConfig = new SparkMaxConfig();
        liftConfig.closedLoop
            .p(0)
            .i(0)
            .d(0);
        
        liftMotor.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
            liftController.setSetpoint(.25, ControlType.kPosition); // TODO: Fine-tune value
            isIntakeOut = true;
        }
    }

    public void liftIntake() {
        if (isIntakeOut) {
            liftController.setSetpoint(0, ControlType.kPosition);
            isIntakeOut = false;
        }
    }

    private void updateSmartDashboard() {
        NetworkedConfig.Intake.setIntakeSpeed(this.intakeMotor.getEncoder().getVelocity());
    }

    public void pullSmartDashboardData() {
        SparkMaxConfig liftConfig = new SparkMaxConfig();
        liftConfig.closedLoop
            .p(NetworkedConfig.Intake.getLiftKP())
            .i(NetworkedConfig.Intake.getLiftKI())
            .d(NetworkedConfig.Intake.getLiftKD());
        
        this.liftMotor.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
