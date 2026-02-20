// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.networking.NetworkedConfig;

/**
 * A subsystem that allows control of lifting and running the intake.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax liftMotor;
    private final TalonFX intakeMotor;
    @SuppressWarnings("unused")
    private final RelativeEncoder liftEncoder;
    private final SparkClosedLoopController liftController;

    private boolean isIntakeOut = false;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        liftMotor = new SparkMax(IntakeSubsystemConstants.LIFT_MOTOR_ID, MotorType.kBrushless);
        intakeMotor = new TalonFX(IntakeSubsystemConstants.INTAKE_MOTOR_ID);

        liftEncoder = liftMotor.getEncoder();
        liftController = liftMotor.getClosedLoopController();

        SparkMaxConfig liftConfig = new SparkMaxConfig();
        liftConfig.closedLoop
            .p(0)
            .i(0)
            .d(0);
        
        liftMotor.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Updates the subsystem's various elements.
     * Currently only updates the intake speed on the NetworkTables.
     */
    @Override
    public void periodic() {
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
     * 
     * @return A {@link Command} that stops the intake.
     */
    public Command stop() {
        return this.run(() -> intakeMotor.set(0));
    }

    /**
     * Drops the intake to the extended position.
     */
    public void dropIntake() {
        if (!isIntakeOut) {
            liftController.setSetpoint(.25, ControlType.kPosition); // TODO: Fine-tune value
            isIntakeOut = true;
        }
    }

    /**
     * Lifts the intake to the starting position.
     */
    public void liftIntake() {
        if (isIntakeOut) {
            liftController.setSetpoint(0, ControlType.kPosition);
            isIntakeOut = false;
        }
    }

    /**
     * Updates the data on the NetworkTables.
     */
    private void updateNetworkTables() {
        NetworkedConfig.Intake.setIntakeSpeed(this.intakeMotor.getVelocity().getValueAsDouble());
    }

    /**
     * Pulls data from the NetworkTables.
     */
    public void pullNetworkTableData() {
        SparkMaxConfig liftConfig = new SparkMaxConfig();
        liftConfig.closedLoop
            .p(NetworkedConfig.Intake.getLiftKP())
            .i(NetworkedConfig.Intake.getLiftKI())
            .d(NetworkedConfig.Intake.getLiftKD());
        
        this.liftMotor.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
