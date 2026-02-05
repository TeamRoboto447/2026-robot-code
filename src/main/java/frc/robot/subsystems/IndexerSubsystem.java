// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerSubsystemConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX spinnerMotor;
    /** Creates a new IndexerSubsystem. */
    public IndexerSubsystem() {
        spinnerMotor = new TalonFX(IndexerSubsystemConstants.SPINNER_MOTOR_ID);
    }

    @Override
    public void periodic() {}

    public void spin() {
        spinnerMotor.set(1);        // TODO: May need to be reversed/modified
    }

    public void stop() {
        spinnerMotor.set(0);
    }
}
