// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberSubsystemConstants;
import frc.robot.networking.NetworkedConfig;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberMotor;
  private int climberDir = 0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    this.climberMotor = new SparkMax(ClimberSubsystemConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    this.climberMotor.set(NetworkedConfig.Climber.getOpenLoopOutput() * this.climberDir);
  }

  public void climb() {
    this.climberDir = 1;
  }

  public void lower() {
    this.climberDir = -1;
  }

  public void stopClimber() {
    this.climberDir = 0;
  }
}
