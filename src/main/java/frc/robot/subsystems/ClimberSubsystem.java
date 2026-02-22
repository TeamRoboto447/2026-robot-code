// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberSubsystemConstants;
import frc.robot.networking.NetworkedConfig;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor;
  private final PositionVoltage holdRequest = new PositionVoltage(0).withSlot(0);
  private final StatusSignal<Angle> positionSignal;
  private double holdPosition = 0.0;
  private int climberDir = 0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    this.climberMotor = new TalonFX(ClimberSubsystemConstants.CLIMBER_MOTOR_ID);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = ClimberSubsystemConstants.CLIMBER_HOLD_KP;
    slot0.kI = ClimberSubsystemConstants.CLIMBER_HOLD_KI;
    slot0.kD = ClimberSubsystemConstants.CLIMBER_HOLD_KD;
    slot0.kS = ClimberSubsystemConstants.CLIMBER_HOLD_KS;

    this.climberMotor.getConfigurator().apply(cfg);

    this.positionSignal = climberMotor.getPosition();
    this.positionSignal.setUpdateFrequency(20);
    this.climberMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal);

    if (climberDir != 0) {
      climberMotor.set(NetworkedConfig.Climber.getOpenLoopOutput() * climberDir);
    } else {
      double error = Math.abs(holdPosition - positionSignal.getValueAsDouble());
      if (error > ClimberSubsystemConstants.CLIMBER_HOLD_TOLERANCE_ROTATIONS) {
        climberMotor.setControl(holdRequest.withPosition(holdPosition));
      } else {
        climberMotor.set(0);
      }
    }
  }

  public void climb() {
    climberDir = 1;
  }

  public void lower() {
    climberDir = -1;
  }

  public void stopClimber() {
    // Latch the current position so the hold controller has a stable target.
    holdPosition = positionSignal.getValueAsDouble();
    climberDir = 0;
  }
}
