// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple testing subsystem that exposes a single TalonFX motor.
 * Use only for local/manual testing — do NOT add permanent robot behaviour here.
 */
public class MotorTestingSubsystem extends SubsystemBase {
  private final TalonFX testMotor;

  /** Create a new MotorTestingSubsystem. */
  public MotorTestingSubsystem() {
    // NOTE: This is a testing subsystem — use a literal CAN ID instead of Constants.
    // Pick an ID not used elsewhere on your robot.
    this.testMotor = new TalonFX(60);
  }

  /** Set the motor output as a percentage (-1.0 .. 1.0). */
  public void setPercent(double percent) {
    // clamp for safety
    if (percent > 1.0) percent = 1.0;
    if (percent < -1.0) percent = -1.0;
    testMotor.set(percent);
  }

  /** Stop the motor. */
  public void stop() {
    testMotor.set(0);
  }

  @Override
  public void periodic() {
    // no periodic work required for this simple test subsystem
  }
}
