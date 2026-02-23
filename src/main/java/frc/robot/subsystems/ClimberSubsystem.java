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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberSubsystemConstants;
import frc.robot.networking.NetworkedConfig;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor;
  private final PositionVoltage holdRequest = new PositionVoltage(0).withSlot(0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private double holdPosition = 0.0;
  private int climberDir = 0;
  private boolean isHomed = false;

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

    this.statorCurrentSignal = climberMotor.getStatorCurrent();
    this.statorCurrentSignal.setUpdateFrequency(20);

    this.climberMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal, statorCurrentSignal);

    if (climberDir == 1 || (climberDir == -1 && positionSignal.getValueAsDouble() > 0)) {
      // Normal open-loop drive (climb or lower).
      climberMotor.set(NetworkedConfig.Climber.getOpenLoopOutput() * climberDir);
    } else if (climberDir == 0) {
      // Hold mode — engage position-hold PID when outside tolerance.
      double error = Math.abs(holdPosition - positionSignal.getValueAsDouble());
      if (error > ClimberSubsystemConstants.CLIMBER_HOLD_TOLERANCE_ROTATIONS) {
        climberMotor.setControl(holdRequest.withPosition(holdPosition));
      } else {
        climberMotor.set(0);
      }
    }
    // climberDir == -2 means a homeClimber() command is running and owns the motor directly.
    // periodic() intentionally does nothing in that state.
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

  /**
   * Returns a {@link Command} that homes the climber by slowly retracting to the
   * lower hard stop, detecting the stall via stator current, zeroing the encoder,
   * and then engaging the position-hold at zero.
   *
   * <p>The command drives the motor at {@code CLIMBER_HOMING_SPEED} (a small
   * negative open-loop output) until the stator current exceeds
   * {@code CLIMBER_HOMING_STALL_AMPS} for at least
   * {@code CLIMBER_HOMING_STALL_DURATION_S} seconds, indicating contact with
   * the hard stop. It then zeros the motor's internal position sensor and hands
   * control back to the normal hold logic.</p>
   *
   * <p>This command requires (and therefore interrupts) this subsystem while
   * running.</p>
   *
   * @return the homing command
   */
  public Command homeClimber() {
    // A Timer local to this command instance tracks how long current has been
    // above the stall threshold. It is created inside the factory method so
    // each invocation gets its own independent timer.
    Timer stallTimer = new Timer();

    return this.runOnce(() -> {
          // Disable the normal hold loop while homing.
          climberDir = -2; // sentinel: "homing in progress"
          stallTimer.restart();
        })
        .andThen(this.run(() -> {
          // Drive slowly toward the lower hard stop.
          climberMotor.set(ClimberSubsystemConstants.CLIMBER_HOMING_SPEED);

          double amps = statorCurrentSignal.getValueAsDouble();
          if (Math.abs(amps) < ClimberSubsystemConstants.CLIMBER_HOMING_STALL_AMPS) {
            // Not yet stalled — restart the timer so it only counts
            // *continuous* time above the threshold.
            stallTimer.restart();
          }
          System.out.println(statorCurrentSignal.getValueAsDouble());
        }))
        .until(() ->
            Math.abs(statorCurrentSignal.getValueAsDouble()) >= ClimberSubsystemConstants.CLIMBER_HOMING_STALL_AMPS
            && stallTimer.hasElapsed(ClimberSubsystemConstants.CLIMBER_HOMING_STALL_DURATION_S))
        .finallyDo((interrupted) -> {
          climberMotor.set(0);
          stallTimer.stop();

          if (!interrupted) {
            // Hard stop confirmed — zero the position sensor.
            climberMotor.setPosition(0);
            holdPosition = 0.0;
            isHomed = true;
            System.out.println("Homed Climber Position");
          }
          // Return to normal hold mode (climberDir = 0).
          climberDir = 0;
        })
        .unless(() -> isHomed);
  }

  /** Returns true once the climber has been successfully homed. */
  public boolean isHomed() {
    return isHomed;
  }

  /** Returns the current climber position in motor rotations (from the cached position signal). */
  public double getPositionRotations() {
    return positionSignal.getValueAsDouble();
  }

  public Command idle() {
    return this.run(() -> {
      climberMotor.set(0);
    });
  }
}
