// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberSubsystemConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.networking.NetworkedConfig;

public class ClimberSubsystem extends SubsystemBase {
  @SuppressWarnings("unused")
  private final PoseProvider poseProvider;
  private final TalonFX climberMotor;
  private final DigitalInput climberLimitSwitch;
  private final PositionTorqueCurrentFOC holdRequest = new PositionTorqueCurrentFOC(0).withSlot(0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private double holdPosition = 0.0;
  private int climberDir = 0;
  private boolean isHomed = false;

  
  public final Trigger withinSafeClimberRange;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(PoseProvider poseProvider) {
    this.poseProvider = poseProvider;
    this.withinSafeClimberRange = new Trigger(() -> {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        double distanceToAllianceTower;
        if (alliance.equals(Alliance.Blue)) {
            distanceToAllianceTower = FieldConstants.RED_TOWER_CENTER.getDistance(poseProvider.getPose().getTranslation());
        } else {
            distanceToAllianceTower = FieldConstants.RED_TOWER_CENTER.getDistance(poseProvider.getPose().getTranslation());
        }
        return (distanceToAllianceTower < 2);
    });

    this.climberMotor = new TalonFX(ClimberSubsystemConstants.CLIMBER_MOTOR_ID);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    cfg.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(ClimberSubsystemConstants.CLIMBER_STATOR_CURRENT_LIMIT_A)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(ClimberSubsystemConstants.CLIMBER_SUPPLY_CURRENT_LIMIT_A)
        .withSupplyCurrentLimitEnable(true);

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = ClimberSubsystemConstants.CLIMBER_HOLD_KP;
    slot0.kI = ClimberSubsystemConstants.CLIMBER_HOLD_KI;
    slot0.kD = ClimberSubsystemConstants.CLIMBER_HOLD_KD;
    slot0.kS = ClimberSubsystemConstants.CLIMBER_HOLD_KS;

    this.climberMotor.getConfigurator().apply(cfg);

    this.climberLimitSwitch = new DigitalInput(0);

    this.positionSignal = climberMotor.getPosition();
    this.positionSignal.setUpdateFrequency(20);

    this.statorCurrentSignal = climberMotor.getStatorCurrent();
    this.statorCurrentSignal.setUpdateFrequency(20);

    this.climberMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal, statorCurrentSignal);

    boolean safeRange = withinSafeClimberRange.getAsBoolean();
    //  if (!safeRange && climberMotor.getPosition().getValueAsDouble() > 0) {
    //   // Lower if outside safe range
    //   climberMotor.set(NetworkedConfig.Climber.getOpenLoopOutput() * -1);

    // } else 
    if (safeRange && (climberDir == 1 || climberDir == -1)) {
      // Normal open-loop drive (climb or lower).
      if (climberDir == -1 && !climberLimitSwitch.get()) {
        climberMotor.set(0);
      } else {
        climberMotor.set(NetworkedConfig.Climber.getOpenLoopOutput() * climberDir);
      }

    } else if (climberDir == 0) {
      // Hold mode — engage position-hold PID when outside tolerance.
      double error = Math.abs(holdPosition - positionSignal.getValueAsDouble());
      if (false) {//(error > ClimberSubsystemConstants.CLIMBER_HOLD_TOLERANCE_ROTATIONS) {
        climberMotor.setControl(holdRequest.withPosition(holdPosition));
      } else {
        climberMotor.set(0);
      }
    }
    // climberDir == -2 means a homeClimber() command is running and owns the motor directly.
    // periodic() intentionally does nothing in that state.


    NetworkedConfig.Climber.setCurrentLimitState(!climberLimitSwitch.get());
  }

  public void raise() {
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
  // public Command homeClimber() {
  //   // A Timer local to this command instance tracks how long current has been
  //   // above the stall threshold. It is created inside the factory method so
  //   // each invocation gets its own independent timer.
  //   Timer stallTimer = new Timer();

  //   return this.runOnce(() -> {
  //         // Disable the normal hold loop while homing.
  //         climberDir = -2; // sentinel: "homing in progress"
  //         stallTimer.restart();
  //       })
  //       .andThen(this.run(() -> {
  //         // Drive slowly toward the lower hard stop.
  //         climberMotor.set(ClimberSubsystemConstants.CLIMBER_HOMING_SPEED);

  //         double amps = statorCurrentSignal.getValueAsDouble();
  //         if (Math.abs(amps) < ClimberSubsystemConstants.CLIMBER_HOMING_STALL_AMPS) {
  //           // Not yet stalled — restart the timer so it only counts
  //           // *continuous* time above the threshold.
  //           stallTimer.restart();
  //         }
  //       }))
  //       .until(() ->
  //           Math.abs(statorCurrentSignal.getValueAsDouble()) >= ClimberSubsystemConstants.CLIMBER_HOMING_STALL_AMPS
  //           && stallTimer.hasElapsed(ClimberSubsystemConstants.CLIMBER_HOMING_STALL_DURATION_S))
  //       .finallyDo((interrupted) -> {
  //         climberMotor.set(0);
  //         stallTimer.stop();

  //         if (!interrupted) {
  //           // Hard stop confirmed — zero the position sensor.
  //           climberMotor.setPosition(0);
  //           holdPosition = 0.0;
  //           isHomed = true;
  //           System.out.println("Homed Climber Position");
  //         }
  //         // Return to normal hold mode (climberDir = 0).
  //         climberDir = 0;
  //       })
  //       .unless(() -> isHomed);
  // }

  public Command homeClimber() {
    return this.runOnce(() -> {
          // Disable the normal hold loop while homing.
          climberDir = -2; // sentinel: "homing in progress"
        })
        .andThen(this.run(() -> {
          // Drive slowly toward the lower hard stop.
          climberMotor.set(ClimberSubsystemConstants.CLIMBER_HOMING_SPEED);
        }))
        .until(() ->
            !climberLimitSwitch.get())
        .finallyDo((interrupted) -> {
          climberMotor.set(0);

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

  /** Clears the homed flag, forcing a re-home on the next homing command. */
  public void resetHoming() {
    isHomed = false;
  }

  /** Returns the current climber position in motor rotations (from the cached position signal). */
  public double getPositionRotations() {
    return positionSignal.getValueAsDouble();
  }

  /**
   * Returns a {@link Command} that drives the climber to full extension and then
   * holds position. Intended for use in the automated climb sequence — call this
   * before driving under the bar.
   *
   * <p>The command runs {@link #raise()} until the position reaches
   * {@code CLIMBER_FULL_EXTENSION_ROTATIONS}, then calls {@link #stopClimber()} to
   * latch the hold controller at that position.</p>
   */
  public Command raiseToFull() {
    return this.run(() -> raise())
        .until(() -> getPositionRotations()
            >= ClimberSubsystemConstants.CLIMBER_FULL_EXTENSION_ROTATIONS
               - ClimberSubsystemConstants.CLIMBER_HOLD_TOLERANCE_ROTATIONS)
        .withTimeout(5.0)
        .andThen(this.runOnce(() -> stopClimber()));
  }

  /**
   * Returns a {@link Command} that lowers the climber back to the homed (zero)
   * position. Intended for use after the robot has driven under the bar — lowering
   * engages the hooks on the bar.
   *
   * <p>The command runs {@link #lower()} until the encoder reads near zero, then
   * calls {@link #stopClimber()} to engage the hold controller.</p>
   */
  public Command lowerOntoBar() {
    return this.run(() -> lower())
        .until(() -> getPositionRotations()
            <= ClimberSubsystemConstants.CLIMBER_HOLD_TOLERANCE_ROTATIONS)
        .withTimeout(5.0)
        .andThen(this.runOnce(() -> stopClimber()));
  }
}
