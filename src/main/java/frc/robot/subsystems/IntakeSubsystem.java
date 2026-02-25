// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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

    // Cached signals — avoids creating new StatusSignal objects every periodic() call.
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> intakeVelocitySignal;
    private final StatusSignal<edu.wpi.first.units.measure.Angle> liftPositionSignal;
    private final StatusSignal<Current> liftStatorCurrentSignal;

    /** Reusable position requests — allocated once to avoid per-loop GC pressure. */
    private final PositionTorqueCurrentFOC liftPositionReq = new PositionTorqueCurrentFOC(0).withSlot(0);

    private boolean isLiftHomed = false;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        liftMotor = new TalonFX(IntakeSubsystemConstants.LIFT_MOTOR_ID);
        intakeMotor = new TalonFX(IntakeSubsystemConstants.INTAKE_MOTOR_ID);

        // Apply current limits to the intake roller motor.
        // No PID config needed — it runs open-loop only.
        TalonFXConfiguration intakeFXConfigs = new TalonFXConfiguration();
        intakeFXConfigs.CurrentLimits
            .withStatorCurrentLimit(IntakeSubsystemConstants.INTAKE_STATOR_CURRENT_LIMIT_A)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(IntakeSubsystemConstants.INTAKE_SUPPLY_CURRENT_LIMIT_A)
            .withSupplyCurrentLimitEnable(true);
        intakeMotor.getConfigurator().apply(intakeFXConfigs);

        liftFXConfigs.Slot0
            .withKP(IntakeSubsystemConstants.LIFT_KP)
            .withKI(IntakeSubsystemConstants.LIFT_KI)
            .withKD(IntakeSubsystemConstants.LIFT_KD);

        // Current limits — prevent brownouts and protect motor/wiring.
        CurrentLimitsConfigs liftCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeSubsystemConstants.LIFT_STATOR_CURRENT_LIMIT_A)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(IntakeSubsystemConstants.LIFT_SUPPLY_CURRENT_LIMIT_A)
            .withSupplyCurrentLimitEnable(true);
        liftFXConfigs.CurrentLimits = liftCurrentLimits;

        liftMotor.getConfigurator().apply(liftFXConfigs);
        liftMotor.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(IntakeSubsystemConstants.LIFT_GEARBOX_RATIO)
        );
        // Assume stowed at startup — zero will be corrected by homeLift() before
        // any closed-loop move is attempted.
        liftMotor.setPosition(0);

        // Cache and configure signals.
        this.intakeVelocitySignal = intakeMotor.getVelocity();
        this.intakeVelocitySignal.setUpdateFrequency(20);

        this.liftPositionSignal = liftMotor.getPosition();
        this.liftPositionSignal.setUpdateFrequency(50);

        this.liftStatorCurrentSignal = liftMotor.getStatorCurrent();
        this.liftStatorCurrentSignal.setUpdateFrequency(20);

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
        // Refresh all cached signals in a single batched CAN read.
        BaseStatusSignal.refreshAll(intakeVelocitySignal, liftPositionSignal, liftStatorCurrentSignal);
        updateNetworkTables();
    }

    /**
     * Runs the intake.
     * 
     * @param speed The speed to intake at
     */
    public void intake(double speed) {
        intakeMotor.set(-speed);
    }

    /**
     * Runs the intake in reverse.
     * 
     * @param speed The speed to reverse at
     */
    public void reverseIntake(double speed) {
        intakeMotor.set(speed);
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
     * Drops the intake to the extended (lowered) position.
     * No-ops until the lift has been homed.
     */
    public void dropIntake() {
        if (!isLiftHomed) return;
        liftMotor.setControl(liftPositionReq.withPosition(NetworkedConfig.Intake.getLiftLoweredPosition()));
    }

    /**
     * Lifts the intake back to the stowed (zero) position.
     * No-ops until the lift has been homed.
     */
    public void liftIntake() {
        if (!isLiftHomed) return;
        liftMotor.setControl(liftPositionReq.withPosition(0));
    }

    /**
     * Returns true if the lift is within tolerance of the fully-lowered position.
     */
    public boolean isIntakeDown() {
        return Math.abs(liftPositionSignal.getValueAsDouble()
                - NetworkedConfig.Intake.getLiftLoweredPosition())
            <= IntakeSubsystemConstants.LIFT_POSITION_TOLERANCE_ROTATIONS;
    }

    /**
     * Returns true if the lift is within tolerance of the stowed (zero) position.
     */
    public boolean isIntakeUp() {
        return Math.abs(liftPositionSignal.getValueAsDouble())
            <= IntakeSubsystemConstants.LIFT_POSITION_TOLERANCE_ROTATIONS;
    }

    /** Returns true once the lift has been successfully homed. */
    public boolean isLiftHomed() {
        return isLiftHomed;
    }

    /** Clears the lift-homed flag, forcing a re-home on the next homing command. */
    public void resetLiftHoming() {
        isLiftHomed = false;
    }

    /**
     * Returns a {@link Command} that homes the lift by slowly driving it upward
     * toward the upper hard stop, detecting the stall via stator current, and
     * zeroing the encoder when the stall is confirmed.
     *
     * <p>The lift motor drives at {@code LIFT_HOMING_SPEED} (positive = raise)
     * until the stator current exceeds {@code LIFT_HOMING_STALL_AMPS} for at least
     * {@code LIFT_HOMING_STALL_DURATION_S} continuous seconds, indicating the
     * mechanism has hit the hard stop. The encoder is then zeroed so that 0 is the
     * stowed position and {@link IntakeSubsystemConstants#LIFT_LOWERED_ROTATIONS}
     * is the fully-lowered position.</p>
     *
     * @return the homing command
     */
    public Command homeLift() {
        Timer stallTimer = new Timer();

        return this.runOnce(() -> {
                liftMotor.set(0);
                stallTimer.restart();
            })
            .andThen(this.run(() -> {
                // Drive slowly upward toward the hard stop.
                liftMotor.set(IntakeSubsystemConstants.LIFT_HOMING_SPEED);

                // Reset the stall timer while current is below the threshold so it
                // only counts *continuous* over-current time.
                if (Math.abs(liftStatorCurrentSignal.getValueAsDouble())
                        < IntakeSubsystemConstants.LIFT_HOMING_STALL_AMPS) {
                    stallTimer.restart();
                }
            }))
            .until(() ->
                Math.abs(liftStatorCurrentSignal.getValueAsDouble())
                    >= IntakeSubsystemConstants.LIFT_HOMING_STALL_AMPS
                && stallTimer.hasElapsed(IntakeSubsystemConstants.LIFT_HOMING_STALL_DURATION_S))
            .finallyDo((interrupted) -> {
                liftMotor.set(0);
                stallTimer.stop();

                if (!interrupted) {
                    // Hard stop confirmed — zero the encoder so stowed = 0.
                    liftMotor.setPosition(0);
                    isLiftHomed = true;
                    System.out.println("Homed Lift Position");
                }
            })
            .unless(() -> isLiftHomed);
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
