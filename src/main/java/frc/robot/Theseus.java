// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.Elastic;

public class Theseus extends TimedRobot {
    private Command m_autonomousCommand;

    private final ShipOfTheseus m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Theseus() {
        m_robotContainer = new ShipOfTheseus();
    }

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        LiveWindow.disableAllTelemetry();
        StatusLogger.disableAutoLogging();
        SignalLogger.stop();

        DataLogManager.start();

        addPeriodic(() -> m_robotContainer.motorStatusCheck(), 10);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        // Repulsor main update loop (minimal integration)
        if (m_robotContainer != null && m_robotContainer.repulsor != null) {
            m_robotContainer.repulsor.update();
            m_robotContainer.periodicUpdate();
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        m_robotContainer.pullAllNetworkedConfigs();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
        CommandScheduler.getInstance().schedule(Commands.runOnce(() -> m_robotContainer.pullAllNetworkedConfigs()));
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        m_robotContainer.pullAllNetworkedConfigs();
        Elastic.selectTab("Teleoperated");
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.pullAllNetworkedConfigs();
        CommandScheduler.getInstance().schedule(m_robotContainer.getSystemsCheckCommand());
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
