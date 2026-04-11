// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.Elastic;

public class Theseus extends TimedRobot {
    private Command m_autonomousCommand;

    private final ShipOfTheseus m_robotContainer;
    private int periodicLoopCount = 0;

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

        //  Publish the status of the flash drive to networktables (connected, space remaining, etc.) once a second
        if (periodicLoopCount > 49) {
            periodicLoopCount = 0;

            try {
                File logDir = new File(DataLogManager.getLogDir());
                long rawLogSpaceLeft = logDir.getFreeSpace();  // Gets remaining space in bytes
                SmartDashboard.putNumber("Logging Info/Log Space Remaining (MB)", rawLogSpaceLeft / 1024.0 / 1024.0); // Sends space to NT in MB

                boolean flashDriveConnected = DataLogManager.getLogDir().charAt(1) == 'u' && logDir.exists();
                SmartDashboard.putBoolean("Logging Info/Flash Drive Connected", flashDriveConnected); // Sends true or false depending on whether or not the flash is connected
            } catch (NullPointerException e) {
                System.out.println("Could not open file " + DataLogManager.getLogDir());
            }

        }

        
        //  Repulsor main update loop (minimal integration)
        if (m_robotContainer != null && m_robotContainer.repulsor != null) {
            m_robotContainer.repulsor.update();
            m_robotContainer.periodicUpdate();
        }

        periodicLoopCount++;
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
