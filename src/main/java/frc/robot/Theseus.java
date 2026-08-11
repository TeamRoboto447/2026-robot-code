// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.Elastic;

public class Theseus extends LoggedRobot {
    private Command m_autonomousCommand;

    private final ShipOfTheseus m_robotContainer;
    private int periodicLoopCount = 0;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Theseus() {
        Logger.recordMetadata("ProjectName", "2026 Theseus"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            try {
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                setUseTiming(false); // Run as fast as possible
            } catch (StringIndexOutOfBoundsException e) {
                System.out.println("Running Basic Simulator");
                Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            }
            
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        m_robotContainer = new ShipOfTheseus();
    }

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        LiveWindow.disableAllTelemetry();
        StatusLogger.disableAutoLogging();
        SignalLogger.stop();

        

        // addPeriodic(() -> m_robotContainer.motorStatusCheck(), 10);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        
        //  Repulsor main update loop (minimal integration)
        if (m_robotContainer != null && m_robotContainer.repulsor != null) {
            m_robotContainer.repulsor.update();
            m_robotContainer.periodicUpdate();
        }

        //  Publish the status of the flash drive to networktables (connected, space remaining, etc.) once a second
        
        if (periodicLoopCount > 49) {
            periodicLoopCount = 0;

            SmartDashboard.putNumber("Intake/Lift Angle (Degrees)", m_robotContainer.intakeSubsystem.getIntakeAngleDegrees());

            if (isReal()) {
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

        periodicLoopCount++;
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
