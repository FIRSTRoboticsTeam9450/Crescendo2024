// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;
  private boolean runnable;

  @Override
  public void robotInit() {

    Logger.recordMetadata("ProjectName", "9450 Comp Bot"); // Set a metadata value

    if (isReal()) {
        
        // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
       /* 
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        */
    }

    Logger.disableDeterministicTimestamps(); // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start();

    m_robotContainer = new RobotContainer();

    disabledTimer = new Timer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    disabledTimer.restart();
    runnable = true;
    m_robotContainer.score.logStickyFaults(); // Log sticky faults
  

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.climbSub.enablePid(false);
    m_robotContainer.score.goToPosition(Constants.ScoringPos.CLIMB);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopInit() {

    m_robotContainer.score.ext.runAndResetEncoder();
    m_robotContainer.score.goToPosition(Constants.ScoringPos.CLIMB);
    
    m_robotContainer.climbSub.enablePid(false);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
