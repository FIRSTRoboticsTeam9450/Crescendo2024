// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimbSubsystem;

public class ResetClimbCommand extends Command {
  CommandXboxController controller;
  /** Creates a new MotorCommand. */
  ClimbSubsystem climb;
  public ResetClimbCommand(ClimbSubsystem climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    this.climb = climb;
  }

  @Override
  public void initialize() {
    climb.setPid(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setLeftVoltage(-1);
    climb.setRightVoltage(-1);
    /*
    Run Vortex motors at 0.2.
    */
    
  }

  @Override
  public boolean isFinished() {
    if (climb.getLeftVoltage() == 0) {
      climb.setPid(true);
      return true;
    }
    return false;
  }

  
}
