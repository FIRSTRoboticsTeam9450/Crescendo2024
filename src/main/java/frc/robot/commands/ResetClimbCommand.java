// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimbSubsystem;

public class ResetClimbCommand extends Command {
  CommandXboxController controller;
  ClimbSubsystem climb;

  /**
   * Creates a new ResetClimbCommand
   * @param climb the ClimbSubsystem to be used
   */
  public ResetClimbCommand(ClimbSubsystem climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    this.climb = climb;
  }

  @Override
  public void initialize() {
    climb.enablePid(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // run both climbers at -1 volts
    // they will automatically stop when they reach the bottom
    climb.setLeftVoltage(-1);
    climb.setRightVoltage(-1);
  }

  @Override
  public boolean isFinished() {
    // stop command once both motors have reached the bottom
    return climb.getLeftVoltage() == 0 && climb.getRightVoltage() == 0;
  }

  
}
