// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.MotorSubsystem;

public class ClimbCommand extends Command {
  /** Creates a new MotorCommand. */
  ClimbSubsystem subsystem;
  public ClimbCommand(ClimbSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    Run Vortex motors at 0.2.
    */
    subsystem.updatePID();
  }

  
}
