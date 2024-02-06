// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.MotorSubsystem;

public class ClimbCommand extends Command {
  double target;
  /** Creates a new MotorCommand. */
  ClimbSubsystem subsystem;
  public ClimbCommand(ClimbSubsystem subsystem, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.target = target;
  }

  @Override
  public void initialize() {
      subsystem.setPid(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    Run Vortex motors at 0.2.
    */
    subsystem.setTargetPosition(target);
  }

  
}
