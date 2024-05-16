// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

/**
 * Sets the target position of the climbers
 */
public class ClimbCommand extends Command {
  double target;
  Climb subsystem;
  /**
   * Creates a new climb command
   * @param subsystem the ClimbSubsystem to be used
   * @param target the desired setpoint (should be between 10 and 80 for safety)
   */
  public ClimbCommand(Climb subsystem, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.target = target;
  }

  @Override
  public void initialize() {
      subsystem.enablePid(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.setTargetPosition(MathUtil.clamp(target, 0, 90));
  }

  // Exit command when climber reaches target
  @Override
  public boolean isFinished() {
    return subsystem.isFinishedMoving();
  }

  
}
