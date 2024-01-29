// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWristSubsystem;

public class ArmWristCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ArmWristSubsystem armWrist;
  private double armTarget, wristTarget;

  public ArmWristCommand(ArmWristSubsystem armWrist, double armTarget, double wristTarget){
    // Use addRequirements() here to declare subsystem dependencies.
    this.armWrist = armWrist;
    this.armTarget = armTarget;
    this.wristTarget = wristTarget;
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armWrist.setArmWristGoal(armTarget, wristTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Krish is the best
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
