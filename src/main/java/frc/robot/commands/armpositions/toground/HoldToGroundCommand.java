// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpositions.toground;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmWristSubsystem;

public class HoldToGroundCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ArmWristSubsystem armWrist;
  private boolean finished = false;
  

  public HoldToGroundCommand(ArmWristSubsystem armWrist){
    // Use addRequirements() here to declare subsystem dependencies.
    this.armWrist = armWrist;
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armWrist.setArmWristGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToStraightOutPos, 
    Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToGround);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(armWrist.getAbsArmPos() - armWrist.getGoal()) < 0.05) {
      armWrist.setExtensionGoal(Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToGround);
    }

    if(Math.abs(armWrist.getAbsArmPos() - armWrist.getGoal()) < 3) {
      armWrist.setArmGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToGround);
      finished = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
