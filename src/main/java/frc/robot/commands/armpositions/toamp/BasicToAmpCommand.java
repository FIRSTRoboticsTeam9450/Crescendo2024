// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpositions.toamp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmWristSubsystem;

public class BasicToAmpCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ArmWristSubsystem armWrist;
  

  public BasicToAmpCommand(ArmWristSubsystem armWrist){
    // Use addRequirements() here to declare subsystem dependencies.
    this.armWrist = armWrist;
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armWrist.setArmWristExtGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToAmpFromGround, 
    Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToAmpFromSource_Hold, 
    Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToAmpFromSource_Hold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
