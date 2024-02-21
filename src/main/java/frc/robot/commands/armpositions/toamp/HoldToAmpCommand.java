// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpositions.toamp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmWristSubsystem;

public class HoldToAmpCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ArmWristSubsystem armWrist;
  private boolean finished = false;
  

  public HoldToAmpCommand(ArmWristSubsystem armWrist){
    // Use addRequirements() here to declare subsystem dependencies.
    this.armWrist = armWrist;
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armWrist.setArmWristGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToAmpFromGround, 
    Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToAmpFromSource_Hold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(armWrist.getAbsArmPos() - armWrist.getGoal()) < 0.1){
      armWrist.setExtensionGoal(Constants.ArmPositions.extHardLowerLimit - 5);
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
