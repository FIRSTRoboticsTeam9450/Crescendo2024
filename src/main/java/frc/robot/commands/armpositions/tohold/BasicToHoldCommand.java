// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpositions.tohold;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.ArmWristSubsystem.Height;

public class BasicToHoldCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ArmWristSubsystem armWrist;
  private boolean finished = false;
  private boolean secondState = false;
  

  public BasicToHoldCommand(ArmWristSubsystem armWrist){
    // Use addRequirements() here to declare subsystem dependencies.
    this.armWrist = armWrist;
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    secondState = false;
    finished = false;

    if(armWrist.getHeight() == Height.GROUND){
      armWrist.setArmWristGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToStraightOutPos, 
    Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToHold);

    }else if(armWrist.getHeight() == Height.AMP){
      armWrist.setArmWristExtGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToHold, 
    Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToHold, 
    Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToHold); 

    finished = true;

    }else if(armWrist.getHeight() == Height.SOURCE){
      armWrist.setArmWristExtGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToHold, 
    Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToHold, 
    Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToHold); 
  
    finished = true;

    }else{
      armWrist.setArmWristExtGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToHold, 
      Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToHold, 
      Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToHold);

      finished = true;
    } 
  
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(armWrist.getHeight() == Height.GROUND){
      if (Math.abs(armWrist.getAbsArmPos() - armWrist.newGetArmTarget()) < 0.05) {
        armWrist.setExtensionGoal(Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToHold);
        secondState = true;
      }
  
      if(Math.abs(armWrist.getExtRelPos() - armWrist.getExtensionGoal()) < 3 && secondState) {
        armWrist.setArmGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToHold);
        finished = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armWrist.changeHeight(Height.HOLD);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
