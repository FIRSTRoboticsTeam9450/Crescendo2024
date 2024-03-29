// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpositions.toground;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Examples.ArmWristSubsystem;
import frc.robot.subsystems.Examples.ArmWristSubsystem.Height;

public class BasicToGroundCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ArmWristSubsystem armWrist;
  private boolean finished = false;
  private boolean secondState = false;
  

  public BasicToGroundCommand(ArmWristSubsystem armWrist){
    // Use addRequirements() here to declare subsystem dependencies.
    this.armWrist = armWrist;
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    secondState = false;
    finished = false;

    armWrist.setWasSourceIntake(false);

    if(armWrist.getHeight() == Height.AMP){
      // added an extra 0.02 because when going from amp to ground can cause some values being a bit off
      armWrist.setArmWristExtGoal(Constants.MovementLimits.armHardLowerLimit + Constants.Arm.offsetToGround + 0.013, 
                                Constants.MovementLimits.wristHardLowerLimit + Constants.Wrist.offsetToGround, 
                                Constants.MovementLimits.extHardLowerLimit + Constants.Extension.offsetToGround);

    }else if(armWrist.getHeight() == Height.SOURCE){
      armWrist.setArmWristExtGoal(Constants.MovementLimits.armHardLowerLimit + Constants.Arm.offsetToGround, 
    Constants.MovementLimits.wristHardLowerLimit + Constants.Wrist.offsetToGround, 
    Constants.MovementLimits.extHardLowerLimit + Constants.Extension.offsetToGround);

    finished = true;

    }else if(armWrist.getHeight() == Height.HOLD){
      armWrist.setArmWristGoal(Constants.MovementLimits.armHardLowerLimit + Constants.Arm.offsetToStraightOutPos, 
    Constants.MovementLimits.wristHardLowerLimit + Constants.Wrist.offsetToGround);
  

    }else{
      armWrist.setArmWristExtGoal(Constants.MovementLimits.armHardLowerLimit + Constants.Arm.offsetToGround, 
    Constants.MovementLimits.wristHardLowerLimit + Constants.Wrist.offsetToGround, 
    Constants.MovementLimits.extHardLowerLimit + Constants.Extension.offsetToGround);

      finished = true;

    }
    
    
  
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(armWrist.getHeight() == Height.AMP){
      if (Math.abs(armWrist.getArmRelPos() - armWrist.newGetAbsArmTarget()) < 0.05) {
        armWrist.setExtensionGoal(Constants.MovementLimits.extHardLowerLimit + Constants.Extension.offsetToGround);
        finished = true;
      }
    }



    

    if(armWrist.getHeight() == Height.HOLD){
      if (Math.abs(armWrist.getArmRelPos() - armWrist.newGetAbsArmTarget()) < 0.05) {
        armWrist.setExtensionGoal(Constants.MovementLimits.extHardLowerLimit + Constants.Extension.offsetToGround);
        secondState = true;
      }
  
      if(Math.abs(armWrist.getExtRelPos() - armWrist.getExtensionGoal()) < 1.5 && secondState) {
        armWrist.setArmGoal(Constants.MovementLimits.armHardLowerLimit + Constants.Arm.offsetToGround);
        finished = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armWrist.changeHeight(Height.GROUND);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
