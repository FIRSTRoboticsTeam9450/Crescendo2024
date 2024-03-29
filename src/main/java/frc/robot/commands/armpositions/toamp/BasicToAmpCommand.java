// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpositions.toamp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.ArmWristSubsystem.Height;

public class BasicToAmpCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ArmWristSubsystem armWrist;
  private boolean finished = false;
  

  public BasicToAmpCommand(ArmWristSubsystem armWrist){
    // Use addRequirements() here to declare subsystem dependencies.
    this.armWrist = armWrist;
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    finished = false;

    if(armWrist.getHeight() == Height.GROUND){
      armWrist.setArmWristExtGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToAmpFromGround, 
                                Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToAmpFromGround, 
                                Constants.ArmPositions.extHardLowerLimit);
      System.out.println("GROUND");

    }else if(armWrist.getHeight() == Height.SOURCE){
      armWrist.setArmWristExtGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToAmpFromGround, 
    Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToAmpFromSource_Hold, 
    Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToAmpFromSource_Hold);
      System.out.println("SOURCE");
    finished = true;

    }else if(armWrist.getHeight() == Height.HOLD || armWrist.getHeight() == Height.AMP){
      armWrist.setArmWristGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToAmpFromGround, 
      Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToAmpFromSource_Hold);
      System.out.println("HOLD");
    }else{
      armWrist.setArmWristExtGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToAmpFromGround, 
      Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToAmpFromSource_Hold, 
      Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToAmpFromSource_Hold);
      System.out.println("IDEK");
      finished = true;

    }
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(armWrist.getHeight() == Height.GROUND){
      if (Math.abs(armWrist.getArmRelPos() - armWrist.newGetAbsArmTarget()) < 0.15) {
        armWrist.setExtensionGoal(Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToAmpFromGround);
        System.out.println("GROUND 2");
        finished = true;
      }
    }
    
    if(armWrist.getHeight() == Height.HOLD || armWrist.getHeight() == Height.AMP){
      if(Math.abs(armWrist.getArmRelPos() - armWrist.newGetAbsArmTarget()) < 0.15){

        //Need to figure out a way to keep track of whether intaken from amp or source
        if(armWrist.getWasSourceIntake()){
          armWrist.setExtensionGoal(Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToAmpFromSource_Hold);
          System.out.println("SOURCE 2");
        }else{
          armWrist.setExtensionGoal(Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToAmpFromGround);
          System.out.println("GROUND 3");
        }
        
        finished = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armWrist.changeHeight(Height.AMP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
