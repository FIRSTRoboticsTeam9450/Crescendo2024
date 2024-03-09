// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpositions.tosource;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.ArmWristSubsystem.Height;

public class BasicToSourceCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ArmWristSubsystem armWrist;
  private boolean finished = false;
  

  public BasicToSourceCommand(ArmWristSubsystem armWrist){
    // Use addRequirements() here to declare subsystem dependencies.
    this.armWrist = armWrist;
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    finished = false;

    armWrist.setWasSourceIntake(true);

    if(armWrist.getHeight() == Height.GROUND){
      armWrist.setArmWristExtGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToSource, 
                                Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToSource, 
                                Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToSource);

      finished = true;

    }else if(armWrist.getHeight() == Height.HOLD){
      armWrist.setArmWristGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToSource, 
                                Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToSource);

    }else if(armWrist.getHeight() == Height.AMP){
      armWrist.setArmWristExtGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToSource, 
                                Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToSource, 
                                Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToSource);

      finished = true;

    }else{
      armWrist.setArmWristExtGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToSource, 
                                Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToSource, 
                                Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToSource);

      finished = true;
    }
    
  
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(armWrist.getHeight() == Height.HOLD){
      if(Math.abs(armWrist.getAbsArmPos() - armWrist.newGetAbsArmTarget()) < 0.05){
        armWrist.setExtensionGoal(Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToSource);
  
        finished = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armWrist.changeHeight(Height.SOURCE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
