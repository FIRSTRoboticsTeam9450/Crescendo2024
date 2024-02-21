// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpositions.tosource;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmWristSubsystem;

public class HoldToSourceCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ArmWristSubsystem armWrist;
  private boolean finished = false;
  

  public HoldToSourceCommand(ArmWristSubsystem armWrist){
    // Use addRequirements() here to declare subsystem dependencies.
    this.armWrist = armWrist;
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armWrist.setArmWristGoal(Constants.ArmPositions.armHardLowerLimit + Constants.Arm.offsetToSource, 
                                Constants.ArmPositions.wristHardLowerLimit + Constants.Wrist.offsetToSource);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Once the arm is close to the source position, then extend the extension
    if(Math.abs(armWrist.getAbsArmPos() - armWrist.getGoal()) < 0.1){
      armWrist.setExtensionGoal(Constants.ArmPositions.extHardLowerLimit + Constants.Extension.offsetToSource);
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
