// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;

public class ExtensionCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ExtensionSubsystem extension;
  private double extensionTarget;
  private ArmWristSubsystem armWristSub;
  private BooleanSupplier rightBumper;
  public ExtensionCommand(ExtensionSubsystem extension, ArmWristSubsystem armWristSub, BooleanSupplier rightBumper, double extensionTarget){
    // Use addRequirements() here to declare subsystem dependencies.
    this.extension = extension;
    this.extensionTarget = extensionTarget;
    this.armWristSub = armWristSub;
    this.rightBumper = rightBumper;
    addRequirements(extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(rightBumper.getAsBoolean()){
      extension.toggleRun();
    }
    //Need to figure out how to pass arm value into this so that the FF works
    //extension.setExtensionGoal(extensionTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    armWristSub.updateArmFF(extension.getExtensionAbsPosition());

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
