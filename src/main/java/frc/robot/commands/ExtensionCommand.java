// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;

public class ExtensionCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private ExtensionSubsystem extension;
  private double radiusX, radiusY, extensionLength, extensionTarget, totalextensionX, totalextensionY, theta;
  private ArmWristSubsystem armWristSub;
  private BooleanSupplier rightBumper;
  private boolean enabled = false;
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
      enabled = true;
    }

    theta = (((Constants.Arm.intakeArmAngle - Constants.Arm.ampArmAngle)/(Constants.Arm.intakeArmTics - Constants.Arm.ampArmTics)) * armWristSub.getAbsArmPos()) + (Constants.Arm.intakeArmAngle - (((Constants.Arm.intakeArmAngle - Constants.Arm.ampArmAngle)/(Constants.Arm.intakeArmTics - Constants.Arm.ampArmTics)) * Constants.Arm.intakeArmTics));
    radiusX = Constants.Arm.armLength - Math.abs((armWristSub.getAbsWristPos() - Constants.Wrist.straightWristTics) / ((Constants.Wrist.upWristTics-Constants.Wrist.straightWristTics)/(Constants.Wrist.straightWristInches-Constants.Wrist.upWristInches))) + Constants.Wrist.straightWristInches;
    radiusY = radiusX + 1;
    extensionLength = (Constants.Extension.maxExtensionInches / (Constants.Extension.maxExtensionTics - Constants.Extension.zeroTics)) * extension.getExtensionAbsPosition() - (Constants.Extension.zeroTics * (Constants.Extension.maxExtensionInches/(Constants.Extension.maxExtensionTics - Constants.Extension.zeroTics)));
    totalextensionX = (radiusX * Math.abs(Math.cos(theta))) + extensionLength;
    totalextensionY = (radiusY * Math.sin(theta)) + extensionLength;
    


    //Need to figure out how to pass arm value into this so that the FF works
    //extension.setExtensionGoal(extensionTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    //Updating all the values
    theta = (((Constants.Arm.intakeArmAngle - Constants.Arm.ampArmAngle)/(Constants.Arm.intakeArmTics - Constants.Arm.ampArmTics)) * armWristSub.getAbsArmPos()) + (Constants.Arm.intakeArmAngle - (((Constants.Arm.intakeArmAngle - Constants.Arm.ampArmAngle)/(Constants.Arm.intakeArmTics - Constants.Arm.ampArmTics)) * Constants.Arm.intakeArmTics));
    radiusX = Constants.Arm.armLength - Math.abs((armWristSub.getAbsWristPos() - Constants.Wrist.straightWristTics) / ((Constants.Wrist.upWristTics-Constants.Wrist.straightWristTics)/(Constants.Wrist.straightWristInches-Constants.Wrist.upWristInches))) + Constants.Wrist.straightWristInches;
    radiusY = radiusX + 1;
    extensionLength = (Constants.Extension.maxExtensionInches / (Constants.Extension.maxExtensionTics - Constants.Extension.zeroTics)) * extension.getExtensionAbsPosition() - (Constants.Extension.zeroTics * (Constants.Extension.maxExtensionInches/(Constants.Extension.maxExtensionTics - Constants.Extension.zeroTics)));
    totalextensionX = (radiusX * Math.abs(Math.cos(theta))) + extensionLength;
    totalextensionY = (radiusY * Math.sin(theta)) + extensionLength;


  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (enabled) { extension.toggleRun(); }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
