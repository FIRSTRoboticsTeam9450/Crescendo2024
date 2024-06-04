// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Scoring;

public class TestScoringCommand extends Command {
  private Scoring score;
  private CommandXboxController armController;
  /** Creates a new ScoringCommand. */
  public TestScoringCommand(Scoring score, CommandXboxController armController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.score = score;
    this.armController = armController;
    addRequirements(score);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //score.togglePIDs(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //score.togglePIDs(false);
    if (!score.ext.runAndReset) {
      //score.limit();
    }

    if (Math.abs(armController.getLeftY()) > 0.1) {
      score.wrist.setTarget(score.wrist.getTarget() + armController.getLeftY() * 0.5);
    }

    if (Math.abs(armController.getRightY()) > 0.1) {
      score.arm.setTarget(score.arm.getTarget() + armController.getRightY() * 0.5);
    }

    if (armController.povDown().getAsBoolean()) {
      score.ext.setTargetInches(score.ext.getTargetInches() - 0.02);
    }

    if (armController.povUp().getAsBoolean()) {
      score.ext.setTargetInches(score.ext.getTargetInches() + 0.02);
    }
    /*
    if (Math.abs(armController.getLeftY()) > 0.08) score.getExtSub().setVoltage(Math.abs(armController.getLeftY()) * 10 > 3 ? -Math.signum(armController.getLeftY()) * 3 : -armController.getLeftY() * 10);
    if (Math.abs(armController.getRightY()) > 0.08)    score.getArmSub().setVoltage(Math.abs(armController.getRightY()) * 10 > 3 ? Math.signum(armController.getRightY()) * 3 : armController.getRightY() * 10);
    if (Math.abs(armController.getRightTriggerAxis()) > 0.1) {
      score.getWristSub().setVoltage(Math.abs(armController.getRightTriggerAxis()) * 10 > 3 ? 3 : armController.getRightTriggerAxis() * 10);
    } else {
      score.getWristSub().setVoltage(Math.abs(armController.getLeftTriggerAxis()) * 10 > 3 ? -3 : -armController.getRightTriggerAxis() * 10);
    }
    */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
