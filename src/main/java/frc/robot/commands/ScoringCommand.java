// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Scoring;

public class ScoringCommand extends Command {
  private Scoring score;
  private CommandXboxController armController;
  /** Creates a new ScoringCommand. */
  public ScoringCommand(Scoring score, CommandXboxController armController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.score = score;
    this.armController = armController;
    addRequirements(score);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    score.getArmSub().setVoltage(Math.abs(armController.getRightY()) * 10 > 3 ? Math.signum(armController.getRightY()) * 3 : armController.getRightY() * 10);
    score.getExtSub().setVoltage(Math.abs(armController.getLeftY()) * 10 > 3 ? Math.signum(armController.getLeftX()) * 3 : armController.getLeftY() * 10);
    score.getWristSub().setVoltage(Math.abs(armController.getRightTriggerAxis()) * 10 > 3 ? 3 : armController.getRightTriggerAxis() * 10);
    score.getWristSub().setVoltage(Math.abs(armController.getLeftTriggerAxis()) * 10 > 3 ? -3 : armController.getRightTriggerAxis() * 10);

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
