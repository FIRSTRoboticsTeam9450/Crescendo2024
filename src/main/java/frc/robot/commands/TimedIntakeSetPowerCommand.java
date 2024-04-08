// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Scoring;

public class TimedIntakeSetPowerCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private Scoring score;
  private double voltage, seconds;
  private boolean finished;
  Timer time;

  public TimedIntakeSetPowerCommand(Scoring score, double voltage, double seconds){
    // Use addRequirements() here to declare subsystem dependencies.
    this.score = score;
    this.voltage = voltage;
    this.seconds = seconds;
    time = new Timer();
    addRequirements(score);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    finished = false;
    score.setIntakeVoltage(-Math.abs(voltage));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time.get() > seconds){
        finished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //score.rampDownIntakeVoltage(-Math.abs(voltage), 0, 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
