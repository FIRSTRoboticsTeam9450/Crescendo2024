// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Scoring;

public class IntakingCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private Scoring score;
  private double intakeVoltage;
  boolean finished = false;
  private Timer timer = new Timer();
  MedianFilter median = new MedianFilter(3);

  public IntakingCommand(Scoring score, double intakeVoltage){
    // Use addRequirements() here to declare subsystem dependencies.
    this.score = score;
    this.intakeVoltage = intakeVoltage;
    addRequirements(score);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    finished = false;
    median = new MedianFilter(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double medianValue = median.calculate(score.getLaserDistance());

    if(score.getIntakeTemp() > 60){
        score.setIntakeVoltage(0);
    }else{        
        if (medianValue <= 10) {
          score.setIntakeVoltage(0.01);
          finished = true;
        } else {
          score.setIntakeVoltage(intakeVoltage);

        }
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
