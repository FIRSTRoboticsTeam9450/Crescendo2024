// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Scoring;

public class IntakingCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private Scoring score;
  private double intakeVoltage;
  boolean finished = false;
  boolean ramp = false;
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
    ramp = false;
    median = new MedianFilter(3);
    score.setIntaking();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double medianValue = median.calculate(score.getLaserDistance());
    
    //System.out.println("INTAKE VELOCITY" + score.getIntakeVelocity()); // REMOVE AFTER DONE TESTING
    
    if(score.getIntakeTemp() > 60){
        score.setIntakeVoltage(0);
    }else{     
        
        // if (score.useVelocityIntake()) { // inside this first iff statement is all the code for velocity intake

        //   if (!ramp) {
        //     score.setIntakeVoltage(intakeVoltage);
        //     if(score.getIntakeVelocity() < -2500 || timer.get() > 1){
        //         ramp = true;
        //     }
        //   } else if(ramp && score.getIntakeVelocity() > -700){ // velocity might not be right
        //       score.setIntakeVoltage(0.01);
        //   }

        // } else {
          if (score.getIntakeState().equals(Constants.IntakeState.HAS_NOTE)) {
            score.setIntakeVoltage(0.01);
            finished = true;
          } else {
            score.setIntakeVoltage(intakeVoltage);

          }
        //}
        
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
