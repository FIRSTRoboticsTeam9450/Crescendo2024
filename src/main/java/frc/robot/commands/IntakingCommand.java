// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakingCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private IntakeSubsystem intake;
  private double intakeVoltage;
  private boolean ramp;
  boolean finished = false;
  private Timer timer = new Timer();
  MedianFilter median = new MedianFilter(3);

  public IntakingCommand(IntakeSubsystem intake, double intakeVoltage){
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.intakeVoltage = intakeVoltage;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ramp = false;
    timer.reset();
    timer.start();
    finished = false;
    median = new MedianFilter(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double medianValue = median.calculate(intake.getLaserDistance());

    if(intake.getTemp() > 60){
        intake.setIntakeVoltage(0);
    }else{
        // if (!ramp) {
        //   intake.setIntakeVoltage(intakeVoltage);
        //     if(intake.getIntakeVelocity() < -2500 || timer.get() > 1){
        //         ramp = true;
        //     }
        // } else if(ramp && intake.getIntakeVelocity() > -700){
        //     intake.setIntakeVoltage(0.01);
        // }

        
        if (/*intake.getLaserDistance()*/medianValue <= 10 /*millimeters */) {
          intake.setIntakeVoltage(0.01);
          finished = true;
        } else {
          intake.setIntakeVoltage(intakeVoltage);

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
