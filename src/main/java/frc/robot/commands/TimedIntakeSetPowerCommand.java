// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class TimedIntakeSetPowerCommand extends Command {
  /** Creates a new WristIntakeCommand. */
  private IntakeSubsystem intake;
  private double voltage, seconds;
  private boolean finished;
  Timer time;

  public TimedIntakeSetPowerCommand(IntakeSubsystem intake, double voltage, double seconds){
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.voltage = voltage;
    this.seconds = seconds;
    time = new Timer();
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    finished = false;
    intake.setIntakeVoltage(-Math.abs(voltage));
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
    intake.setIntakeVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
