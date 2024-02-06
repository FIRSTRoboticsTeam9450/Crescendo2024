// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeTestingSubsystem extends SubsystemBase {
  /** Creates a new IntakeTestingSubsystem. */
  CANSparkFlex intake = new CANSparkFlex(26, MotorType.kBrushless);

  public IntakeTestingSubsystem() {
    intake.setIdleMode(IdleMode.kBrake);
    intake.setSmartCurrentLimit(20);
    intake.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // intake.setVoltage(5);
    // SmartDashboard.putNumber("Intake Velocity", intake.getEncoder().getVelocity());
  }

}
