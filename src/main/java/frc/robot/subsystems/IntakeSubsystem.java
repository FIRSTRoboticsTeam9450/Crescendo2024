// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
  /* Initializing Motors */
  CANSparkFlex intake = new CANSparkFlex(Constants.intakeId, MotorType.kBrushless);

  

  /* PID/FF */
 

  /* Other Values */
  boolean isIntaking;

  /* Constructor -> creates new WristIntakeSubsystem */
  public IntakeSubsystem() {
    
    intake.setIdleMode(IdleMode.kBrake);
    
    isIntaking = false;
  }


  /* Called every 20ms ish */
  @Override
  public void periodic() {
    
   
    /* Telemetry */
    SmartDashboard.putNumber("Intake Pos", getIntakePos());
    SmartDashboard.putNumber("Intake Velocity", getIntakeVelocity());
    System.out.println(getIntakeVelocity());
    
  }

  /* Wrist Methods */
    
  
  


  


  
  /* Intake Methods */
  public void intakeNote(double power) {
    // goal voltage

    if (getIntakeVelocity() > -2500 && !isIntaking) {
      setIntakePower(power);
      isIntaking = !isIntaking;
    } else {
      setIntakePower(0.05);
    }
  }

  public double getTemp(){
    return intake.getMotorTemperature();
  }



  
  /* Intake Methods [Small] */
  public void setIntakePower(double power) { intake.set(-power); }

  //NEED TO CHECK IF THIS IS THE RIGHT DIRECTION
  public void setIntakeVoltage(double voltage) { intake.setVoltage(voltage); }


  public double getIntakePos() { return intake.getEncoder().getPosition(); }
  public void stopIntake() { intake.stopMotor(); isIntaking = !isIntaking;}
  public double getIntakeVelocity() { return intake.getEncoder().getVelocity(); }
  /* Wrist Methods [Small] */  /* Range: (0, -16.27) [0 = side w/ wire management] */
                              // abs encoder (0.97, 0.399)
  // public void setWristPower(double power) { wrist.set(power); }
  
  /* General Methods [Small] */
  private double convertToVolts(double percentOutput){ return percentOutput * 12 /* Robot.getInstance().getVoltage() */ ;}
}
