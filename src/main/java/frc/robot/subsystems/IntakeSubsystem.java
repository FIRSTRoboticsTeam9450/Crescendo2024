// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /* Initializing Motors */
  CANSparkFlex intake = new CANSparkFlex(Constants.intakeId, MotorType.kBrushless);
  

  /* PID/FF */
 

  /* Other Values */
  boolean isIntaking;
  boolean rampDownBool;
  double currentVoltage;
  double voltageTo;
  double rampTime;


  Timer rampDownTimer;

  /* Constructor -> creates new WristIntakeSubsystem */
  public IntakeSubsystem() {
    
    intake.setIdleMode(IdleMode.kBrake);
    intake.setSmartCurrentLimit(20);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); // For Motor Position
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50); //Duty Cycle Absolute Encoder Position and Abs angle
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency
    intake.burnFlash();
    rampDownBool = false;
    rampDownTimer = new Timer();
    isIntaking = false;
  }


  /* Called every 20ms ish */
  @Override
  public void periodic() {
    
   
    /* Telemetry */
    SmartDashboard.putNumber("Intake Pos", getIntakePos());
    SmartDashboard.putNumber("Intake Velocity", getIntakeVelocity());
    
    //System.out.println(getIntakeVelocity());
    
    if (rampDownBool) {
      if (rampDownTimer.get() < rampTime) {
        setIntakeVoltage(-(currentVoltage - voltageTo) * rampDownTimer.get() / rampTime + currentVoltage);
      } else {
        stopIntake();
        rampDownBool = false;
        rampDownTimer.stop();
      }
    }
  }

  /* Wrist Methods */
    
  
  


  


  
  /* Intake Methods */
  public void intakeNotes(double voltage) {
    // goal voltage

    if (getIntakeVelocity() == 0 && !isIntaking) {
      setIntakeVoltage(voltage);
      isIntaking = !isIntaking;
    } else if (getIntakeVelocity() == 0 && isIntaking){
      setIntakeVoltage(0);
    }
  }
  public void intakeNote(double voltage){
    
      if (getIntakeVelocity() > -2500 && !isIntaking) {
        setIntakeVoltage(voltage);
        isIntaking = !isIntaking;
      } else {
        setIntakeVoltage(0);
      }
    
  }

  public double getTemp(){
    return intake.getMotorTemperature();
  }
  
  // linear ramp here https://www.desmos.com/calculator/ygpschqwqe
  public void rampDownVoltage(double currentVoltage, double voltageTo, double rampTime) {
    this.currentVoltage = currentVoltage;
    this.voltageTo = voltageTo;
    this.rampTime = rampTime;
    rampDownTimer.restart();
    rampDownBool = true;
  }

  

  
  /* Intake Methods [Small] */
  public void setIntakePower(double power) { intake.set(-power); }

  //NEED TO CHECK IF THIS IS THE RIGHT DIRECTION
  public void setIntakeVoltage(double voltage) { intake.setVoltage(-voltage); SmartDashboard.putNumber("Intake Voltage", -voltage);}
  public void setOuttake(double voltageMagnitude) { stopIntake(); intake.setVoltage(Math.abs(voltageMagnitude)); }

  public double getIntakePos() { return intake.getEncoder().getPosition(); }
  public void stopIntake() { intake.stopMotor(); isIntaking = false;}
  public double getIntakeVelocity() { return intake.getEncoder().getVelocity(); }
  /* Wrist Methods [Small] */  /* Range: (0, -16.27) [0 = side w/ wire management] */
                              // abs encoder (0.97, 0.399)
  // public void setWristPower(double power) { wrist.set(power); }
  
  /* General Methods [Small] */
  private double convertToVolts(double percentOutput){ return percentOutput * 12 /* Robot.getInstance().getVoltage() */ ;}
}
