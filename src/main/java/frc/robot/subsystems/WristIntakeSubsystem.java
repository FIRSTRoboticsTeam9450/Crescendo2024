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

public class WristIntakeSubsystem extends SubsystemBase {
  /* Initializing Motors */
  CANSparkFlex wrist = new CANSparkFlex(Constants.wristId, MotorType.kBrushless);
  CANSparkFlex intake = new CANSparkFlex(Constants.intakeId, MotorType.kBrushless);

  /* Absolute Encoder */
  // if absolute encoder plugged into cansparkmax:
  CANSparkMax wristmax = new CANSparkMax(29, MotorType.kBrushless);
  SparkAbsoluteEncoder wristEncoder = wristmax.getAbsoluteEncoder(Type.kDutyCycle);

  /* PID/FF */
  private final PIDController wristPIDController =
      new PIDController(0.9, 0, 0); 
  public SimpleMotorFeedforward wristFeedForward = new SimpleMotorFeedforward(0.00001, 0.00003, 0.00001); 

  /* Other Values */
  boolean wristBrakeToggle;
  boolean wristPIDRun;
  boolean isIntaking;

  /* Constructor -> creates new WristIntakeSubsystem */
  public WristIntakeSubsystem() {
    wrist.setIdleMode(IdleMode.kBrake);
    intake.setIdleMode(IdleMode.kBrake);
    wristPIDRun = false;
    wristBrakeToggle = false;
    isIntaking = false;
  }


  /* Called every 20ms ish */
  @Override
  public void periodic() {
    if (wristPIDRun) {
      updateWristPos();
    }
   
    /* Telemetry */
    SmartDashboard.putNumber("Wrist Pos", getWristAbsPos());
    SmartDashboard.putNumber("Intake Pos", getIntakePos());
    SmartDashboard.putNumber("Intake Velocity", getIntakeVelocity());
    System.out.println(getIntakeVelocity());
    SmartDashboard.putBoolean("Wrist is Brake", wrist.getIdleMode() == IdleMode.kBrake ? true : false);
    
  }

  /* Wrist Methods */
    
  double oldVel = 0;
  double oldTime = 0;
  double oldPos = 0;
  public void updateWristPos() {
    double goalPos = wristPIDController.getSetpoint();
    double pidValue = wristPIDController.calculate(getWristAbsPos(), goalPos);
    double changeInTime = Timer.getFPGATimestamp() - oldTime;
    double velSetpoint = (getWristAbsPos() - oldPos) / changeInTime;
    double accel = (velSetpoint - oldVel) / (changeInTime); 
    double ffVal = wristFeedForward.calculate(velSetpoint, accel); //takes velocity, and acceleration
    
    double percentOutput = MathUtil.clamp(pidValue + ffVal, -1.0, 1.0);
    double voltage = convertToVolts(percentOutput);
    
    
    SmartDashboard.putNumber("PID Value", pidValue);
    SmartDashboard.putNumber("Feed Forward", ffVal);
    SmartDashboard.putNumber("Voltage", voltage);
    SmartDashboard.putNumber("Position error", wristPIDController.getPositionError());
  

    if (Math.abs(getWristAbsPos() - goalPos) <= 0.05) { // no voltage
      wrist.setVoltage(0);
    } else { // set voltage
      if (getWristAbsPos() <= Constants.maxWristPos && getWristAbsPos() >= Constants.minWristPos) { // is between max and min
        wrist.setVoltage(voltage); // could be negative voltage based upon direction of motor and gear
      }
    }
    
    // update vars for determining acceleration later
    oldVel =  velSetpoint; 
    oldTime = Timer.getFPGATimestamp(); 
    oldPos = getWristAbsPos();
  }


  public void setWristSetpoint(double goalPos) {
    wristPIDController.setSetpoint(goalPos);
    wristPIDRun = true;
  }



  
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



  
  /* Intake Methods [Small] */
  public void setIntakePower(double power) { intake.set(-power); }
  public double getIntakePos() { return intake.getEncoder().getPosition(); }
  public void stopIntake() { intake.stopMotor(); isIntaking = !isIntaking;}
  public double getIntakeVelocity() { return intake.getEncoder().getVelocity(); }
  /* Wrist Methods [Small] */  /* Range: (0, -16.27) [0 = side w/ wire management] */
                              // abs encoder (0.97, 0.399)
  // public void setWristPower(double power) { wrist.set(power); }
  public double getWristPos() { return wrist.getEncoder().getPosition(); }
  public double getWristAbsPos() { return wristEncoder.getPosition(); }
  public void stopWrist() { wrist.stopMotor(); }
  public void toggleWristBrake() { wrist.setIdleMode(wristBrakeToggle ? IdleMode.kBrake : IdleMode.kCoast); wristBrakeToggle = !wristBrakeToggle; }
  /* General Methods [Small] */
  private double convertToVolts(double percentOutput){ return percentOutput * 12 /* Robot.getInstance().getVoltage() */ ;}
}
