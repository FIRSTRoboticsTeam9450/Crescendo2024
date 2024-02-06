// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;


public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new MotorSubsystem. */
  
  /*
  Initalize each SparkFlex with specific IDs
  */
  CANSparkMax leftClimb;
  CANSparkMax rightClimb;
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  PIDController leftClimbController;
  PIDController rightClimbController;
  DigitalInput leftLimitSwitch;
  DigitalInput rightLimitSwitch;
  private double leftMotorVoltage;
  private double rightMotorVoltage;
  private boolean runPid;
  
  // 0 -> 97
  public ClimbSubsystem() {
    runPid = true;

    leftMotorVoltage = 0;
    rightMotorVoltage = 0;

    leftClimb = new CANSparkMax(30, MotorType.kBrushless);
    rightClimb = new CANSparkMax(29, MotorType.kBrushless);

    rightClimb.setInverted(false);

    leftClimb.setSmartCurrentLimit(40);
    rightClimb.setSmartCurrentLimit(40);

    leftEncoder = leftClimb.getEncoder();
    rightEncoder = rightClimb.getEncoder();

    leftClimbController = new PIDController(0.5, 0, 0);
    rightClimbController = new PIDController(0.5, 0, 0);

    leftClimbController.setSetpoint(10);
    rightClimbController.setSetpoint(10);

    leftLimitSwitch = new DigitalInput(0);
    rightLimitSwitch = new DigitalInput(1);

    rightClimb.burnFlash();
  } 

  
  public void setLeftVoltage(double volts) {
    leftMotorVoltage = volts;
  }

  public void setRightVoltage(double volts) {
    rightMotorVoltage = volts;
  }

  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition();
  }

  public void setTargetPosition(double position) {
    leftClimbController.setSetpoint(position);
    rightClimbController.setSetpoint(position);
  }

  public void updatePID() {
    double leftPower = leftClimbController.calculate(getLeftPosition());
    leftPower = MathUtil.clamp(leftPower, -2, 2);
    SmartDashboard.putNumber("Left Climb power", leftPower);
    setLeftVoltage(leftPower);

    double rightPower = rightClimbController.calculate(getRightPosition());
    rightPower = MathUtil.clamp(rightPower, -2, 2);
    SmartDashboard.putNumber("Right Climb power", rightPower);
    setRightVoltage(rightPower);
  }

  public void setPid(boolean runPid) {
    this.runPid = runPid;
    setLeftVoltage(0);
    setRightVoltage(0);
  }

  public boolean getLeftLimitSwitch() {
    return !leftLimitSwitch.get();
  }

  public boolean getRightLimitSwitch() {
    return !rightLimitSwitch.get();
  }

  public double getLeftVoltage() {
    return leftMotorVoltage;
  }

  public double getRightVoltage() {
    return rightMotorVoltage;
  }

  @Override
  public void periodic() {
    if (runPid) {
      updatePID();
    }
    if (getLeftLimitSwitch() && leftMotorVoltage < 0) {
      leftMotorVoltage = 0;
      leftEncoder.setPosition(0);
    }
    if (getRightLimitSwitch() && rightMotorVoltage < 0) {
      rightMotorVoltage = 0;
      rightEncoder.setPosition(0);
    }
    leftClimb.setVoltage(leftMotorVoltage);
    rightClimb.setVoltage(rightMotorVoltage);

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climb pos", getLeftPosition());
        SmartDashboard.putNumber("Right Climb pos", getRightPosition());
    SmartDashboard.putNumber("Left Climb Voltage", leftMotorVoltage);
        SmartDashboard.putNumber("Right Climb Voltage", rightMotorVoltage);
    SmartDashboard.putNumber("Left Target", leftClimbController.getSetpoint());
        SmartDashboard.putNumber("Right Target", rightClimbController.getSetpoint());
    SmartDashboard.putNumber("Error", leftClimbController.getPositionError());
    SmartDashboard.putBoolean("Left Switch", getLeftLimitSwitch());
        SmartDashboard.putBoolean("Right Switch", getRightLimitSwitch());
    SmartDashboard.putBoolean("PID", runPid);
  }
}
