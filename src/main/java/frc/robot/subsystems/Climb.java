// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

/**
 * Manages two climber modules with independent limit switches and PID controllers
 */
public class Climb extends SubsystemBase {
  CANSparkFlex leftClimb;
  CANSparkFlex rightClimb;
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  PIDController leftClimbController;
  PIDController rightClimbController;
  SparkLimitSwitch leftLimitSwitch;
  SparkLimitSwitch rightLimitSwitch;
  private double leftMotorVoltage;
  private double rightMotorVoltage;
  private boolean runPid;
  private boolean stopped;
  
  /**
   * Creates a new Climb subsystem
   */
  public Climb() {
    runPid = false;

    leftMotorVoltage = 0;
    rightMotorVoltage = 0;

    leftClimb = new CANSparkFlex(Constants.lClimberId, MotorType.kBrushless);
    rightClimb = new CANSparkFlex(Constants.rClimberId, MotorType.kBrushless);

    leftClimb.setSmartCurrentLimit(40);
    rightClimb.setSmartCurrentLimit(40);

    leftEncoder = leftClimb.getEncoder();
    rightEncoder = rightClimb.getEncoder();

    // PID Constants
    leftClimbController = new PIDController(0.5, 0, 0);
    rightClimbController = new PIDController(0.5, 0, 0);

    leftClimbController.setSetpoint(10);
    rightClimbController.setSetpoint(10);

    leftLimitSwitch = leftClimb.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    rightLimitSwitch = rightClimb.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  } 

  /**
   * Sets a specified voltage to the left climber motor
   * @param volts the desired voltage of the motor (-12 to 12)
   */
  public void setLeftVoltage(double volts) {
    leftMotorVoltage = volts;
  }

  /**
   * Sets a specified voltage to the right climber motor
   * @param volts the desired voltage of the motor (-12 to 12)
   */
  public void setRightVoltage(double volts) {
    rightMotorVoltage = volts;
  }

  /**
   * Gets the position of the left climber motor's encoder
   * @return the encoder position in revolutions
   */
  public double getLeftPosition() {
    try {
      return leftEncoder.getPosition();
    } catch (NullPointerException e) {
      e.printStackTrace();
    }
    return 0;
  }

  /**
   * Gets the position of the right climber motor's encoder
   * @return the encoder position in revolutions
   */
  public double getRightPosition() {
    try {
      return rightEncoder.getPosition();
    } catch (NullPointerException e) {
      e.printStackTrace();
    }
    return 0;
  }

  /**
   * Sets the setpoint of both PID controllers to the given value
   * @param position the desired position in revolutions
   */
  public void setTargetPosition(double position) {
    leftClimbController.setSetpoint(position);
    rightClimbController.setSetpoint(position + 1);
  }

  public boolean isStopped() {
    return stopped;
  }

  /**
   * Updates the motor output voltage based on their current position
   */
  public void updatePID() {
    double leftPower = leftClimbController.calculate(getLeftPosition());
    leftPower = MathUtil.clamp(leftPower, -8, 10);
    //SmartDashboard.putNumber("Left Climb power", leftPower);
    setLeftVoltage(leftPower);

    double rightPower = rightClimbController.calculate(getRightPosition());
    rightPower = MathUtil.clamp(rightPower, -8, 10);
    //SmartDashboard.putNumber("Right Climb power", rightPower);
    setRightVoltage(rightPower);
  }

  /**
   * Turns the PID controllers of the climbers on or off
   * @param runPid the desired state of the pid controller
   */
  public void enablePid(boolean runPid) {
    this.runPid = runPid;
    setLeftVoltage(0);
    setRightVoltage(0);
  }

  /**
   * Returns true if the magnet is in range of the left limit switch
   * @return the state of the limit switch
   */
  public boolean getLeftLimitSwitch() {
    try {
      return leftLimitSwitch.isLimitSwitchEnabled();
    } catch (NullPointerException e) {
      e.printStackTrace();
    }
    return false; //cuz running down is better than running up
  }

  /**
   * Returns true if the magnet is in range of the right limit switch
   * @return the state of the limit switch
   */
  public boolean getRightLimitSwitch() {
    try {
      return rightLimitSwitch.isLimitSwitchEnabled();
    } catch (NullPointerException e) {
      e.printStackTrace();
    }
    return false; //cuz running down is better than running up
  }

  /**
   * Gets the voltage the left climber motor is set to
   * @return the left climber voltage
   */
  public double getLeftVoltage() {
    return leftMotorVoltage;
  }

  /**
   * Gets the voltage the right climber motor is set to
   * @return the right climber voltage
   */
  public double getRightVoltage() {
    return rightMotorVoltage;
  }

  public boolean isFinishedMoving() {
    return leftClimbController.getPositionError() < 2 && rightClimbController.getPositionError() < 2;
  }

  @Override
  public void periodic() {
    if (runPid) {
      updatePID();
    }
    
    // Set voltage to 0 and reset encoder if either module is going down and the limit switch trips 
    if (getLeftLimitSwitch() && leftMotorVoltage < 0) {
      leftMotorVoltage = 0;
      leftEncoder.setPosition(0);
    }
    if (getRightLimitSwitch() && rightMotorVoltage < 0) {
      rightMotorVoltage = 0;
      rightEncoder.setPosition(0);
    }

    stopped = leftMotorVoltage == 0 && rightMotorVoltage == 0;

    // Update motor voltages
    leftClimb.setVoltage(leftMotorVoltage);
    rightClimb.setVoltage(rightMotorVoltage);


    /* 
    // Telemetry
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
    */
  
  }
}
