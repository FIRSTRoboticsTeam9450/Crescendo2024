// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.motorcontroller.BrushlessSparkFlexController;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.SparkPIDController;


public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new MotorSubsystem. */
  
  /*
  Initalize each SparkFlex with specific IDs
  */
  CANSparkFlex wrist;
  SparkAbsoluteEncoder wristEncoder;
  CANSparkMax climb;
  RelativeEncoder encoder;
  PIDController climbController;
  DigitalInput limitSwitch;

  /* <!---- IMPORTANT ----!>
  Without the two line of code below, the vortexs run perfectly fine.
  However, with the CTRE CANcoder initialized or the CTRE PigeonIMU, 
  the issue of the vortexes switching between brake mode and 
  coast mode occurs.
  */
  
  // 0 -> 97
  public ClimbSubsystem() {
    climb = new CANSparkMax(25, MotorType.kBrushless);
    encoder = climb.getEncoder();
    encoder.setPosition(0);
    climbController = new PIDController(0.5, 0, 0);
    climbController.setSetpoint(90);

    wrist = new CANSparkFlex(30, MotorType.kBrushless);
    wristEncoder = wrist.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);

    limitSwitch = new DigitalInput(0);
  } 

  
  public void runMotor(double power) {
    climb.setVoltage(power);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public void setTargetPosition(double position) {
    climbController.setSetpoint(position);
  }

  public void updatePID() {
    System.out.println("Running!");
    double power = climbController.calculate(getPosition());
    power = MathUtil.clamp(power, -12, 12);
    SmartDashboard.putNumber("Climb power", power);
    //power = -1;
    runMotor(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb pos", getPosition());
    SmartDashboard.putNumber("Wrist Encoder", wristEncoder.getPosition());
    SmartDashboard.putNumber("Target", climbController.getSetpoint());
    SmartDashboard.putNumber("Error", climbController.getPositionError());
    SmartDashboard.putBoolean("Switch", limitSwitch.get());
  }
}
