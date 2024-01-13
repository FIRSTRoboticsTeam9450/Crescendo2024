// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.motorcontroller.BrushlessSparkFlexController;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;


public class MotorSubsystem extends SubsystemBase {
  /** Creates a new MotorSubsystem. */
  // CANSparkFlex motor = new CANSparkFlex(Constants.motorID, MotorType.kBrushless);;
  // CANSparkFlex motorAngle = new CANSparkFlex(Constants.motorAngleID, MotorType.kBrushless);;
  BrushlessSparkFlexController controller1 = new BrushlessSparkFlexController(1);
  BrushlessSparkFlexController controller2 = new BrushlessSparkFlexController(4);
  BrushlessSparkFlexController controller3 = new BrushlessSparkFlexController(7);
  BrushlessSparkFlexController controller4 = new BrushlessSparkFlexController(10);
  
  BrushlessSparkFlexController controller5 = new BrushlessSparkFlexController(2);
  BrushlessSparkFlexController controller6 = new BrushlessSparkFlexController(5);
  BrushlessSparkFlexController controller7 = new BrushlessSparkFlexController(8);
  BrushlessSparkFlexController controller8 = new BrushlessSparkFlexController(11);
  
  // CANcoder coder = new CANcoder(3);
  Talon hello = new Talon(19);
  // PigeonIMU pigeonIMU = new PigeonIMU(13);
  
  
  public MotorSubsystem() {
    

  } 

  
  public void runMotor(double power) {
    controller1.set(power);
    controller2.set(power);
    controller3.set(power);
    controller4.set(power);
    controller5.set(power);
    controller6.set(power);
    controller7.set(power);
    controller8.set(power);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
