// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
 
  private Constants.Height lastHeight = Constants.Height.HOLD;

  private double wristTarget = 0.387;

  private double wristHardLowerLimit = 0.141; // 0.033
  private double wristHardUpperLimit = 0.7785; // 0.632


  private CANSparkFlex wrist = new CANSparkFlex(Constants.wristId, MotorType.kBrushless);
  private SparkAbsoluteEncoder wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

  public SimpleMotorFeedforward wristFeedForward = new SimpleMotorFeedforward(0.00001, 0.00003, 0.00001); 

  private final PIDController wristPIDController = new PIDController(40, 0, 0); 

  boolean reachPos;
  boolean wristBrakeToggle;
  boolean wristPIDRun;
  boolean runStuff = true;

  public WristSubsystem() {
    wrist.restoreFactoryDefaults();
    wrist.setSmartCurrentLimit(40);
    wrist.setIdleMode(IdleMode.kBrake);

    /* Status frames 3-6 set to 65535 if not using data port in spark max otherwise can prob leave them all alone*/
    wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
    //wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
    //wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
    //wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50); //Duty Cycle Absolute Encoder Position and Abs angle
    //wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency
    wrist.burnFlash();

    wristPIDRun = true;
    goToPosition(Constants.Height.SOURCE);

    SmartDashboard.putNumber("Change Wrist Target", Constants.midWristPos);
    SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
    SmartDashboard.putNumber("Change Wrist Is Brake", 1);
  }
  double oldVel = 0;
  double oldTime = 0;
  double oldPos = 0;


  public void updateWristPos() {
      double goalPos = wristPIDController.getSetpoint();
      double pidValue = wristPIDController.calculate(getAbsWristPos(), goalPos); //change back to goalPos after testing
      double changeInTime = Timer.getFPGATimestamp() - oldTime;
      double velSetpoint = (getAbsWristPos() - oldPos) / changeInTime;
      double accel = (velSetpoint - oldVel) / (changeInTime); 
      double ffVal = wristFeedForward.calculate(velSetpoint, accel); //takes velocity, and acceleration
      
      double voltage = MathUtil.clamp(pidValue /*+ ffVal*/, -5.0, 5.0);
      
      
      SmartDashboard.putNumber("Wrist PID", pidValue);
      SmartDashboard.putNumber("Wrist FF", ffVal);
      SmartDashboard.putNumber("Wrist Voltage", voltage);
      SmartDashboard.putNumber("Wrist Pos Error", wristPIDController.getPositionError());
    
  /*
      if (Math.abs(getAbsWristPos() - goalPos) <= 0.05) { // no voltage
        wrist.setVoltage(0);
      } else { // set voltage
        if (getAbsWristPos() <= wristHardLowerLimit && getAbsWristPos() >= wristHardUpperLimit) { // is between max and min
          if(getAbsWristPos() >= wristHardLowerLimit && getAbsWristPos() <= wristHardUpperLimit){
              wrist.setVoltage(-voltage); // could be negative voltage based upon direction of motor and gear
          }else{
              wrist.setVoltage(0);
          }
          
        }
      }
      */
        if (Math.abs(voltage) < 3) {
          wrist.setVoltage(-voltage);
      } else {
          wrist.setVoltage(-3 * Math.signum(voltage));
      }
      
      // update vars for determining acceleration later
      oldVel =  velSetpoint; 
      oldTime = Timer.getFPGATimestamp(); 
      oldPos = getAbsWristPos();
  }

  public void setWristSetpoint(double goalPos) {
      wristPIDController.setSetpoint(goalPos);
      this.wristTarget = goalPos;
      
  }

  public void toggleWrist() {
      wristPIDRun = !wristPIDRun;
  }

  public double getWristPos() { return wrist.getEncoder().getPosition(); }
  public double getAbsWristPos(){
    return wristEncoder.getPosition();
  }
  public void stopWrist() { wrist.stopMotor(); }
  public void toggleWristBrake() { wrist.setIdleMode(wristBrakeToggle ? IdleMode.kBrake : IdleMode.kCoast); wristBrakeToggle = !wristBrakeToggle; }
  




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    reachPos = Math.abs(getAbsWristPos() - this.wristTarget) > 0.08;
    if(runStuff) {
      if(wristPIDRun){
        updateWristPos();
      }else{
        wrist.setVoltage(0);
      }
    }else{
      wrist.setVoltage(0);
    }

    SmartDashboard.putNumber("Wrist Pos", getAbsWristPos());
    SmartDashboard.putBoolean("Wrist is Brake", wrist.getIdleMode() == IdleMode.kBrake ? true : false);
    SmartDashboard.putBoolean("Wrist Enabled", wristPIDRun);
    SmartDashboard.putNumber("Wrist Target", wristTarget);
    if (SmartDashboard.getNumber("Change Wrist Is Brake", 1) == 1) {
      wrist.setIdleMode(IdleMode.kBrake);
    } else {
      wrist.setIdleMode(IdleMode.kCoast);
    }
  }




  public double convertToRads(double angle) {
    return angle/360*2*Math.PI;
  }

  // --------------------------------------------------------------

  public void changeHeight(Constants.Height height){
      lastHeight = height;
  }

  public Constants.Height getHeight(){
      return lastHeight;
  }
  private boolean ground = false;
  private boolean source = true;
  public void goToPosition(Constants.Height pos) {
             

      if(ground && pos == Constants.Height.AMP){
        lastHeight = Constants.Height.AMP;
          // setArmWristExtGoal(0.531, 0.15, 0.25);
          setWristSetpoint(wristHardLowerLimit + Constants.Wrist.offsetToAmpFromGround); // wrist from smallest 0.117

      }else if(source && pos == Constants.Height.AMP){
        lastHeight = Constants.Height.AMP;
          // setArmWristExtGoal(0.511, 0.0487, 0.34); //extTarget = 0.387
          // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
          setWristSetpoint(wristHardLowerLimit + Constants.Wrist.offsetToAmpFromSource_Hold); //extTarget = 0.387

      }else if(pos == Constants.Height.HOLDTOGROUND){
          // lastHeight gets updated for this in the periodic method
          ground = true;
          source = false;
          lastHeight = Constants.Height.GROUND;
          // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
          // move arm to purpendicular (0.21), then move extension and wrist simultaneously while moving arm down
          // this boolean is a way to determine the "range" for when stuff starts moving after the arm, as well as some logic
          wristPIDRun = false;
          
          // the below will make arm go to 90 degree pos (logic in periodic method)
          setWristSetpoint(wristHardLowerLimit + Constants.Wrist.offsetToGround); //extTarget = 0.387

      }else if(pos == Constants.Height.GROUNDTOHOLD){
          // lastHeight gets updated for this in the periodic method
          ground = true;
          source = false;
          lastHeight = Constants.Height.HOLD;
          // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
          // move arm to purpendicular (0.21), then move extension and wrist simultaneously while moving arm down
          // this boolean is a way to determine the "range" for when stuff starts moving after the arm, as well as some logic
          wristPIDRun = false;
          
          // the below will make arm go to 90 degree pos (logic in periodic method)
          setWristSetpoint(wristHardLowerLimit + Constants.Wrist.offsetToHold); //extTarget = 0.387

      }else if(pos == Constants.Height.GROUND){
          lastHeight = Constants.Height.GROUND;
          ground = true;
          source = false;
      
          // setArmWristExtGoal(0.1716, 0.51, 0.463); //extTarget = 0.387
          setWristSetpoint(wristHardLowerLimit + Constants.Wrist.offsetToGround); //extTarget = 0.387

      }else if(pos == Constants.Height.HOLD){
          lastHeight = Constants.Height.HOLD;
      
          // setArmWristExtGoal(0.13, 0.05, 0.73); 
          setWristSetpoint(wristHardLowerLimit + Constants.Wrist.offsetToHold); 

      }else if(pos == Constants.Height.SOURCE){
          lastHeight = Constants.Height.SOURCE;
          ground = false;
          source = true;
          // setArmWristExtGoal(0.39, 0.42, 0.55); //extTarget = 0.5346 wristTarget = 0.33
          // setArmWristExtGoal(0.37, 0.387, 0.55); //extTarget = 0.5346
          setWristSetpoint(wristHardLowerLimit + Constants.Wrist.offsetToSource); //extTarget = 0.5346

      }
      
      
    


  }
}
