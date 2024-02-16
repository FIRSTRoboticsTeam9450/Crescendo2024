// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
 
  private Constants.Height lastHeight = Constants.Height.HOLD;
  private ExtensionSubsystem extSub;
  
  private double armTarget = 0.37;//0.485;  //.453

  private double armHardLowerLimit = 0.105;//0.08;
  private double armHardUpperLimit = 0.7;//0.51;

  private double armStraightUp = 0.46;
  private double armBalanced = 0.36;
  private double armPurpenGround = 0.206;
  private double armFFWhenPurpen = 0.42;
  private double extHardLowerLimit = 0.749; 
  private double extHardUpperLimit = 0.059;

  boolean runStuff = true;
  boolean armPIDRun;
  boolean reachPos;

  private CANSparkMax armMotor = new CANSparkMax(Constants.armId, MotorType.kBrushless);
  private SparkAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

  DoubleSupplier armFFkg = () -> 0.065;
  private ArmFeedforward armFF = new ArmFeedforward(0, 0.065,0); //0.027, 0.00001 => halfway is 0.013505

  private final ProfiledPIDController armPid = new ProfiledPIDController(40, 0, 0, new Constraints(4, 3));//maxVel = 3.5 and maxAccel = 2.5

  

  public ArmSubsystem() {
    armMotor.restoreFactoryDefaults();
    armMotor.setSmartCurrentLimit(40);
    armMotor.setIdleMode(IdleMode.kCoast);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
    // armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
    // armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
    // armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency  
    armMotor.burnFlash();
    armPIDRun = true;

    goToPosition(Constants.Height.SOURCE);

    SmartDashboard.putNumber("Change Arm Target", armTarget);
    SmartDashboard.putNumber("Change ArmFF", 0.065);
    SmartDashboard.putNumber("Change Arm Is Brake", 1);


  }
   public ArmSubsystem(ExtensionSubsystem extSub) {
    armMotor.restoreFactoryDefaults();
    armMotor.setSmartCurrentLimit(40);
    armMotor.setIdleMode(IdleMode.kCoast);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
    // armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
    // armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
    // armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency  
    armMotor.burnFlash();

    this.extSub = extSub;
    armPIDRun = true;

    goToPosition(Constants.Height.SOURCE);

    SmartDashboard.putNumber("Change Arm Target", armTarget);
    SmartDashboard.putNumber("Change ArmFF", 0.065);
    SmartDashboard.putNumber("Change Arm Is Brake", 1);


  }


  
  public double getAbsArmPos(){
    return armEncoder.getPosition();
  }

  


  /**
   * For equation derivation, see {@link https://www.desmos.com/calculator/ygpschqwqe}
   * @return returns a linear modifier from 1 to 1.4375 to multiply the armFF equation by to account for extension
   */
  public double updateArmFF_extension(){
      //return (-0.16134 * extPosition + armFFkg.getAsDouble());
      // return -0.56966 * getExtensionAbsPosition() + 1.39876302083; // old b value was 1.52295
      double slope = (1 - 1.4375) / (extHardLowerLimit - extHardUpperLimit);
      return slope * extSub.getExtensionAbsPosition() + (1 - slope * extHardLowerLimit);
  }


  // cosine equation FF, see https://www.desmos.com/calculator/ygpschqwqe
  public double getFFEquationVoltage() {
      
      return  armFFWhenPurpen * updateArmFF_extension() * Math.cos((2*Math.PI/((armBalanced - armPurpenGround)*4)) * (getAbsArmPos() - armPurpenGround)); 
  } 
  public void setArmVoltage(double voltage){
      // leftMotor.setVoltage(-voltage);

    armMotor.setVoltage(voltage);
  }
  public void downManual(){
      armTarget -= 0.01;

      //wrist.setGoal(2.57 - armTarget);
  }
  public void upManual(){
      armTarget += 0.01;

      //wrist.setGoal(2.57 - armTarget);
  }

  public void setArmGoal(double target) {
      //rotation.setGoal(target);
      armTarget = target;
  }

  public double getGoal(){
      return armPid.getGoal().position;
  }
  public double calculateRotationPID(){
      //return armPid.calculate(getArmPosition(), armTarget);
      return armPid.calculate(getAbsArmPos(), armTarget);
  }
  public void updateRotationOutput(){
      
      double ffValue = getFFEquationVoltage()/*calculateRotationFF()*/;
      double pidValue = calculateRotationPID();

      double voltage = pidValue + ffValue;
      SmartDashboard.putNumber("Rotation Voltage", voltage);

      //voltage = MathUtil.clamp(voltage, -4, 4);

      // double voltage = convertToVolts(percentOutput);
      // SmartDashboard.putNumber("percentOutput", percentOutput);
      SmartDashboard.putNumber("Rotation FF", ffValue);
      SmartDashboard.putNumber("PIDRotate", pidValue);
      
    // boolean limit = (getAbsArmPos() >= armHardUpperLimit && Math.signum(voltage) == 1.0) || (getAbsArmPos() <=  && Math.signum(voltage) == -1.0);
    // if(limit){
          //Technically should set a ff constant negative 
          //Mainly b/c of the limit on the chain rn(if gone can remove this if statment)
      //    setArmVoltage(0);
      //}else{
      if (Math.abs(voltage) < 4) {
          setArmVoltage(voltage);
      } else {
          setArmVoltage(4 * Math.signum(voltage));
      }

      //} 
  }
  public double calculateRotationFF(){
      // if(extensionTarget == 30){
      //     return armExtendedFF.calculate(getArmPosition(), armPid.getSetpoint().velocity);
      // }

      //return armFF.calculate(getArmPosition(), armPid.getSetpoint().velocity);
      //return armFF.calculate(getAbsArmPos(), armPid.getSetpoint().velocity);
      if (getAbsArmPos() >= 0.46) {
          return -armFF.calculate(getAbsArmPos(), 0);
      } else {
          return armFF.calculate(getAbsArmPos(), 0);
      }
  }

  public void toggleArm() {
    armPIDRun = !armPIDRun;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    reachPos = Math.abs(getAbsArmPos() - this.armTarget) > 0.08;
    if(runStuff) {
      if(armPIDRun){
        updateRotationOutput();
      }else{
        armMotor.setVoltage(0);
      }
    }else{
      setArmVoltage(0);
    }

    SmartDashboard.putNumber("Arm Position", getAbsArmPos());
    SmartDashboard.putNumber("Arm Target", armTarget);
    SmartDashboard.putNumber("Arm Actual Voltage", armMotor.getOutputCurrent()*0.6);
    SmartDashboard.putNumber("ArmFF kg", armFF.kg);
    SmartDashboard.putBoolean("Arm Enabled", armPIDRun);
    SmartDashboard.putNumber("FF Equation Value", getFFEquationVoltage());

    armFFkg = () -> SmartDashboard.getNumber("Change ArmFF", 0.065);
    if (SmartDashboard.getNumber("Change Arm Is Brake", 1) == 1) {
        armMotor.setIdleMode(IdleMode.kBrake);
    } else {
        armMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public double convertToRads(double angle) {
    return angle/360*2*Math.PI;
  }

  // --------------------------------------------------------------

  public void changeHeight(Constants.Height height){
      lastHeight = height;
  }
 

  public Constants.Height getLastHeight(){
      return lastHeight;
  }

  private boolean ground = false;
  private boolean source = true;
  public void goToPosition(Constants.Height pos) {
      if(ground && pos == Constants.Height.AMP){
          lastHeight = Constants.Height.AMP;
          // setArmWristExtGoal(0.531, 0.15, 0.25);
          setArmGoal(armHardLowerLimit + Constants.Arm.offsetToAmpFromGround); // wrist from smallest 0.117

      }else if(source && pos == Constants.Height.AMP){
          lastHeight = Constants.Height.AMP;
          // setArmWristExtGoal(0.511, 0.0487, 0.34); //extTarget = 0.387
          // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
          setArmGoal(armHardLowerLimit + Constants.Arm.offsetToAmpFromSource_Hold); //extTarget = 0.387

      }else if(pos == Constants.Height.GROUNDTOHOLD || pos == Constants.Height.HOLDTOGROUND){
          ground = true;
          source = false;
          
          // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
          // move arm to purpendicular (0.21), then move extension and wrist simultaneously while moving arm down
          // this boolean is a way to determine the "range" for when stuff starts moving after the arm, as well as some logic
          
          // the below will make arm go to 90 degree pos (logic in periodic method)
          setArmGoal(armHardLowerLimit + Constants.Arm.offsetToStraightOutPos); //extTarget = 0.387

      }else if(pos == Constants.Height.GROUND){
          lastHeight = Constants.Height.GROUND;
          ground = true;
          source = false;
      
          // setArmWristExtGoal(0.1716, 0.51, 0.463); //extTarget = 0.387
          setArmGoal(armHardLowerLimit + Constants.Arm.offsetToGround); //extTarget = 0.387

      }else if(pos == Constants.Height.HOLD){
          lastHeight = Constants.Height.HOLD;
      
          // setArmWristExtGoal(0.13, 0.05, 0.73); 
          setArmGoal(armHardLowerLimit + Constants.Arm.offsetToHold); 

      }else if(pos == Constants.Height.SOURCE){
          lastHeight = Constants.Height.SOURCE;
          ground = false;
          source = true;
          // setArmWristExtGoal(0.39, 0.42, 0.55); //extTarget = 0.5346 wristTarget = 0.33
          // setArmWristExtGoal(0.37, 0.387, 0.55); //extTarget = 0.5346
          setArmGoal(armHardLowerLimit + Constants.Arm.offsetToSource); //extTarget = 0.5346

      }
      
      

    
  }
}
