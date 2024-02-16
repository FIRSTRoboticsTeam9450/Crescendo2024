// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ExamplesSubsystem extends SubsystemBase {
  /** Creates a new ExamplesSubsystem. */
  public ExamplesSubsystem() {}

  // Making a motor a absolute encoder
  // if absolute encoder plugged into cansparkmax:
  // SparkMaxAbsoluteEncoder turretEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);


  // EXAMPLE PID FOR COPY PASTING  
  // PIDController + SimpleMotorFeedforward
  // Includes essential methods and motors etc.
  // class Example_PID {
  //   CANSparkMax motor = new CANSparkMax(35, MotorType.kBrushless); //replace Constants.turretId with correct Id
  //   private final PIDController turretPIDController =
  //     new PIDController(0, 0, 0); // add p value
  //   public SimpleMotorFeedforward turretFeedForward = new SimpleMotorFeedforward(0, 0, 0); 
    
  //   double oldVel = 0;
  //   double oldTime = 0;
  //   double oldAngle = 0;
    
  //   public void updateTurretAngle(double goalAngle, double hasTarget) {
  //     double pidValue = turretPIDController.calculate(getPosition(), goalAngle);
  //     double changeInTime = Timer.getFPGATimestamp() - oldTime;
  //     double velSetpoint = (getPosition() - oldAngle) / changeInTime;
  //     double accel = (velSetpoint - oldVel) / (changeInTime); 
  //     double ffVal = turretFeedForward.calculate(velSetpoint, accel); //takes velocity, and acceleration
      
  //     double percentOutput = MathUtil.clamp(pidValue + ffVal, -1.0, 1.0);
  //     double voltage = convertToVolts(percentOutput);
      
      
  //     SmartDashboard.putNumber("PID Value", pidValue);
  //     SmartDashboard.putNumber("Feed Forward", ffVal);
  //     SmartDashboard.putNumber("Voltage", voltage);
  //     SmartDashboard.putNumber("Position error", turretPIDController.getPositionError());
    

  //     if (Math.abs(getPosition() - goalAngle) <= 0.3 /*degrees*/) { // no voltage
  //       motor.setVoltage(0);
  //     } else { // set voltage
  //       if (getPosition() <= Constants.maxTurretPosition && getPosition() >= Constants.minTurretPosition) { // is between max and min
  //         motor.setVoltage(voltage); // could be negative voltage based upon direction of motor and gear
  //       }
  //     }
      
  //     // update vars for determining acceleration later
  //     oldVel =  velSetpoint; 
  //     oldTime = Timer.getFPGATimestamp(); 
  //     oldAngle = getPosition();
  //   }
    
  //   public double getPosition() {
  //     return motor.getEncoder().getPosition();
  //   }
    
  //   private double convertToVolts(double percentOutput){
  //     return percentOutput * Robot.getInstance().getVoltage();
  //   }

  // }
  
  // EXAMPLE LIMELIGHT network tables --> getting the values
  class Example_Limelight {
    // Limelight values
    double x;
    double y;
    double area;
    double id;
    double hasTarget;

    // obtaining data with network tables for limelight; see here: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api 
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx"); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) (prob bigger range b/c LL3)
    NetworkTableEntry ty = table.getEntry("ty"); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) (prob bigger range b/c LL3)
    NetworkTableEntry ta = table.getEntry("ta"); // 	Target Area (0% of image to 100% of image)
    NetworkTableEntry tagID = table.getEntry("tid"); // 	id of primary tag in view
    NetworkTableEntry tv = table.getEntry("tv"); // returns either 0 or 1 if there is any tag in frame  
    //returns vision derived pose
    double[] visionPose = table.getEntry("botpose").getDoubleArray(new double[6]); // Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
  

    public double getDistance() {
      double targetOffsetAngle_Vertical = y;
  
      // how many degrees back is your limelight rotated from perfectly vertical?
      double limelightMountAngleDegrees = 25.0; 
  
      // distance from the center of the Limelight lens to the floor
      double limelightLensHeightInches = 20.0; 
  
      // distance from the target to the floor
      double goalHeightInches = 60.0; 
  
      double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
  
      //calculate distance
      double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
      return distanceFromLimelightToGoalInches;
    }

    public double[] getVisionPose() {
      return visionPose;
    }

    // BELOW GOES IN THE @Override periodic() METHOD!!!!!!!!!!!!!!!!!
    public void periodic() {
       //read values periodically
      x = tx.getDouble(0.0);
      y = ty.getDouble(0.0);
      area = ta.getDouble(0.0);
      id = tagID.getDouble(0.0);
      hasTarget = tv.getDouble(0.0);
      visionPose = table.getEntry("botpose").getDoubleArray(new double[6]);
    }

  }

  










  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}