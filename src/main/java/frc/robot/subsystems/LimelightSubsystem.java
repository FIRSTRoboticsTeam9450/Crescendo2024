// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  
  Servo axon;

  IntakeSubsystem intake;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  boolean ampMode;

  MedianFilter median;

  public LimelightSubsystem(IntakeSubsystem intake) {
    axon = new Servo(0);
    SmartDashboard.putNumber("set axon angle", 180);
    setAxonAngle(180);
    this.intake = intake;
    median = new MedianFilter(3);
  }

  /**
   * Set the servo angle.
   *
   * <p>The angles are based on the HS-322HD Servo, and have a range of 0 to 180 degrees.
   *
   * <p>Servo angles that are out of the supported range of the servo simply "saturate" in that
   * direction In other words, if the servo has a range of (X degrees to Y degrees) than angles of
   * less than X result in an angle of X being set and angles of more than Y degrees result in an
   * angle of Y being set.
   *
   * @param degrees The angle in degrees to set the servo.
   */
  public void setAxonAngle(double degrees) {
    axon.setAngle(degrees);
  }

  /**
   * Set the servo position.
   *
   * <p>Servo positions range from 0.0 to 1.0 corresponding to the range of full left to full right.
   *
   * @param position Position from 0.0 to 1.0.
   */
  public void set(double position) {
    axon.set(position);
  }

  /**
   * returns the last commanded angle, not the current angle of the axon
   */
  public double getAxonAngle() {

    // angles -> 130 facing angle forward, 180 facing angled backward, 107 approximately straight forward

    return axon.getAngle();
  }

  public void runThroughAllAngles() {
    for (int i = 0; i < 360; i++) {
      setAxonAngle(i);
    }
  }

  public boolean isInAmpMode() {
    return ampMode;
  }

  public void resetAngle() {
    if (intake.getLaserDistance() < 15) {
      axon.setAngle(180);
    } else {
      axon.setAngle(130);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double medianValue = median.calculate(intake.getLaserDistance());
    //double medianValue = 0;

    SmartDashboard.putNumber("laser distance", medianValue);
    if (medianValue < 12) {
      if (table.getEntry("pipeline").getInteger(-1) == 1 || DriverStation.isAutonomous()) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        axon.setAngle(180);
        ampMode = true;
      }
    } else {
      if (table.getEntry("pipeline").getInteger(-1) == 0) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
        axon.setAngle(130);
        ampMode = false;
      }    
    }
    
    //setAxonAngle(SmartDashboard.getNumber("set axon angle", 107));
    SmartDashboard.putNumber("Axon angle", getAxonAngle());
  }
}
