// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */

  Servo axon;

  Scoring score;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  boolean ampMode;

  boolean liningUp;

  MedianFilter median;

  public Limelight(Scoring score) {
    axon = new Servo(0);
    SmartDashboard.putNumber("set axon angle", 180);
    this.score = score;
    median = new MedianFilter(3);
    resetAngle();
  }

  /**
   * Set the servo angle.
   *
   * <p>
   * The angles are based on the HS-322HD Servo, and have a range of 0 to 180
   * degrees.
   *
   * <p>
   * Servo angles that are out of the supported range of the servo simply
   * "saturate" in that
   * direction In other words, if the servo has a range of (X degrees to Y
   * degrees) than angles of
   * less than X result in an angle of X being set and angles of more than Y
   * degrees result in an
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
   * <p>
   * Servo positions range from 0.0 to 1.0 corresponding to the range of full left
   * to full right.
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

    // angles -> 130 facing angle forward, 180 facing angled backward, 107
    // approximately straight forward

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
    if (score.getLaserDistance() < 50) {
      axon.setAngle(180);
    } else {
      axon.setAngle(130);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double medianValue = median.calculate(score.getLaserDistance());
    // double medianValue = 0;

    if (DriverStation.isAutonomous()) {
      table.getEntry("pipeline").setNumber(0);
      ampMode = true;
    }

    // if (DriverStation.isDisabled()) {
    //   double[] robotPose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    //   if (robotPose[4] > -0.5 && robotPose[4] < 0.5 && table.getEntry("tid").getDouble(-1) != -1) {
    //     if (table.getEntry("ledMode").getInteger(-1) != 3) {
    //       table.getEntry("ledMode").setNumber(3);
    //     }

    //   } else {
    //       table.getEntry("ledMode").setNumber(0);
    //   }
    // } else {
    //   table.getEntry("ledMode").setNumber(0);
    // }

    SmartDashboard.putNumber("laser distance", medianValue);
    if (score.getIntakeState() == Constants.IntakeState.HAS_NOTE || DriverStation.isAutonomous() || DriverStation.isDisabled()) {
      if (table.getEntry("pipeline").getInteger(-1) == 1) {
        table.getEntry("pipeline").setNumber(0);
        axon.setAngle(180);
        ampMode = true;
      }
    } else {
      if (table.getEntry("pipeline").getInteger(-1) == 0) {
        table.getEntry("pipeline").setNumber(1);
        axon.setAngle(130);
        ampMode = false;
      }
    }

    // setAxonAngle(SmartDashboard.getNumber("set axon angle", 107));
    SmartDashboard.putNumber("Axon angle", getAxonAngle());
  }
}
