// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LaserSubsystem extends SubsystemBase {
  private LaserCan laser;
  
  private LaserCan.Measurement measurement;
  /** Creates a new LaserSubsystem. */
  public LaserSubsystem() {
    laser = new LaserCan(Constants.laserId);

    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    try {
      laser.setRangingMode(LaserCan.RangingMode.SHORT);
      laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Laser configuration failed! " + e);
      
    }
  }


  public void wristLaserLogic() {
    if (getLaserDistance() <= 2) {
      // set slow wrist speed
    } else {
      // run intake at 
    }
  }

  public double getLaserDistance() {
    return measurement.distance_mm;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measurement = laser.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println("The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }
}
