// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.ConstantBootstraps;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExtensionCommand;
import frc.robot.Constants;

public class LimitSwitchSubsystem extends SubsystemBase {
  DigitalInput lowerHardLimSwitch;
  RelativeEncoder extRelEncoder;

  CANSparkMax extension;

  boolean runAndResetExt;
  private double extHardLowerLimit = 0; 
  private double extHardUpperLimit = -75; // -75

  /** Creates a new LimitSwitchSubsystem. */
  public LimitSwitchSubsystem() {
    lowerHardLimSwitch = new DigitalInput(2);
    

  
    extension = new CANSparkMax(Constants.extensionId, MotorType.kBrushless);
    extension.setSmartCurrentLimit(40);
    extRelEncoder = extension.getEncoder();

    extension.burnFlash();

    runAndResetExt = false;
  }


  /**
   * Returns true if the magnet is in range of the lower hard limit switch
   * @return the state of the limit switch
   */
  public boolean getLowerLimSwitch() {
    return !lowerHardLimSwitch.get();
  }

  public void setExtVoltage(double voltage) {
    extension.setVoltage(voltage);
    SmartDashboard.putNumber("Ext Lim Voltage", voltage);

  }

  /**
   * @return the relative position of the extension
   */
  public double getExtRelPos() {
    return extRelEncoder.getPosition();
  }

  /**
   * Toggles a boolean so that 
   * the periodic method runs the motor toward the lowerHardLimit at 10 volts, then 
   * will stop the motor and reset encoder after limit switch reached.
   * This method should be called in the init of the {@link ExtensionCommand}.
   */
  public void runAndResetExtEncoder() {
    runAndResetExt = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // if toggle is true, run motor [toward lowerHardLimit]
    if (runAndResetExt) {
      setExtVoltage(1); 
      runAndResetExt = false;
    }
    /* Stops motor and resets encoder after limit switch reached */
    if (getLowerLimSwitch()) {
      extension.stopMotor();
      extRelEncoder.setPosition(0);
    }


    SmartDashboard.putNumber("Ext Rel Pos", getExtRelPos());

  }
}
