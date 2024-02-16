// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.ConstantBootstraps;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExtensionCommand;
import frc.robot.Constants;

public class LimitSwitchSubsystem extends SubsystemBase {
  DigitalInput lowerHardLimSwitch;
  DigitalInput upperHardLimSwitch;
  RelativeEncoder extRelEncoder;

  CANSparkMax extension;

  boolean runAndReset;

  /** Creates a new LimitSwitchSubsystem. */
  public LimitSwitchSubsystem() {
    lowerHardLimSwitch = new DigitalInput(2);
    upperHardLimSwitch = new DigitalInput(3);
  
    extension = new CANSparkMax(Constants.extensionId, MotorType.kBrushless);
    extension.setSmartCurrentLimit(40);
    extRelEncoder = extension.getEncoder();

    runAndReset = false;
  }


  /**
   * Returns true if the magnet is in range of the lower hard limit switch
   * @return the state of the limit switch
   */
  public boolean getLowerLimSwitch() {
    return !lowerHardLimSwitch.get();
  }

  /**
   * Returns true if the magnet is in range of the upper hard limit switch
   * @return the state of the limit switch
   */
  public boolean getUpperLimSwitch() {
    return !upperHardLimSwitch.get();
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
  public void runAndResetEncoder() {
    runAndReset = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // if toggle is true, run motor [toward lowerHardLimit]
    if (runAndReset) {
      extension.setVoltage(10); 
      runAndReset = false;
    }
    /* Stops motor and resets encoder after limit switch reached */
    if (getLowerLimSwitch()) {
      extension.stopMotor();
      extRelEncoder.setPosition(0);
    }

  }
}
