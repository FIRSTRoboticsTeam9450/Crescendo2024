// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.lang.reflect.Array;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import swervelib.SwerveController;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDrive extends Command
{

  private final SwerveSubsystem  swerve;
  private final DoubleSupplier   vX;
  private final DoubleSupplier   vY;
  private final DoubleSupplier   omega;
  private final BooleanSupplier  zeroGyro, speedModify;
  private final SwerveController controller;
  private boolean driveMode;
  private double speedModifier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
                     boolean driveMode, BooleanSupplier zeroGyro, BooleanSupplier speedModify)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.controller = swerve.getSwerveController();
    this.zeroGyro = zeroGyro;
    this.speedModify = speedModify;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
      speedModifier = 0.8;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    
      
    // for zeroing the gyro
    if (zeroGyro.getAsBoolean()) {
      swerve.zeroGyro();
    }



    // for speedModifier drive enabled
    if (speedModify.getAsBoolean()) {
        speedModifier = 1; // instead of 0.5, because drive utilizes a cubic function for speed
    } else {
        speedModifier = 0.8; // if the speedModify boolean isn't toggled, then use regular speed
    }


    // double xVelocity   = -vX.getAsDouble() * speedModifier;
    // double yVelocity   = -vY.getAsDouble() * speedModifier;
    double angVelocity = omega.getAsDouble() * speedModifier * 0.7;


    double xVelocity   = Math.signum(vX.getAsDouble()) * Math.abs(Math.pow(vX.getAsDouble(), 2)) * speedModifier;
    double yVelocity   = Math.signum(vY.getAsDouble()) * Math.abs(Math.pow(vY.getAsDouble(), 2)) * speedModifier;
    // double angVelocity = -Math.pow(omega.getAsDouble(), 3) * speedModifier;

    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);
    
/*   
    Logger.recordOutput("SwerveStates/ControllerInputLog/TeleopDrive/x", xVelocity * SwerveSubsystem.maximumSpeed);
    Logger.recordOutput("SwerveStates/ControllerInputLog/TeleopDrive/y", yVelocity * SwerveSubsystem.maximumSpeed);
    Logger.recordOutput("SwerveStates/ControllerInputLog/TeleopDrive/z", angVelocity * controller.config.maxAngularVelocity);

    Logger.recordOutput("SwerveStates/SwerveModuleAzimuthSetpoint", Math.atan(yVelocity / xVelocity));
*/      
    

    // Drive using raw values.
    swerve.drive(new Translation2d(xVelocity * SwerveSubsystem.maximumSpeed, yVelocity * SwerveSubsystem.maximumSpeed),
                 angVelocity * controller.config.maxAngularVelocity,
                 driveMode);
    // SmartDashboard.putNumber("MaxAngVel", controller.config.maxAngularVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}

