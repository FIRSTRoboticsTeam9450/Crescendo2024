// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class HeadingCorTeleopDrive extends Command
{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier headingHorizontal, headingVertical;
  private final BooleanSupplier zeroGyro, speedModify;
  private double xGoal, yGoal, speedModifier;
  private boolean initRotation = false;
  private boolean isDrifting = false;
  private boolean noRotation;

  ChassisSpeeds desiredSpeeds;


  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve            The swerve drivebase subsystem.
   * @param vX                DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1
   *                          to 1 with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY                DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1
   *                          to 1 with deadband already accounted for.  Positive Y is towards the left wall when
   *                          looking through the driver station glass.
   * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's heading angle. In the
   *                          robot coordinate system, this is along the same axis as vY. Should range from -1 to 1 with
   *                          no deadband.  Positive is towards the left wall when looking through the driver station
   *                          glass.
   * @param headingVertical   DoubleSupplier that supplies the vertical component of the robot's heading angle.  In the
   *                          robot coordinate system, this is along the same axis as vX.  Should range from -1 to 1
   *                          with no deadband. Positive is away from the alliance wall.
   */
  public HeadingCorTeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal,
                       DoubleSupplier headingVertical, BooleanSupplier zeroGyro, BooleanSupplier speedModify)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;
    this.zeroGyro = zeroGyro;
    this.speedModify = speedModify;

    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    initRotation = true;
    noRotation = true;
    isDrifting = false;

    xGoal = 0.0;
    yGoal = 0.0;
    speedModifier = 1.0;

    desiredSpeeds = swerve.getTargetSpeeds(-vX.getAsDouble(), -vY.getAsDouble(),
                                                         0,
                                                         0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // for zeroing the gyro
    if (zeroGyro.getAsBoolean()) {
      swerve.zeroGyro();
      xGoal = 0.0;
      yGoal = 0.0;
    }

    // for speedModifier drive enabled
    if (speedModify.getAsBoolean()) {
        speedModifier = 0.7; // instead of 0.5, because drive utilizes a cubic function for speed
    } else {
        speedModifier = 1.0; // if the speedModify boolean isn't toggled, then use regular speed
    }
      
    // Get the desired chassis speeds based on a 2 joystick module.
    desiredSpeeds = swerve.getTargetSpeeds(-vX.getAsDouble() * speedModifier, -vY.getAsDouble() * speedModifier,
                                                         xGoal,
                                                         yGoal);

    
    if (initRotation) {
      if(Math.abs(headingHorizontal.getAsDouble()) <= Constants.OperatorConstants.LEFT_X_DEADBAND && Math.abs(headingVertical.getAsDouble()) <= Constants.OperatorConstants.LEFT_Y_DEADBAND)
      {
        // Get the curretHeading
        Rotation2d firstLoopHeading = swerve.getHeading();

        // System.out.println(firstLoopHeading.toString());
      
        // Set the Current Heading to the desired Heading
        desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());

      } 

      initRotation = false;

    }
    
    

    // Limit velocity to prevent tippy (currently unused, may be used in future)
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());
    //SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    //SmartDashboard.putString("Translation", translation.toString());


    double xVelocity   = -Math.pow(vX.getAsDouble(), 3) * speedModifier;
    double yVelocity   = -Math.pow(vY.getAsDouble(), 3) * speedModifier;
    double angVelocity = -Math.pow(headingHorizontal.getAsDouble(), 3) * speedModifier;
    // SmartDashboard.putNumber("vX", xVelocity);
    // SmartDashboard.putNumber("vY", yVelocity);
    // SmartDashboard.putNumber("omega", angVelocity);

    // Drive using raw values.
    // swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
    //              angVelocity * swerve.getSwerveController().config.maxAngularVelocity,
    //              true);

    // for horizontal rotation
    
    if (Math.abs(headingHorizontal.getAsDouble()) <= Constants.OperatorConstants.RIGHT_X_DEADBAND && Math.abs(vX.getAsDouble()) <= Constants.OperatorConstants.LEFT_Y_DEADBAND && Math.abs(vY.getAsDouble()) <= Constants.OperatorConstants.LEFT_Y_DEADBAND) {
      /* If Robot not moving */
      swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity * swerve.getSwerveController().config.maxAngularVelocity,
                 true);

      swerve.setHeadingCorrection(false);

      // System.out.println("No Drive");

    } else  if (!isDrifting && Math.abs(headingHorizontal.getAsDouble()) <= Constants.OperatorConstants.LEFT_X_DEADBAND && Math.abs(headingVertical.getAsDouble()) <= Constants.OperatorConstants.LEFT_Y_DEADBAND) {
      /* For drive without rotation */
      swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed), desiredSpeeds.omegaRadiansPerSecond, true);
      
      swerve.setHeadingCorrection(true);
      // System.out.println("Drive NO Rotation");


    } else {
      /* For rotation */
      swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity * swerve.getSwerveController().config.maxAngularVelocity,
                 true);

      swerve.setHeadingCorrection(false);
      // System.out.println("Drive + Rotation");

      // updates the drifting bool to determien when to update goalRot speed
      isDrifting = true;
      

    }

    if (isDrifting) {
      if (Math.abs(swerve.getRobotVelocity().omegaRadiansPerSecond) < 0.1) {
        // updates the goal angle (polar)
        xGoal = swerve.getHeading().getSin();
        yGoal = swerve.getHeading().getCos();
        isDrifting = false; 
      }
    }

    // if (((Math.abs(headingVertical.getAsDouble()) > Math.abs(headingHorizontal.getAsDouble()))   
    //          && (Math.abs(headingHorizontal.getAsDouble()) < 0.2) 
    //          && (Math.abs(headingVertical.getAsDouble()) > 0.3)) 
    //          || noRotation){

    //   swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);

    // } else {
    //   swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
    //              angVelocity * swerve.getSwerveController().config.maxAngularVelocity,
    //              true);
    // }

    

    // Make the robot move
    // swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);

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
