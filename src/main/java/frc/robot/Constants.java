// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class ModuleConversion {
    //TOO BE UPDATEEDDDDD
    public static final double WheelDiameter = Units.inchesToMeters(4);
    public static final double DriveMotorGearRatio = 6.75;
    public static final double TurnMotorGearRatio = 150.0/7;
    public static final double DriveEncoderRot2Meter = DriveMotorGearRatio * Math.PI * WheelDiameter;
    public static final double TurnEncoderRot2Rad = TurnMotorGearRatio * 2 * Math.PI;
    public static final double DriveEncoderRPM2MeterPerSec = DriveEncoderRot2Meter / 60;
    public static final double TurnEncoderRPM2RadPerSec = TurnEncoderRot2Rad / 60;
    //ticks * 1 rotations/4096 ticks  *  gear ratio  * 6pi inches/1 rotation  * 1 ft / 12 inches
    public static final double drivetcks2ftfactor = 1.0 / 4096 * 6 * Math.PI / 12;

    public static final double WheelRadius = 0.0508;
    public static final int EncoderResolution = 4096; // tics per revolution
    public static final double DRIVE_MOTOR_CONVERSION = 2 * Math.PI * WheelRadius; // distance 
    public static final double TURNING_MOTOR_CONVERSION = 2 * Math.PI; // distance 
    public static final double VELOCITY_CONVERSION_FACTOR = Math.PI * WheelRadius / 30; //Distance per second
  }

  public static class Speeds{
    public static final double MaxSpeed = 55;
    public static final double MaxAngularSpeed = 50*Math.PI;
  }

  //Extension Limiting Equation Constants----------------------------
  public static class Chassis{
    public static final double pivotToFront = 17;
    public static final double pivotToBack = 13.75;
    public static final double pivotHeight = 19;
  }

  public static class Arm{
    public static final double armLength = 18;
    public static final double intakeArmTics = 0.12;
    public static final double intakeArmAngle = -30;
    public static final double ampArmTics = 0.518;
    public static final double ampArmAngle = 114.4;

  }

  public static class Extension{
    public static final double maxExtensionInches = 14.5;
    public static final double maxExtensionTics = 14.5;
    public static final double zeroTics = 0.918;
  }

  public static class Wrist{
    public static final double straightWristTics = 0.687;
    public static final double upWristTics = 0.987;
    public static final double straightWristInches = 8;
    public static final double upWristInches = 2;

  }

  //-----------------------------------------------------------------
  
  public static final int flDriveId = 1; //1
  public static final int flTurnId = 2; //2
  public static final int flAbsoluteId = 3;


  public static final int frDriveId = 4; //4
  public static final int frTurnId = 5; //5 
  public static final int frAbsoluteId = 6;


  public static final int blDriveId = 7; //7
  public static final int blTurnId = 8; //8
  public static final int blAbsoluteId = 9;


  public static final int brDriveId = 10; //10
  public static final int brTurnId = 11; //11
  public static final int brAbsoluteId = 12;

  public static final int gyroId = 13;

  public static final int armId = 22;
  public static final int extensionId = 23;

  public static final int wristId = 25;

  public static final int intakeId = 24;
  

  public static final double baseWidth = 0.4953;
  public static final double baseLength = 0.6477;
  
  //Offset turn wheel so that gears are all pointed to the right
  public static final double flAbsoluteEncoderOffset = 5.36431997269392 + Math.PI/2;
  public static final double frAbsoluteEncoderOffset = 5.815309412777424 + Math.PI/2; 
  public static final double blAbsoluteEncoderOffset = 2.656849354505539 + Math.PI/2;
  public static final double brAbsoluteEncoderOffset = 1.958889506757259 + Math.PI/2;


  // PID
  public static final double turnKp = 8;//4.0027 //12.0027
  public static final double turnKd = 0.10234;

  public static final double driveKp = 0.034037; //0.034037
  public static final double driveKd = 0.0;



  // feed-forward
  public static final double turnKs = 0.2246;//0.2246
  public static final double turnKv = 0.069612;
  public static final double turnKa = 0.0022882;

  public static final double driveKs = 0.084807;
  public static final double driveKv = 0.12996;
  public static final double driveKa = 0.0086263;

  // wrist stuff
  public static final double upWristBound = 0;
  public static final double lowWristBound = 0;

  //extension stuff
  public static final double maxExtension = 32; // 34.2 full max

  // trajectory things
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);

  public static final double kPXController = 1;
  public static final double kPYController = 1;
  public static final double kPThetaController = 1;

  public static final SwerveDriveKinematics kDriveKinematics = 
    new SwerveDriveKinematics(
      new Translation2d(-Constants.baseWidth / 2, Constants.baseLength / 2),
      new Translation2d(Constants.baseWidth / 2, Constants.baseLength / 2),
      new Translation2d(-Constants.baseWidth / 2, -Constants.baseLength / 2),
      new Translation2d(Constants.baseWidth / 2, -Constants.baseLength / 2)
    );
  public static double maxTurretPosition;
  public static double minTurretPosition;
  public static double maxWristPos = 0.97 - 0.1;
  public static double minWristPos = 0.399 + 0.1;

  public static boolean intakeIsOut = false;

  //Width:  19.5in 0.4953m
  //Length: 25.5in 0.6477m

  // ARM:
  // intake pos: 0.03
  // store pos: 0.35
  // score pos: 0.5

  //WRIST:
  // intake: 0.76
  // store: 0.46
  // score 
}
