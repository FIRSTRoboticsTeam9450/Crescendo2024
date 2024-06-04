// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double LOOP_TIME  = 0.02; //s, 20ms + 110ms sprk max velocity lag

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 0.75;
  }

  public static class Extension{
    // in inches
    public static final double groundExtPosition = 5;
    public static final double storeExtPosition = 0;
    public static final double sourceExtPosition = 1.5;
    public static final double ampExtPositionFromSource = 8.5;
    public static final double ampExtPositionFromGround = 10;
    public static final double trapExtPosition = 12;
    public static final double climbExtPosition = 0;
    
    public static final double extHardwareMax = 12;
    public static final double extHardwareMin = 0;

    public static final double resetExtVoltage = -3;

    // 12 is for gear ratio
    // might need to consider offset of 0.4
    public static final double convertToInches = .93;

  }
   public static class Arm{
    // In degrees
    public static final double groundArmPosition = 72;
    public static final double storeArmPosition = 55;
    public static final double sourceArmPosition = 135;
    public static final double ampArmPositionFromSource = 200;
    public static final double ampArmPositionFromGround = 200;
    public static final double trapArmPosition = 200;
    public static final double climbArmPosition = 170;
    public static final double tempArmPosition = 80;

    /**converts from degrees to rotations */
    public static final double RotateConversionFactor = (1.0/360);
    public static final double AbsEncoderShift = 0.1177;
    // 180 -> 0.55 abs encoder
    // 90 -> 0.2946 abs encoder

    public static final double armHardwareMax = 225;
    public static final double armHardwareMin = 55;
  }
   public static class Wrist{
    // In degrees
    public static final double groundWristPosition = 130;
    public static final double storeWristPosition = 270;
    public static final double sourceWristPosition = 180;
    public static final double ampWristPositionFromSource = 250;
    public static final double ampWristPositionFromGround = 240;
    public static final double trapWristPosition = 240;
    public static final double climbWristPosition = 180;
    
    public static final double gearRatio = 1.2/1;
    /**converts from degrees to rotations */
    public static final double RotateConversionFactor = -(gearRatio/360);
    public static final double AbsEncoderShift = -1.171;

    public static final double wristHardwareMax = 270;
    public static final double wristHardwareMin = 90;
    
  }

  public static enum ScoringPos{
    NONE,
    GROUND,
    STORE,
    SOURCE,
    AMP,
    CLIMB,
    TRAP,
    TEMP
  }

  public static enum IntakeState {
    HAS_NOTE,
    NO_NOTE
  }

  public static enum RobotState {
    DEFAULT,
    CLIMBING
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

  public static final int armFrontId = 22;
  public static final int armBackId = 33;
  public static final int armEncoderId = 21;

  public static final int extensionId = 23;

  public static final int wristId = 24;

  public static final int intakeId = 26;

  public static final int lClimberId = 29;
  public static final int rClimberId = 31;
  
  public static final int laserId = 32;

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
  public static double maxWristPos = 0.97 - 0.02;
  public static double minWristPos = 0.421 + 0.02;//0.68
  public static double midWristPos = 0.68;

  public static double maxWristPOS = 0.632;
  public static double minWristPOS = 0.033;
  public static double maxExtPos = 0.06;
  public static double minExtPos = 0.75;
  public static double maxArmPos = 0.7;
  public static double minArmPos = 0.105;
  
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
