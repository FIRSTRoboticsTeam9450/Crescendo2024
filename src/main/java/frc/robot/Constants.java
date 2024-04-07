// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.02; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 1;
  }

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
    public static final int EncoderResolution = 7168; // tics per revolution
    public static final double DRIVE_MOTOR_CONVERSION = 2 * Math.PI * WheelRadius; // distance 
    public static final double TURNING_MOTOR_CONVERSION = 2 * Math.PI; // distance 
    public static final double VELOCITY_CONVERSION_FACTOR = Math.PI * WheelRadius / 30; //Distance per second
  }

  public static class Speeds{
    public static final double MaxSpeed = 21;
    public static final double MaxAngularSpeed = 4*Math.PI;
  }

  //Extension Limiting Equation Constants----------------------------
  public static class Chassis{
    public static final double pivotToFront = 17;
    public static final double pivotToBack = 13.75;
    public static final double pivotHeight = 19;
  }

  public static class Arm{
    public static final double armLength = 18;
    public static final double intakeArmTics = 0.1716;
    public static final double intakeArmAngle = -20;
    public static final double ampArmTics = 0.531;
    public static final double ampArmAngle = 115;

    public static final double offsetToAmpFromGround = 0.416;
    public static final double offsetToAmpFromSource_Hold = 0.386;
    public static final double offsetToStraightOutPos = 0.125;//0.195
    public static final double offsetToGround = 0.09;//0.102
    public static final double offsetToHold = 0.05545 /* 3/4 inch from plate!! */; // 0.035
    public static final double offsetToSource = 0.2651; //0.255

    public static final double offsetToPreClimb = 0.356;

    public static final double offsetToTrapFromGround = 0.396;
  }

  public static class MovementLimits{
    public static final double armHardLowerLimit = 0.12455;//0.08;
    public static final double armHardUpperLimit = 0.7;//0.51;
    public static final double wristHardLowerLimit = 0.208; //      0.234
    public static final double wristHardUpperLimit = 0.862; //    0.8
    public static final double extHardLowerLimit = 0; // 0.749
    public static final double extHardUpperLimit = -75 / 2.0833; // 0.059
  }

  public static class Extension{
    public static final double maxExtensionInches = 12; //14.5
    public static final double maxExtensionTics = 0.06; 
    public static final double zeroTics = 0.73;

    public static final double offsetToAmpFromGround = (0.5/0.69 * -75 + 20 - 24) / 2.0833 + 1; // -0.5
    public static final double offsetToAmpFromSource_Hold = (0.409/0.69 * -75 + 6 - 16) / 2.0833; // -0.409
    /**Not updated */
    public static final double offsetToGroundFromHold = 0;
    public static final double offsetToGround = 0.287/0.69 * -75 / 2.0833; // -0.287
    public static final double offsetToHold = 0.02/0.69 * -75 / 2.0833; // -0.02
    public static final double offsetToSource = (0.2/0.69 * -75 + 5 + 4) / 2.0833; // -0.2
    
    public static final double offsetToTrapFromGround = (0.5/0.69 * -75 + 20 - 24 - 12) / 2.0833; // -0.5

    public static final double offsetToPreClimb = -5;




  }
  public static class NewExtension{
    // in inches
    public static final double groundExtPosition = 4;
    public static final double storeExtPosition = 0;
    public static final double sourceExtPosition = 4;
    public static final double ampExtPosition = 6;
    public static final double trapExtPosition = 13;
    public static final double climbExtPosition = 0;

    // 12 is for gear ratio
    // might need to consider offset of 0.4
    public static final double convertToInches = 4.7244/12;

  }
   public static class NewArm{
    // In degrees
    public static final double groundArmPosition = 70;
    public static final double storeArmPosition = 50;
    public static final double sourceArmPosition = 135;
    public static final double ampArmPosition = 220;
    public static final double trapArmPosition = 165;
    public static final double climbArmPosition = 180;

    /**converts from degrees to rotations */
    public static final double RotateConversionFactor = (1.0/360);
    public static final double AbsEncoderShift = 0.05;
    // 180 -> 0.55 abs encoder
    // 90 -> 0.2946 abs encoder

  }
   public static class NewWrist{
    // In degrees
    public static final double groundWristPosition = 150;
    public static final double storeWristPosition = 250;
    public static final double sourceWristPosition = 180;
    public static final double ampWristPosition = 100;
    public static final double trapWristPosition = 100;
    public static final double climbWristPosition = 180;
    
    public static final double gearRatio = 1.2/1;
    /**converts from degrees to rotations */
    public static final double RotateConversionFactor = -(gearRatio/360);
    public static final double AbsEncoderShift = -1.157;
    
  }

  public static class Wrist{
    public static final double straightWristTics = 0.36;
    public static final double upWristTics = 0.66;
    public static final double straightWristInches = 8;
    public static final double upWristInches = 2;

    public static final double offsetToAmpFromGround = 0.137 + 0.12;
    public static final double offsetToAmpFromSource_Hold = 0.02;
    /**Not updated */
    public static final double offsetToGroundFromHold = 0;
    public static final double offsetToGround = 0.44;
    public static final double offsetToHold = 0.017;
    public static final double offsetToSource = 0.279;

    public static final double offsetToTrapFromGround = 0.117;

    public static final double offsetToPreClimbFromGround = 0.367;

  }
  public static enum ScoringPos{
    NONE,
    GROUND,
    STORE,
    SOURCE,
    AMP,
    CLIMB,
    TRAP,
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
