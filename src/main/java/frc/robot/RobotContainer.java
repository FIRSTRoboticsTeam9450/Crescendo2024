// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakingCommand;
import frc.robot.commands.ResetClimbCommand;
import frc.robot.commands.ScoringCommand;
import frc.robot.commands.TimedIntakeSetPowerCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Scoring;

public class RobotContainer {
  public final Scoring score = new Scoring();
  public final Limelight servo = new Limelight(score);
  public final Climb climbSub = new Climb();

  // try switching maxspeed and maxangularspeed to see what happens
  private double MaxSpeed = 5.22;/*TunerConstants.kSpeedAt12VoltsMps;*/ // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // all joysticks initially negative (removed)
  CommandXboxController armController = new CommandXboxController(1);
  CommandXboxController testingController = new CommandXboxController(2);
  XboxController driverXbox = new XboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with Brian added - sign
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left) brian added - sign
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(driverController.getLeftY(), driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    driverController.povDown().onTrue(new InstantCommand(() -> 
      score.ext.setTargetInches(score.ext.getTargetInches() - 2)));

    driverController.povUp().onTrue(new InstantCommand(() -> 
      score.ext.setTargetInches(score.ext.getTargetInches() + 2)));
 


    // ARM
    armController.leftTrigger().onTrue(new ScoringCommand(score, Constants.ScoringPos.AMP));

    // /* hold */
    armController.rightTrigger().onTrue(new SequentialCommandGroup(
        new TimedIntakeSetPowerCommand(score, 10, 0.75),
        new ScoringCommand(score, Constants.ScoringPos.STORE)));
    
    // source
    armController.y().onTrue(new SequentialCommandGroup(
        new ScoringCommand(score, Constants.ScoringPos.SOURCE),
        new IntakingCommand(score, 8),
        new ScoringCommand(score, Constants.ScoringPos.CLIMB),
        new InstantCommand(() -> score.ext.runAndResetEncoder())));
    
    // Preclimb
    armController.b().onTrue(new ScoringCommand(score, Constants.ScoringPos.CLIMB));

    /* ground, intake, hold */
    armController.a().onTrue(new SequentialCommandGroup(
        new ScoringCommand(score, Constants.ScoringPos.GROUND),
        new IntakingCommand(score, 12),
        new ScoringCommand(score, Constants.ScoringPos.STORE)));
    
    armController.x().onTrue(new ScoringCommand(score, Constants.ScoringPos.STORE));

    // Climber
    armController.pov(180).onTrue(new ClimbCommand(climbSub, 0));
    armController.pov(90).onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> score.arm.setState(Constants.RobotState.CLIMBING)),
        new ScoringCommand(score, Constants.ScoringPos.STORE), 
        new WaitCommand(0.5),
        new ClimbCommand(climbSub, 90)
    ));
    armController.pov(0).onTrue(new ParallelCommandGroup(
        new ClimbCommand(climbSub, 90),
        new ScoringCommand(score, Constants.ScoringPos.CLIMB)
    ));
    armController.pov(270).onTrue(new SequentialCommandGroup(
        new ClimbCommand(climbSub, 25),
        new InstantCommand(() -> score.arm.setState(Constants.RobotState.DEFAULT))
    ));

    armController.leftBumper().onTrue(new ResetClimbCommand(climbSub));

    armController.rightBumper().onTrue(new InstantCommand(() -> score.ext.runAndResetEncoder()));

    armController.rightStick().onTrue(new InstantCommand( () -> score.setUseVelocityIntake(!score.getUseVelocityIntake()))); // I think this is on click

    /* GET DRIVE MOTOR ETC.
    SwerveDrivetrain drive = new SwerveDrivetrain(null, null);
    drive.getModule(0).getDriveMotor();
    */
    
  }

  public RobotContainer() {
    configureBindings();
  }

  public void logDriveStickyFaults() {
    for (int i = 0; i < 3; i++) {
      //BitToStickyfaultString.getStickyFaultString(drivetrain.getModule(i).getDriveMotor().getStickyFaultField()); // max 24 bit instead of 16 
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
