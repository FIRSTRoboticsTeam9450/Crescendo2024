// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoClimbCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakingCommand;
import frc.robot.commands.ResetClimbCommand;
import frc.robot.commands.TimedIntakeSetPowerCommand;
import frc.robot.commands.armpositions.toamp.BasicToAmpCommand;
import frc.robot.commands.armpositions.toground.BasicToGroundCommand;
import frc.robot.commands.armpositions.tohold.BasicToHoldCommand;
import frc.robot.commands.armpositions.tosource.BasicToSourceCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AlignSource;
import frc.robot.commands.swervedrive.drivebase.AlignSource2;
import frc.robot.commands.swervedrive.drivebase.HeadingCorTeleopDrive;
import frc.robot.commands.swervedrive.drivebase.SweepCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimitSwitchSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmWristSubsystem.Height;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * 
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/vortex"));
                                                                         
  public final IntakeSubsystem intakeSub = new IntakeSubsystem();
  //private final ExtensionSubsystem extSub = new ExtensionSubsystem();
  //private final ArmSubsystem armSub = new ArmSubsystem(extSub);
  //private final WristSubsystem wristSub = new WristSubsystem();
  public final ArmWristSubsystem armWristSub = new ArmWristSubsystem();
  // public final LaserSubsystem laserSub = new LaserSubsystem();
  public final LimelightSubsystem servo = new LimelightSubsystem(intakeSub);
  public final ClimbSubsystem climbSub = new ClimbSubsystem();
  // private final LimitSwitchSubsystem extLimitSub = new LimitSwitchSubsystem();
 // private final ExtensionSubsystem extSub = new ExtensionSubsystem();


  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController armController = new CommandXboxController(1);
  CommandXboxController testingController = new CommandXboxController(2);
  //CommandXboxController driverController = new CommandXboxController(0);
  // CommandJoystick driverController   = new CommandJoystick(3]\[]);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  private double speedModifier = 0.67; //0.5
  private final SendableChooser<String> autoChooser;
  private BooleanSupplier speedModify;
  public boolean driveEnabled = true;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    //autoChooser = AutoBuilder.buildAutoChooser();
    //autoChooser = new SendableChooser<Command>();

    
    autoChooser = new SendableChooser<String>();
    autoChooser.addOption("Drive Forward (no scoring)", "ExitStarting");
    autoChooser.addOption("Preload Only", "PreloadScoreAlign");
    autoChooser.addOption("Preload + Center", "PreloadGrabCenter");
    autoChooser.addOption("Preload + Close", "TwoNoteAlign");

    autoChooser.setDefaultOption("Preload + Close", "TwoNoteAlign");
    


    SmartDashboard.putData("Auto Chooser", autoChooser);

    Command armStore = new BasicToHoldCommand(armWristSub);
    Command armAmp = new BasicToAmpCommand(armWristSub);
    Command armGround = new SequentialCommandGroup(
      new BasicToGroundCommand(armWristSub),
      new IntakingCommand(intakeSub, 12)
    );
    // Command armStart = new InstantCommand(() -> armWristSub.setArmWristExtGoal(armWristSub.armHardLowerLimit + Constants.Arm.offsetToAmpFromGround - 0.05, 
    // armWristSub.wristHardLowerLimit + Constants.Wrist.offsetToSource, 
    // armWristSub.extHardLowerLimit - 5));

    Command outtake = new TimedIntakeSetPowerCommand(intakeSub, 10, 0.75);
    Command resetExt = new InstantCommand(() -> armWristSub.runAndResetExtEncoder());
    Command align = new AlignSource2(drivebase, null, servo, true).withTimeout(2);
    Command armUp = new InstantCommand(() -> armWristSub.goToPosition(Height.PRECLIMB));
    // Command sweep = new SweepCommand(drivebase);

    // NamedCommands.registerCommand("ArmStart", armStart);
    NamedCommands.registerCommand("ArmStore", armStore);
    NamedCommands.registerCommand("ArmGround", armGround);
    NamedCommands.registerCommand("ArmAmp", armAmp);
    NamedCommands.registerCommand("Outtake", outtake);  
    NamedCommands.registerCommand("ResetExt", resetExt); 
    NamedCommands.registerCommand("AlignAmp", align);
    NamedCommands.registerCommand("ArmUp", armUp);
    // NamedCommands.registerCommand("Sweep", sweep);
    // speedModify = () ->  driverController.leftTrigger().getAsBoolean(); // accounts for buttons.run() overrun issue by caching value

    HeadingCorTeleopDrive drvHeadingCorr = new HeadingCorTeleopDrive(drivebase, 
                                                () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                                                () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                                                () -> driverController.getRightX(), () -> driverController.getRightY(),
                                                () -> driverController.rightBumper().getAsBoolean(),
() -> driverController.leftTrigger().getAsBoolean());

    HeadingCorTeleopDrive simDrvHeadingCorr = new HeadingCorTeleopDrive(drivebase, 
                                                () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                                                () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                                                () -> driverController.getRightX(), () -> driverController.getRightY(), 
                                                () -> driverController.rightBumper().getAsBoolean(),
() -> driverController.leftTrigger().getAsBoolean());






    
    // TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
    //                                                 () -> MathUtil.applyDeadband(driverController.getLeftY() ,
    //                                                                              OperatorConstants.LEFT_Y_DEADBAND),
    //                                                 () -> MathUtil.applyDeadband(driverController.getLeftX() ,
    //                                                                              OperatorConstants.LEFT_X_DEADBAND),
    //                                                 () -> MathUtil.applyDeadband(driverController.getRawAxis(4), 0.1), () -> true,
    //                                              () -> driverController.getHID().getRightBumper(),
    //                                               speedModify /* left trigger */);
    // TeleopDrive closedFieldRel = new TeleopDrive(
    //     drivebase,
    //     () -> MathUtil.applyDeadband(driverController.getLeftY() , OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverController.getLeftX() , OperatorConstants.LEFT_X_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverController.getRawAxis(4), 0.1), () -> true,
    //     () -> driverController.getHID().getRightBumper(),
    //     speedModify /* left trigger */); 



    TeleopDrive simClosedFieldRel = new TeleopDrive(
                drivebase,
                () -> Math.abs(driverController.getLeftY()) < OperatorConstants.LEFT_Y_DEADBAND ? 0.0 : driverController.getLeftY(),
                () -> Math.abs(driverController.getLeftX()) < OperatorConstants.LEFT_X_DEADBAND ? 0.0 : driverController.getLeftX(),
                () -> Math.abs(driverController.getRawAxis(4)) < OperatorConstants.RIGHT_X_DEADBAND ? 0.0 : driverController.getRawAxis(4), 
                true,
                () -> driverController.getHID().getRightBumper(),
                () -> driverController.getHID().getLeftBumper());


    TeleopDrive closedFieldRel = new TeleopDrive(
                drivebase,
                () -> Math.abs(driverController.getLeftY()) < OperatorConstants.LEFT_Y_DEADBAND ? 0.0 : driverController.getLeftY(),
                () -> Math.abs(driverController.getLeftX()) < OperatorConstants.LEFT_X_DEADBAND ? 0.0 : driverController.getLeftX(),
                () -> Math.abs(driverController.getRawAxis(4)) < OperatorConstants.RIGHT_X_DEADBAND ? 0.0 : driverController.getRawAxis(4), 
                true,
                () -> driverController.getHID().getRightBumper(),
                () -> driverController.getHID().getLeftBumper());

                // driverController.axisGreaterThan(2, 0.2).getAsBoolean()
/* 
    DoubleSupplier y = () -> driverController.getLeftY();
    DoubleSupplier x = () -> driverController.getLeftX();
    DoubleSupplier z = () -> driverController.getRawAxis(4);

    Logger.recordOutput("SwerveStates/ControllerInputLog/RobotContainer/x", y.getAsDouble());
    Logger.recordOutput("SwerveStates/ControllerInputLog/RobotContainer/y", x.getAsDouble());
    Logger.recordOutput("SwerveStates/ControllerInputLog/RobotContainer/z", z.getAsDouble());
*/
    


    drivebase.setDefaultCommand(RobotBase.isSimulation() ? simClosedFieldRel : closedFieldRel);
    
    
    
    // drivebase.setDefaultCommand(!RobotBase.isSimulation() ? simDrvHeadingCorr : drvHeadingCorr);
    
    // driverController.rightTrigger().whileFalse();
    // driverController.rightTrigger().whileTrue(closedFieldRelSlow);
    // driverController.start().onTrue(new SequentialCommandGroup(new InstantCommand( () -> drivebase.), new InstantCommand( () -> resetDrive(/*closedFieldRel, simClosedFieldRel*/)).andThen(new WaitCommand(.2)),
    
    /* reset drive?? */
    driverController.start().onTrue(new SequentialCommandGroup(
        new InstantCommand( () -> drivebase.removeDefaultCommand()), 
        new InstantCommand( () -> drivebase.drive(new Translation2d(), drivebase.getTargetSpeeds(0, 0, drivebase.getHeading().getSin(), drivebase.getHeading().getCos()).omegaRadiansPerSecond, true)), 
        new WaitCommand(0.2), 
        new InstantCommand( () -> drivebase.setDefaultCommand(!RobotBase.isSimulation() ? simClosedFieldRel : closedFieldRel))));
                         
    //                                 new InstantCommand( () -> drivebase.setDefaultCommand(!RobotBase.isSimulation() ? simClosedFieldRel : closedFieldRel))));

    // Configure the trigger bindings
    configureBindings();
  } // FR: 323.086, FL: 303.486
  // 

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings()
  {
    /* Reset Gyro is now built into drive commands!! (hit rightBumper) */
    /* [same as halfing the drive speed (hold rightTrigger)]


    //driverController.rightBumper().onTrue(new InstantCommand(drivebase::zeroGyro));
    

    //new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));



    /* amp */
    armController.leftTrigger().onTrue(new BasicToAmpCommand(armWristSub));
    // driverController.leftTrigger().onTrue(new InstantCommand(() -> intakeSub.intakeNote(5)));
    
    //driverController.leftBumper().onTrue( new TimedIntakeSetPowerCommand(intakeSub, 10, 1.5));
    
    // /* hold */
    armController.rightTrigger().onTrue(new SequentialCommandGroup(
      new TimedIntakeSetPowerCommand(intakeSub, 10, 0.75),
      new BasicToHoldCommand(armWristSub),
      new InstantCommand(() -> servo.setAxonAngle(130))
    ));

    //driverController.leftBumper().onFalse(new InstantCommand( () -> intakeSub.stopIntake() ));
    
    //driverController.rightBumper().onFalse(new InstantCommand( () -> wristIntake.stopIntake() ));

   
    /*
    // New Source

    armController.y().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> armSub.goToPosition(Constants.Height.SOURCE)),
      new InstantCommand(() -> extSub.goToPosition(Constants.Height.SOURCE)),
      new InstantCommand(() -> wristSub.goToPosition(Constants.Height.SOURCE)),
      new IntakingCommand(intakeSub, 5)
    ));

    */

    /* Source */
    
    // armController.y().onTrue(new SequentialCommandGroup(
    //   new InstantCommand(() -> armWristSub.goToPosition(Height.SOURCE)),
    //   new IntakingCommand(intakeSub, 5)
    // ));

    armController.y().onTrue(new SequentialCommandGroup(
      new BasicToSourceCommand(armWristSub),
      new IntakingCommand(intakeSub, 8),
      new InstantCommand(() -> armWristSub.goToPosition(Height.PRECLIMB)),
      new InstantCommand(() -> armWristSub.changeHeight(Height.PRECLIMB)),
      new InstantCommand(() -> armWristSub.runAndResetExtEncoder())
    ));
   
    //driverController.rightTrigger().onTrue(new InstantCommand(() -> armWristSub.goToPosition(Height.SOURCE)));

    /*
    // New Amp

    armController.b().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> armSub.goToPosition(Constants.Height.AMP)),
      new InstantCommand(() -> extSub.goToPosition(Constants.Height.AMP)),
      new InstantCommand(() -> wristSub.goToPosition(Constants.Height.AMP))
    ));
    */
    
    /* preclimb */
    //armController.b().onTrue(new InstantCommand(() -> armWristSub.goToPosition(Height.AMP)));
    armController.b().onTrue(new InstantCommand(() -> armWristSub.goToPosition(Height.PRECLIMB)));
   
   /*
    //New Ground
    armController.a().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> armSub.goToPosition(Constants.Height.GROUND)),
      new InstantCommand(() -> extSub.goToPosition(Constants.Height.GROUND)),
      new InstantCommand(() -> wristSub.goToPosition(Constants.Height.GROUND)),
      new IntakingCommand(intakeSub, 5)
    ));
    */

    // Ground
    // armController.a().onTrue(new SequentialCommandGroup(
    //   new InstantCommand(() -> armWristSub.goToPosition(Height.GROUND)),
    //   new IntakingCommand(intakeSub, 8)
    //   ));
    
    /* ground, intake, hold */
    armController.a().onTrue(new SequentialCommandGroup(
      new BasicToGroundCommand(armWristSub),
      new IntakingCommand(intakeSub, 12),
      new BasicToHoldCommand(armWristSub),
      new InstantCommand(() -> servo.setAxonAngle(180))
    ));
        
    
    /*
    // New Holding
    armController.x().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> wristSub.goToPosition(Constants.Height.HOLD)),
      new InstantCommand(() -> extSub.goToPosition(Constants.Height.HOLD)),
      new InstantCommand(() -> armSub.goToPosition(Constants.Height.HOLD))
    ));
    */

    // Holding Position
    //armController.x().onTrue(new InstantCommand(() -> armWristSub.goToPosition(Height.HOLD)));
    armController.x().onTrue(new BasicToHoldCommand(armWristSub));


    // Climber
    armController.pov(180).onTrue(new ClimbCommand(climbSub, 0));
    armController.pov(90).onTrue(new BasicToHoldCommand(armWristSub).andThen(new WaitCommand(0.5)).andThen(new ClimbCommand(climbSub, 90)));
    armController.pov(0).onTrue(new ClimbCommand(climbSub, 90));
    armController.pov(270).onTrue(new ClimbCommand(climbSub, 25));

    armController.leftBumper().onTrue(new ResetClimbCommand(climbSub));

    //armController.pov(0).onTrue(new ClimbCommand(climbSub, 10));
    //armController.pov(180).onTrue(new ClimbCommand(climbSub, 80));
    //armController.pov(90).onTrue(new ResetClimbCommand(climbSub));
    //armController.pov(270).onTrue(new ClimbCommand(climbSub, 25));
    armController.rightBumper().onTrue(new InstantCommand(() -> armWristSub.runAndResetExtEncoder()));

    driverController.a().onTrue(
      new InstantCommand(() -> armWristSub.goToPosition(Height.PRECLIMB))
      //.andThen(new ResetClimbCommand(climbSub)).andThen(new WaitCommand(1))
      //.andThen(new ClimbCommand(climbSub, 90))
      );

    driverController.b().onTrue(new InstantCommand(() -> armWristSub.goToPosition(Height.TRAP)));

    driverController.y().onTrue(new ParallelCommandGroup(new TimedIntakeSetPowerCommand(intakeSub, 12, 0.75), new WaitCommand(0.1).andThen(new InstantCommand(() -> armWristSub.goToPosition(Height.PRECLIMB)))));

    

    driverController.rightTrigger().onFalse(new InstantCommand(() -> armWristSub.setExtensionGoal(armWristSub.extensionTarget - 12 / 2.083)));
    driverController.pov(270).onFalse(new InstantCommand(() -> armWristSub.setExtensionGoal(armWristSub.extensionTarget + 12 / 2.083)));

    driverController.x().onTrue(new AutoClimbCommand(climbSub, armWristSub));

    driverController.leftTrigger().whileTrue(new AlignSource2(drivebase, driverXbox, servo, false));


  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand(autoChooser.getSelected(), true, true);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void resetDrive(/*TeleopDrive closedFieldRel, TeleopDrive simClosedFieldRel*/) {
    // drivebase.removeDefaultCommand();
    
    // drivebase.setDefaultCommand(!RobotBase.isSimulation() ? simClosedFieldRel : closedFieldRel);
    drivebase.resetAngleMotors();
  }

  public void setDriveBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void setArmWristExtBrake(boolean brake) {
    armWristSub.setAllBrake(brake);
  }

} 