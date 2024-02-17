// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakingCommand;
import frc.robot.commands.ResetClimbCommand;
import frc.robot.commands.TimedIntakeSetPowerCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.HeadingCorTeleopDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimitSwitchSubsystem;
import frc.robot.subsystems.ArmWristSubsystem.Height;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;

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
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/vortex"));
                                                                         
  private final IntakeSubsystem intakeSub = new IntakeSubsystem();
  private final ArmWristSubsystem armWristSub = new ArmWristSubsystem();
  private final ClimbSubsystem climbSub = new ClimbSubsystem();
  // private final LimitSwitchSubsystem extLimitSub = new LimitSwitchSubsystem();
 // private final ExtensionSubsystem extSub = new ExtensionSubsystem();


  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController armController = new CommandXboxController(1);
  //CommandXboxController driverController = new CommandXboxController(0);
  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  //XboxController driverXbox = new XboxController(0);

  private double speedModifier = 0.5;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    Command armStore = new InstantCommand(() -> armWristSub.goToPosition(Height.HOLD));
    NamedCommands.registerCommand("ArmStore", armStore);
    

    HeadingCorTeleopDrive drvHeadingCorr = new HeadingCorTeleopDrive(drivebase, 
                                                () -> MathUtil.applyDeadband(driverController.getLeftY() * 0.5, OperatorConstants.LEFT_Y_DEADBAND),
                                                () -> MathUtil.applyDeadband(driverController.getLeftX() * 0.7, OperatorConstants.LEFT_X_DEADBAND),
                                                () -> driverController.getRightY() * 0.5, () -> driverController.getRightX() * 0.5);

    HeadingCorTeleopDrive simDrvHeadingCorr = new HeadingCorTeleopDrive(drivebase, 
                                                () -> MathUtil.applyDeadband(driverController.getLeftY() * 0.5, OperatorConstants.LEFT_Y_DEADBAND),
                                                () -> MathUtil.applyDeadband(driverController.getLeftX() * 0.5, OperatorConstants.LEFT_X_DEADBAND),
                                                () -> driverController.getRightY() * 0.5, () -> driverController.getRightX() * 0.5);







    
    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                                                    () -> MathUtil.applyDeadband(driverController.getLeftY() ,
                                                                                 OperatorConstants.LEFT_Y_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverController.getLeftX() ,
                                                                                 OperatorConstants.LEFT_X_DEADBAND),
                                                    () -> driverController.getRawAxis(4), () -> true);
    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverController.getLeftX() , OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftY() , OperatorConstants.LEFT_Y_DEADBAND),
        () -> driverController.getRawAxis(4), () -> true); // change the int in the parameter to the appropriate axis

    TeleopDrive simClosedFieldRelSlow = new TeleopDrive(drivebase,
                                                () -> MathUtil.applyDeadband(driverController.getLeftY() * speedModifier,
                                                                              OperatorConstants.LEFT_Y_DEADBAND),
                                                () -> MathUtil.applyDeadband(driverController.getLeftX() * speedModifier,
                                                                              OperatorConstants.LEFT_X_DEADBAND),
                                                () -> driverController.getRawAxis(4) * speedModifier, () -> true);
    TeleopDrive closedFieldRelSlow = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverController.getLeftX() * speedModifier, OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftY() * speedModifier, OperatorConstants.LEFT_Y_DEADBAND),
        () -> driverController.getRawAxis(4) * speedModifier, () -> true);

    
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? simClosedFieldRel : closedFieldRel);
    // drivebase.setDefaultCommand(!RobotBase.isSimulation() ? simDrvHeadingCorr : drvHeadingCorr);
    
    
    // driverController.rightTrigger().whileFalse();
    // driverController.rightTrigger().whileTrue(closedFieldRelSlow);
    

    // Configure the trigger bindings
    configureBindings(simClosedFieldRel, closedFieldRel, simClosedFieldRelSlow, closedFieldRelSlow);
  } // FR: 323.086, FL: 303.486
  // 

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings(TeleopDrive simDrv, TeleopDrive drv, TeleopDrive simDrvSlow, TeleopDrive drvSlow)
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    final RepeatCommand slowDrvCmd = new RepeatCommand(!RobotBase.isSimulation() ? simDrvSlow : drvSlow);
    
    driverController.rightBumper().onTrue(new InstantCommand(drivebase::zeroGyro));
    // speed modifier by half
    
    // driverController.rightTrigger().onTrue(new SequentialCommandGroup( new InstantCommand( () -> drivebase.removeDefaultCommand()),
    //                                        new InstantCommand( () -> drivebase.setDefaultCommand(!RobotBase.isSimulation() ? simDrv : drv))));
    // driverController.rightTrigger().onFalse(new SequentialCommandGroup( new InstantCommand( () -> drivebase.removeDefaultCommand()),
    //                                        new InstantCommand( () -> drivebase.setDefaultCommand(!RobotBase.isSimulation() ? simDrvSlow : drvSlow))));
    driverController.rightTrigger().onTrue(slowDrvCmd);
    driverController.rightTrigger().onFalse(new InstantCommand( () -> CommandScheduler.getInstance().cancel(slowDrvCmd)));
    

    //new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));



    /* intake */
    armController.leftTrigger().onTrue(new IntakingCommand(intakeSub, 5));
    // driverController.leftTrigger().onTrue(new InstantCommand(() -> intakeSub.intakeNote(5)));
    
    //driverController.leftBumper().onTrue( new TimedIntakeSetPowerCommand(intakeSub, 10, 1.5));
    
    // /* outtake */
    armController.rightTrigger().onTrue(new TimedIntakeSetPowerCommand(intakeSub, 10, 0.75));

    //driverController.leftBumper().onFalse(new InstantCommand( () -> intakeSub.stopIntake() ));
    
    //driverController.rightBumper().onFalse(new InstantCommand( () -> wristIntake.stopIntake() ));

    /* toggle wrist idlemode */
    //driverController.a().onTrue(new InstantCommand( () -> armWristSub.toggleWristBrake() ));
 
    // /* arm *//* right trigger run wrist pid */
    //driverController.x().onTrue(new InstantCommand( () -> armWristSub.toggleWrist()));
    //driverController.a().onTrue(new InstantCommand(() -> armWristSub.toggleArm()));
    
    // Source
    
       armController.y().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> armWristSub.goToPosition(Height.SOURCE)),
      new IntakingCommand(intakeSub, 5)
      ));
   
    //driverController.rightTrigger().onTrue(new InstantCommand(() -> armWristSub.goToPosition(Height.SOURCE)));
    
    // Amp
    armController.b().onTrue(new InstantCommand(() -> armWristSub.goToPosition(Height.AMP)));
    //driverController.rightBumper().onTrue(new InstantCommand(() -> armWristSub.goToPosition(Height.AMP)));
    
    // Ground
    armController.a().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> armWristSub.goToPosition(Height.GROUND)),
      new IntakingCommand(intakeSub, 5)
      ));
        
    // Holding Position
    armController.x().onTrue(new InstantCommand(() -> armWristSub.goToPosition(Height.HOLD)));


    // Climber
    armController.pov(180).onTrue(new ClimbCommand(climbSub, 3));
    armController.pov(90).onTrue(new InstantCommand(() -> armWristSub.goToPosition(Height.HOLD)));
    armController.pov(0).onTrue(new ClimbCommand(climbSub, 90));
    armController.pov(270).onTrue(new ClimbCommand(climbSub, 25));

    armController.leftBumper().onTrue(new ResetClimbCommand(climbSub));

    //armController.pov(0).onTrue(new ClimbCommand(climbSub, 10));
    //armController.pov(180).onTrue(new ClimbCommand(climbSub, 80));
    //armController.pov(90).onTrue(new ResetClimbCommand(climbSub));
    //armController.pov(270).onTrue(new ClimbCommand(climbSub, 25));
    armController.rightBumper().onTrue(new InstantCommand(() -> armWristSub.runAndResetExtEncoder()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("BlueLeftOneNote", true, true);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
} 