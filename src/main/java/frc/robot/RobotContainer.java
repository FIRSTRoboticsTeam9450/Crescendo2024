// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.ExtensionCommand;
import frc.robot.commands.IntakingCommand;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem intakeSub = new IntakeSubsystem();
  private final ArmWristSubsystem armWristSub = new ArmWristSubsystem();
 // private final ExtensionSubsystem extSub = new ExtensionSubsystem();
  // private final DrivebaseSubsystem driveTest = new DrivebaseSubsystem(0);
  // private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);

// can't terminate on vortex, can't have more than 10 vortexs, and a sparkflex motorcontroller randomly stopped working
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller =
      new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    //extSub.setDefaultCommand(new ExtensionCommand(extSub, armWristSub, () -> controller.rightBumper().getAsBoolean(), 0));
    
    // controller.leftBumper().onTrue(new InstantCommand( () -> extSub.setExtensionVoltage(1)));
    // controller.leftBumper().onFalse(new InstantCommand( () -> extSub.setExtensionVoltage(0)));

    /* intake */
    //controller.leftTrigger().onTrue(new IntakingCommand(intakeSub, 5));
    controller.leftTrigger().onTrue(new InstantCommand(() -> intakeSub.intakeNote(5)));
    controller.leftBumper().onTrue(new InstantCommand(() -> intakeSub.setIntakeVoltage(-3)));
    //controller.rightBumper().onFalse(new InstantCommand( () -> wristIntake.stopIntake() ));

    /* toggle wrist idlemode */
    //controller.a().onTrue(new InstantCommand( () -> armWristSub.toggleWristBrake() ));

    
    // /* outtake */
     controller.leftBumper().onTrue(new InstantCommand( () -> intakeSub.setIntakePower(-0.5) ));
     controller.leftBumper().onFalse(new InstantCommand( () -> intakeSub.stopIntake() ));
    
 
    // /* arm *//* right trigger run wrist pid */
    controller.x().onTrue(new InstantCommand( () -> armWristSub.toggleWrist()));
    controller.a().onTrue(new InstantCommand(() -> armWristSub.toggleArm()));
    // Source
    controller.rightTrigger().onTrue(new InstantCommand(() -> armWristSub.setArmWristExtGoal(0.37, 0.387, 0.5346)));
    // Amp
    controller.rightBumper().onTrue(new InstantCommand(() -> armWristSub.setArmWristExtGoal(0.511, 0.0487, 0.387)));
    
    // Ground
    controller.y().onTrue(new InstantCommand(() -> armWristSub.setArmWristExtGoal(0.1716, 0.5, 0.387)));
        
    // Holding Position
    controller.b().onTrue(new InstantCommand(() -> armWristSub.setArmWristExtGoal(0.108, 0.04, 0.7)));

    // /* extension */


    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
  }

  
 
}
