package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Examples.ArmWristSubsystem;

public class AutoClimbCommand extends Command{

    private enum ClimbState {
        FIRSTCLIMB,
        ARMDOWN,
        CLIMBERSUP,
        SECONDCLIMB
    }

    Climb climb;
    ArmWristSubsystem arm;
    ClimbState state;

    Timer timer;

    boolean firstTime;

    boolean timerThing;

    public AutoClimbCommand(Climb climb, ArmWristSubsystem arm) {
        this.climb = climb;
        this.arm = arm;
        addRequirements(climb);
        addRequirements(arm);
        timer = new Timer();
        
    }

    @Override
    public void initialize() {
        timerThing = true;
        firstTime = true;
        state = ClimbState.FIRSTCLIMB;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Climbers down first
        if (state == ClimbState.FIRSTCLIMB && firstTime) {
            climb.setTargetPosition(0);
            firstTime = false;
        }

        // Arm down when they reach position
        if (climb.getLeftPosition() < 2 && climb.getRightPosition() < 2 && state == ClimbState.FIRSTCLIMB) {
            if (timerThing) {
                timer.restart();
                timerThing = false;
            } else if (timer.get() > 2) { // wait 2 seconds after finishing for next thing
                state = ClimbState.ARMDOWN;
                arm.isClimbing = true;
                arm.goToPosition(ArmWristSubsystem.Height.HOLD);
                timerThing = true;
            }
            
        }

        // Climbers up next
        if (state == ClimbState.ARMDOWN && Math.abs(arm.getAbsArmPos() - arm.getGoal()) < 0.1) {
            arm.isClimbing = false;
            if (timerThing) {
                timer.restart();
                timerThing = false;
            } else if (timer.get() > 2) { // wait 2 seconds
                climb.setTargetPosition(90);
                state = ClimbState.CLIMBERSUP;
                timerThing = true;
            }
            
        }

        // Climbers down one more time
        if (state == ClimbState.CLIMBERSUP && climb.getLeftPosition() > 88 && climb.getRightPosition() > 88) {
            if (timerThing) {
                timer.restart();
                timerThing = false;
            } else if (timer.get() > 2) { // wait 2 seconds
                climb.setTargetPosition(25);
                state = ClimbState.SECONDCLIMB;
                timerThing = true;
            }
        }


    }

    @Override
    public boolean isFinished() {
        return state == ClimbState.SECONDCLIMB && climb.isFinishedMoving();
    }
    
}
