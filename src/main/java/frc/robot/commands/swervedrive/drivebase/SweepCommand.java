package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SweepCommand extends Command{

    Timer timer;
    SwerveSubsystem drive;

    public SweepCommand(SwerveSubsystem drive) {
        timer = new Timer();
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        double power = 0.1;
        if (timer.get() > 1) {
            power *= -1;
        }
        drive.drive(new Translation2d(), power, false);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 3;
    }
    
}
