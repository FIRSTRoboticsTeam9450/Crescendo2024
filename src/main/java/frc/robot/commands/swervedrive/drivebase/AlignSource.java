package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignSource extends Command {
    SwerveSubsystem drive;
    PIDController xController;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    public AlignSource(SwerveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xController = new PIDController(0.01, 0, 0);
        xController.setSetpoint(0);
    }

    @Override
    public void execute() {
        double tx = table.getEntry("tx").getDouble(0.0);
        double power = xController.calculate(tx);
        //power = MathUtil.clamp(power, -0.2, 0.2);
        SmartDashboard.putNumber("x power", power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
