package frc.robot.commands.swervedrive.drivebase;

import java.math.MathContext;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignSource extends Command {
    SwerveSubsystem drive;
    PIDController yawController;
    PIDController xController;
    PIDController zController;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    public AlignSource(SwerveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xController = new PIDController(1.5, 0, 0);
        zController = new PIDController(1.5, 0, 0);
        xController.setSetpoint(0);
        zController.setSetpoint(-1.2);

        yawController = new PIDController(0.05, 0, 0);
        yawController.setSetpoint(0);
    }

    @Override
    public void execute() {
        double[] pose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        double yawPower = yawController.calculate(pose[4]);
        //power = MathUtil.clamp(power, -0.2, 0.2);
        yawPower = MathUtil.clamp(yawPower, -5, 5);

        double xPower = xController.calculate(pose[0]);
        xPower = MathUtil.clamp(xPower, -1, 1);
        SmartDashboard.putNumber("x power", xPower);

        double zPower = zController.calculate(pose[2]);
        zPower = MathUtil.clamp(zPower, -1, 1);

        drive.drive(new Translation2d(zPower, -xPower), -yawPower, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}