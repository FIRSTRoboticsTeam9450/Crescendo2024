package frc.robot.commands.swervedrive.drivebase;

import java.math.MathContext;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignSource2 extends Command {
    SwerveSubsystem drive;
    PIDController yawController;
    PIDController xController;
    PIDController zController;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    double[] pose;
    Timer timer;
    
    public AlignSource2(SwerveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        xController = new PIDController(3, 0, 0);
        zController = new PIDController(3, 0, 0);
        xController.setSetpoint(-0.22);
        zController.setSetpoint(-0.39);

        yawController = new PIDController(0.05, 0, 0);
        yawController.setSetpoint(0);
        timer.restart();
        pose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        drive.resetOdometry(new Pose2d(new Translation2d(pose[0], pose[2]), new Rotation2d(pose[5])));

    }

    @Override
    public void execute() {
        Pose2d drivePose = drive.getPose();
        double yawPower = yawController.calculate(drivePose.getRotation().getRadians());
        //power = MathUtil.clamp(power, -0.2, 0.2);
        yawPower = MathUtil.clamp(yawPower, -5, 5);

        double xPower = xController.calculate(drivePose.getX());
        xPower = MathUtil.clamp(xPower, -1, 1);
        SmartDashboard.putNumber("x power", xPower);

        double zPower = zController.calculate(drivePose.getY());
        zPower = MathUtil.clamp(zPower, -1, 1);

        drive.drive(new Translation2d(zPower, -xPower), -yawPower, false);
    }

    @Override
    public boolean isFinished() {
        if (yawController.getPositionError() < 1 && xController.getPositionError() < 0.05 && zController.getPositionError() < 0.05 && timer.get() > 1) {
            drive.drive(new Translation2d(0, 0), 0, false);
            return true;
        }
        return false;
    }

}