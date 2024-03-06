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
        xController.setSetpoint(0.5);
        zController.setSetpoint(-0.39);

        yawController = new PIDController(0.05, 0, 0);
        yawController.setSetpoint(0);
        timer.restart();
        //pose = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        drive.resetOdometry(new Pose2d());

    }

    @Override
    public void execute() {
        Pose2d drivePose = drive.getPose();
        double yawPower = yawController.calculate(drivePose.getRotation().getRadians());
        //power = MathUtil.clamp(power, -0.2, 0.2);
        yawPower = MathUtil.clamp(yawPower, -5, 5);

        double xPower = xController.calculate(drivePose.getX());
        xPower = MathUtil.clamp(xPower, -0.25, 0.25);
        SmartDashboard.putNumber("x power", xPower);

        double zPower = zController.calculate(drivePose.getY());
        zPower = MathUtil.clamp(zPower, -1, 1);

        // positive x = forward, positive y = left, positive rotation = counterclockwise
        drive.drive(new Translation2d(xPower, 0), 0, true);
    }

    @Override
    public boolean isFinished() {
        if (yawController.getPositionError() < 1 && xController.getPositionError() < 0.05 && zController.getPositionError() < 0.05 && timer.get() > 1) {
            //drive.drive(new Translation2d(0, 0), 0, false);
            return false;
        }
        return false;
    }

}