package frc.robot.commands.swervedrive.drivebase;

import java.math.MathContext;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignSource2 extends Command {
    SwerveSubsystem drive;
    PIDController yawController;
    PIDController xController;
    PIDController zController;
    PIDController servoController;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    double[] tagPose;
    double[] robotPose;
    Timer timer;

    boolean justLost = true;

    boolean noTagStart = false;

    XboxController controller;

    LimelightSubsystem lSubsystem;

    Rotation3d originalHeading;
    
    public AlignSource2(SwerveSubsystem drive, XboxController controller, LimelightSubsystem lSubsystem) {
        this.drive = drive;
        addRequirements(drive);
        timer = new Timer();
        this.controller = controller;
        this.lSubsystem = lSubsystem;
    }

    @Override
    public void initialize() {
        noTagStart = false;

        xController = new PIDController(3, 0, 0);
        zController = new PIDController(3, 0, 0);
        

        yawController = new PIDController(5, 0, 0);

        servoController = new PIDController(0.05, 0, 0);
        servoController.setSetpoint(0);

        timer.restart();

        tagPose = new double[6];
        robotPose = new double[6];
        justLost = true;

        if (table.getEntry("tid").getDouble(-1) == -1) {
            noTagStart = true;
            lSubsystem.setAxonAngle(107);
        }

    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("No tag", noTagStart);
        if (noTagStart) {
            drive.drive(new Translation2d(0, 0), 0, false);
            if (lSubsystem.isInAmpMode()) {
                return;
            }
        }

        double yawPower = 0;
        double xPower = 0;
        double zPower = 0;

        // Check if it can see any tags
        if (table.getEntry("tid").getDouble(-1) == -1) {
            drive.drive(new Translation2d(0, 0), 0, false);
            controller.setRumble(RumbleType.kBothRumble, 1);
            /*
            if (justLost) {
                if (amp) {
                    xController.setSetpoint(tagPose[2] - 0.7);
                    zController.setSetpoint(tagPose[0]);
                    yawController.setSetpoint(0);
                } else {
                    xController.setSetpoint(tagPose[2] - 0.7);
                    zController.setSetpoint(-tagPose[0] + 0.22);
                    yawController.setSetpoint(0);
                }
                double heading = tagPose[5] * Math.PI / 180;
                drive.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(-robotPose[4] * Math.PI / 180)));
                justLost = false;
            }
            Pose2d drivePose = drive.getPose();
            yawPower = yawController.calculate(drivePose.getRotation().getRadians());
            xPower = xController.calculate(drivePose.getX());
            zPower = zController.calculate(drivePose.getY());
            yawPower = MathUtil.clamp(yawPower, -5, 5);
            xPower = MathUtil.clamp(xPower, -1, 1);
            zPower = MathUtil.clamp(zPower, -1, 1);
            yawPower = 0;

            if (amp) {
                drive.drive(new Translation2d(xPower, zPower), yawPower, true);
            } else {
                drive.drive(new Translation2d(xPower, zPower), yawPower, true);
            }
            */
            
        } else {
            controller.setRumble(RumbleType.kBothRumble, 0);

            tagPose = table.getEntry("targetpose_robotspace").getDoubleArray(tagPose);
            robotPose = table.getEntry("botpose_targetspace").getDoubleArray(tagPose);

            double axonPower = servoController.calculate(table.getEntry("ty").getDouble(0));
            MathUtil.clamp(axonPower, -1, 1);
            lSubsystem.setAxonAngle(lSubsystem.getAxonAngle() - axonPower);

            yawController.setP(0.05);
            if (lSubsystem.isInAmpMode()) {
                yawController.setSetpoint(0);
                xController.setSetpoint(0.12);
                zController.setSetpoint(-0.7);
            } else {
                yawController.setSetpoint(0);
                xController.setSetpoint(-0.22);
                zController.setSetpoint(-0.39);
            }
            
            yawPower = yawController.calculate(robotPose[4]);
            xPower = xController.calculate(robotPose[0]);
            zPower = zController.calculate(robotPose[2]);
            yawPower = MathUtil.clamp(yawPower, -5, 5);
            xPower = MathUtil.clamp(xPower, -1, 1);
            zPower = MathUtil.clamp(zPower, -1, 1);

            if (lSubsystem.isInAmpMode()) {
                drive.drive(new Translation2d(-zPower, xPower), -yawPower, false);
            } else {
                drive.drive(new Translation2d(zPower, -xPower), -yawPower, false);
            }

            justLost = true;
        }


    }

    @Override
    public boolean isFinished() {
        if (yawController.getPositionError() < 1 && xController.getPositionError() < 0.05 && zController.getPositionError() < 0.05 && timer.get() > 1) {
            //drive.drive(new Translation2d(0, 0), 0, false);
            return false;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0, 0), 0, true);
        controller.setRumble(RumbleType.kBothRumble, 0);
        lSubsystem.resetAngle();
    }

}