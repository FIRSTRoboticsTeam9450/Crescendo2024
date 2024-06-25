package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.BitToStickyfaultString;
import frc.robot.Constants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private Field2d field;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
        this.field = field;
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
        this.field = field;
    }

    /**
     * Gets the sticky faults from the drive motors as an int (in bits), sending them to BitToStickyfaultString.java and clears the faults.
     */
    public void getStickyFaults() {
        

        int i = 0;
        for (SwerveModule swerveModule : Modules) {
            
            BitToStickyfaultString.getStickyFaultString(getStickyFaults(swerveModule.getDriveMotor()),
                                                        Constants.SwerveModuleMaps.driveMotorMap.get(i), 
                                                        Constants.stickyFaultNames.stickyFaultStringCTRE);
            BitToStickyfaultString.getStickyFaultString(getStickyFaults(swerveModule.getSteerMotor()),
                                                        Constants.SwerveModuleMaps.steerMotorMap.get(i), 
                                                        Constants.stickyFaultNames.stickyFaultStringCTRE);
            i++;
        }

        
        
    }

    /**for each individual motor, not meant to be called directly to get a sticky fault */
    public int getStickyFaults(TalonFX motor) {
        int stickyFaultBits = 0;
        int shiftInt = 1;
        
        stickyFaultBits = motor.getStickyFault_BootDuringEnable().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_BridgeBrownout().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_DeviceTemp().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_ForwardHardLimit().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_ForwardSoftLimit().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_FusedSensorOutOfSync().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_Hardware().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_MissingDifferentialFX().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;
        
        stickyFaultBits = motor.getStickyFault_OverSupplyV().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_ProcTemp().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_RemoteSensorDataInvalid().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_RemoteSensorPosOverflow().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_RemoteSensorReset().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_ReverseHardLimit().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_ReverseSoftLimit().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_StatorCurrLimit().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_SupplyCurrLimit().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_Undervoltage().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        stickyFaultBits = motor.getStickyFault_UnstableSupplyV().getValue() ? stickyFaultBits >> 1 : stickyFaultBits + shiftInt;
        shiftInt = shiftInt >> 1;

        motor.clearStickyFaults();

        return stickyFaultBits;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
        Logger.recordOutput("Robot Heading", getState().Pose.getRotation());
        if (field != null) {
            field.setRobotPose(getState().Pose);
        }
    }
}
