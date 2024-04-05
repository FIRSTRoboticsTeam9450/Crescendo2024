package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ExtensionCommand;

public class Extension extends SubsystemBase {
    /* Class Constants */
    private double target = Constants.MovementLimits.extHardLowerLimit + Constants.Extension.offsetToPreClimb;
    private boolean runAndReset;
    private boolean run = true;

    private CANSparkMax motor = new CANSparkMax(Constants.extensionId, MotorType.kBrushless);

    /* Ext Rel Encoder */
    private RelativeEncoder encoderRel;

    /* Limit Switch */
    private DigitalInput lowerHardLimSwitch;

    /* Enums */ 
    @SuppressWarnings("unused")
    private Constants.RobotState state = Constants.RobotState.DEFAULT;

    /* PIDConstants */
    private PIDConstants pidConstantsClimb = new PIDConstants(0, 0);
    private PIDConstants pidConstantsDefault = new PIDConstants(1, 12);
    private PIDConstants currentPIDConstants = pidConstantsDefault;


    public Extension() {
        motor.restoreFactoryDefaults();

        motor.setSmartCurrentLimit(40);

        /* Periodic frame rate */
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300); // For follower motors
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); // For Motor Position

        /* for extension 'reset' at hard lower limit */
        lowerHardLimSwitch = new DigitalInput(2);

        motor.setIdleMode(IdleMode.kBrake);

        /* encoder :) */
        encoderRel = motor.getEncoder();

        motor.burnFlash();

        runAndResetEncoder();

    }

    /**
     * Returns true if the magnet is in range of the lower hard limit switch
     * 
     * @return the state of the limit switch
     */
    public boolean getLowerLimSwitch() {
        try {
            return !lowerHardLimSwitch.get();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return true;

    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        Logger.recordOutput("Extension/ExtVoltage", voltage);

        SmartDashboard.putNumber("Ext Voltage", voltage);

    }

    /**
     * @return the relative position of the extension
     */
    public double getRelPos() {

        Logger.recordOutput("Extension/ExtPos", encoderRel.getPosition());
        return encoderRel.getPosition();

    }

    /**
     * Toggles a boolean so that
     * the periodic method runs the motor toward the lowerHardLimit at 10 volts,
     * then
     * will stop the motor and reset encoder after limit switch reached.
     * This method should be called in the init of the {@link ExtensionCommand}.
     */
    public void runAndResetEncoder() {
        runAndReset = true;
        setVoltage(3);
    }

    public void setState(Constants.RobotState state) {
        this.state = state;
        if (state.equals(Constants.RobotState.CLIMBING)) {
            currentPIDConstants = pidConstantsClimb;
        } else {
            currentPIDConstants = pidConstantsDefault;
        }
    }

    public void setTarget(double target) {

        this.target = convertToTics(target);
    }

    public double getTarget() {
        return convertToInches(target);
    }

    private double convertToTics(double inches) {
        return inches * (1/Constants.NewExtension.convertToInches);

    }

    private double convertToInches(double tics){
        return tics * Constants.NewExtension.convertToInches;
    }

    public void updatePID() {
        double error = target - getRelPos();
        double pidValue = (error) * currentPIDConstants.kP;
        double maxVoltage = currentPIDConstants.maxVoltage;
        Logger.recordOutput("Extension/maxVoltage", maxVoltage);

        if (Math.abs(pidValue) < maxVoltage) {
            setVoltage(pidValue);
        } else {
            setVoltage(maxVoltage * Math.signum(pidValue));
        }
    }

    @Override
    public void periodic() {
        if (!runAndReset) {
            if (run) {
                updatePID();
            } else {
                motor.stopMotor();
            }
            
        }
        
        
        /* Stops motor and resets encoder after limit switch reached */
        if (getLowerLimSwitch() && runAndReset) {
            motor.stopMotor();
            setVoltage(0);
            encoderRel.setPosition(0);
            runAndReset = false;

        }
    }

    /** changes the boolean for whether or not to run ext...if run == false, then stopMotor() is called */
    public void toggleExt(boolean run) {
        this.run = run;
    }

}