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

public class Extension extends SubsystemBase {
    /* Class Constants */
    private double target = convertToTics(Constants.Extension.climbExtPosition);
    public boolean runAndReset = true;
    private boolean run = true;

    private CANSparkMax motor = new CANSparkMax(Constants.extensionId, MotorType.kBrushless);

    /* Ext Rel Encoder */
    private RelativeEncoder encoderRel;

    /* Limit Switch */
    private DigitalInput lowerHardLimSwitch;

    // removed robot state thing

    /* PIDConstants */
    private PIDConstants pidConstantsDefault = new PIDConstants(1, 3);
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

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        Logger.recordOutput("Extension/ExtVoltage", voltage);

        SmartDashboard.putNumber("Ext Voltage", voltage);

    }

    /**
     * @return the positive relative position of the extension (normally negative)
     */
    public double getRelPos() {

        Logger.recordOutput("Extension/ExtPos", -encoderRel.getPosition());
        double inches = convertToInches(-encoderRel.getPosition());
        Logger.recordOutput("Extension/ExtInches", inches);
        Logger.recordOutput("Extension/ExtPos(converted)", convertToTics(inches));

        return inches;

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
        setVoltage(Constants.Extension.resetExtVoltage);
    }

    public boolean finishedPID() {
        return Math.abs(target - (-encoderRel.getPosition())) < 1 ? true : false;
    }

    public void setTarget(double targetInches) {
        double newTarget = Math.min(targetInches, Constants.Extension.extHardwareMax);
        newTarget = Math.max(newTarget, Constants.Extension.extHardwareMin);
        target = convertToTics(newTarget);
    }

    public double getTarget() {
        return convertToInches(target);
    }

    private double convertToTics(double inches) {
        return inches * (1/Constants.Extension.convertToInches);

    }

    private double convertToInches(double tics){
        return tics * Constants.Extension.convertToInches;
    }

    public void updatePID() {
        double error = target - convertToTics(getRelPos());
        double pidValue = (error) * currentPIDConstants.kP;
        double maxVoltage = currentPIDConstants.maxVoltage;
        Logger.recordOutput("Extension/maxVoltage", maxVoltage);

        if (Math.abs(pidValue) < maxVoltage) {
            setVoltage(-pidValue);
        } else {
            setVoltage(-maxVoltage * Math.signum(pidValue));
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
        Logger.recordOutput("Extension/target", this.target);
        
        /* Stops motor and resets encoder after limit switch reached */
        if (!lowerHardLimSwitch.get() && runAndReset) {
            motor.stopMotor();
            setVoltage(0);
            encoderRel.setPosition(1.55);
            runAndReset = false;

        }
    }

    /** changes the boolean for whether or not to run ext...if run == false, then stopMotor() is called */
    public void toggleExt(boolean run) {
        this.run = run;
    }

}