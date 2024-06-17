package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BitToStickyfaultString;
import frc.robot.Constants;

public class Extension extends SubsystemBase {
    // Class Constants
    // target is stored in inches
    private double targetInches = 0;

    // this only needs to run once after powerup
    public boolean runAndReset = true;

    // these are in ticks or rotations
    private double currentPosInches;

    // holds the previous output voltage 
    // initialize to an illegal voltage so
    // voltage is set on start
    private double prevVoltage = -100;
    
    // it is really the controller but we like to call it motor
    private CANSparkFlex motor;
    
    // Relative Encoder
    private RelativeEncoder encoderRel;

    // Limit Switch
    private SparkLimitSwitch lowerHardLimSwitch; 

    // PIDConstants for our simple PID
    private PIDConstants currentPIDConstants = new PIDConstants(1, 6);


    // constructor
    public Extension() {
        motor = new CANSparkFlex(Constants.extensionId, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(40);

        // Periodic frame rate
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300); // For follower motors
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); // For Motor Position

        // to protect extension mechanism if the code drives it too low
        lowerHardLimSwitch = motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        // the pid will hold it
        motor.setIdleMode(IdleMode.kCoast);

        motor.setInverted(true);

        // get initial value
        encoderRel = motor.getEncoder();

        // this is probably not needed but it only runs once per powerup
        motor.burnFlash();

        runAndReset = true;
        
        Logger.recordOutput("Extension/maxVoltage", currentPIDConstants.maxVoltage);
    }

    public void logMotorStickyFaults() {
        BitToStickyfaultString.getStickyFaultStringRevMotor(motor.getStickyFaults(), "Extension");
        motor.clearFaults();
    }

    int c=0;
    // this is the only place the voltage is set on the motor
    private void setVoltage(double newVoltage) {

        if (newVoltage != prevVoltage){
            prevVoltage = newVoltage;

            if (Math.abs(newVoltage) > currentPIDConstants.maxVoltage){
                newVoltage = currentPIDConstants.maxVoltage*Math.signum(newVoltage);
            }
            // this only called here
            motor.setVoltage(newVoltage);
            
            Logger.recordOutput("Extension/ExtVoltage", newVoltage);
            SmartDashboard.putNumber("Ext Voltage", newVoltage);
        }
    }

    /**
     * Only called once at the start of update
     * @return the positive relative position of the extension (normally negative)
     */
    private void updateCurrentPosInches() {
        double currentPos = encoderRel.getPosition();
        currentPosInches = convertToInches(currentPos);

        Logger.recordOutput("Extension/ExtPos", currentPos);
        Logger.recordOutput("Extension/ExtInches", currentPosInches);
    }

    public double getRelPos(){
        return currentPosInches;
    }

    // is the arm within .5 inches of target
    public boolean finishedPID() {
        return Math.abs(targetInches - currentPosInches) < .7 ? true : false;
    }

    // this is how other classes tell us what to do
    public void setTargetInches(double newTarget) {
        double target = Math.min(newTarget, Constants.Extension.extHardwareMax);
        targetInches = Math.max(target, Constants.Extension.extHardwareMin);
    }

    public double getTargetInches() {
        return targetInches;
    }

    // private double convertToTics(double inches) {
    //     return inches / Constants.Extension.convertToInches;
    // }

    private double convertToInches(double tics){
        return tics * Constants.Extension.convertToInches; // this needs an adjustment of about .95
    }

    // targetInches and currentPosInches should already be set
    // the pid works with tics
    public void updatePID() {
        setVoltage((targetInches - currentPosInches) * currentPIDConstants.kP);
    }

    @Override
    public void periodic() {
        
        updateCurrentPosInches(); // must be called at start of update
        if (!runAndReset) {
            updatePID();
        }
        /* Stops motor and resets encoder after limit switch reached */
        else {
            if (lowerHardLimSwitch.isPressed()) {
                setVoltage(0);
                encoderRel.setPosition(0);
                runAndReset = false;
            }
            else {
                setVoltage(Constants.Extension.resetExtVoltage);
            }
        }
        Logger.recordOutput("Extension/target(tics)", this.targetInches);
    }

    // no longer used
    /** changes the boolean for whether or not to run ext...if run == false, then stopMotor() is called */
    public void toggleExt(boolean run) {}

    public void runAndResetEncoder(){
        runAndReset = true;
    }

}