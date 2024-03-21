package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ExtensionCommand;


public class ArmWristSubsystem extends SubsystemBase{
    public enum Height{
        GROUND,
        HOLD,
        SOURCE,
        AMP,
        CLIMB,
        PRECLIMB,
        TRAP
    }
    private Height lastHeight = Height.PRECLIMB;
    private boolean wasSourceIntake = true;


    public double armHardLowerLimit = 0.12455;//0.105
    public double wristHardLowerLimit = 0.234; //      0.141
    public double extHardLowerLimit = 0; // 0.749

    private double armAbsTarget = armHardLowerLimit + Constants.Arm.offsetToClimb;//0.485;  //.453
    private double wristTarget = 0.387;
    public double extensionTarget = 0;

    private boolean ampToGround = false;
    private boolean groundToAmp = false;
    private boolean holdToSource = false;
    private boolean holdToAmp = false;

    private double maxArmVoltage = 6; //9

    
    boolean wristBrakeToggle;
    boolean wristPIDRun;
    boolean armPIDRun;
    boolean pause = false;
    boolean reachPos;
    public boolean isClimbing;
    boolean runAndResetExt;
    boolean firstStartingBot;

    Timer delayExt;

    /* Motors */
    private CANSparkMax armFrontMotor = new CANSparkMax(Constants.armFrontId, MotorType.kBrushless);
    private CANSparkMax armBackMotor = new CANSparkMax(Constants.armBackId, MotorType.kBrushless);
    private CANSparkFlex wrist = new CANSparkFlex(Constants.wristId, MotorType.kBrushless);
    private CANSparkMax extensionMotor = new CANSparkMax(Constants.extensionId, MotorType.kBrushless);

     /* Absolute Encoder */
    SparkAbsoluteEncoder wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
    SparkAbsoluteEncoder armEncoder = armFrontMotor.getAbsoluteEncoder(Type.kDutyCycle);
    RelativeEncoder extRelEncoder;
    RelativeEncoder armRelEncoder;
    
    /* Limit Switch */
    DigitalInput lowerHardLimSwitch;
    
    

    private boolean runStuff = true;

     // extension all the way in kg 0.12(0.89 for ext) 0.19 middle(0.47 for ext pos) and 0.24(0.145 for ext) full extended
    // Equation for this is ffVal = -0.16134 * ExtPos + 0.26427
    DoubleSupplier armFFkg = () -> 0.065;
    public SimpleMotorFeedforward wristFeedForward = new SimpleMotorFeedforward(0.00001, 0.00003, 0.00001); 

    

    // private final ProfiledPIDController armProfiledPid = new ProfiledPIDController(35, 0, 0, new TrapezoidProfile.Constraints(1, 8.2));//maxVel = 3.5 and maxAccel = 2.5 9, 8
    // private final PIDController armPid = new PIDController(30, 0, 0, 0.02);
    private final PIDController armClimbPid = new PIDController(30, 0, 0);
    
    private final PIDController wristPIDController = new PIDController(40, 0, 0); 
    private final PIDController extensionPid = new PIDController(1, 0,0);


    public ArmWristSubsystem(){
        delayExt = new Timer();
        
        armFrontMotor.restoreFactoryDefaults();
        armBackMotor.restoreFactoryDefaults();

        // extensionMotor.restoreFactoryDefaults();
        wrist.restoreFactoryDefaults();


        armFrontMotor.setSmartCurrentLimit(40);
        armBackMotor.setSmartCurrentLimit(40);

        extensionMotor.setSmartCurrentLimit(40);

            
    
        armFrontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
        armBackMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);
        // armFrontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
        // armFrontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
        // armFrontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency

        /* Status frames 3-6 set to 65535 if not using data port in spark max otherwise can prob leave them all alone*/
        wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
        //wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
        //wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
        //wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50); //Duty Cycle Absolute Encoder Position and Abs angle
        //wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency
    

        
        //If we use data port on extesnion, make sure to comment  the lines kstauts3-6
        extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
        extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); // For Motor Position
        // extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
        // extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
        // extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); //Duty Cycle Absolute Encoder Position and Abs angle
        // extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency


        lowerHardLimSwitch = new DigitalInput(2);
        

        wrist.setIdleMode(IdleMode.kBrake);
        armFrontMotor.setIdleMode(IdleMode.kBrake);
        armBackMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setIdleMode(IdleMode.kBrake);
        
        extRelEncoder = extensionMotor.getEncoder();
        armRelEncoder = armBackMotor.getEncoder();
        armRelEncoder.setPositionConversionFactor(1.0 / 103.85652383245);


        wristPIDRun = true;
        armPIDRun = true;
        wristBrakeToggle = false;
        isClimbing = false;
        runAndResetExt = false;
        //firstStartingBot = true;
        
        // armPid.setTolerance(0.021);


        armFrontMotor.setInverted(true); // b/c we flipped direction of motor
        armBackMotor.setInverted(true);
        
        
        armFrontMotor.burnFlash();
        armBackMotor.burnFlash();


        wrist.burnFlash();
        extensionMotor.burnFlash();

        //goToPosition(Height.SOURCE);
        runAndResetExtEncoder();
        setRelArmPos(armEncoder.getPosition());

        SmartDashboard.putNumber("Change Arm Target", armAbsTarget);
        SmartDashboard.putNumber("Change Wrist Target", Constants.midWristPos);
        SmartDashboard.putNumber("Change ArmFF", 0.065);
        SmartDashboard.putNumber("Change Arm Is Brake", 1);
        SmartDashboard.putNumber("Change Wrist Is Brake", 1);
        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Change Extension Target", 0.47);
        SmartDashboard.putNumber("Change Extension Is Brake", 1);
        SmartDashboard.putNumber("Max Arm Voltage", maxArmVoltage);
    }



    



    /*
     * 
     * Extension Methods
     * 
     */

     
    /**
   * Returns true if the magnet is in range of the lower hard limit switch
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

  public void setExtVoltage(double voltage) {
    extensionMotor.setVoltage(voltage);
    Logger.recordOutput("Extension/ExtVoltage", voltage);

    SmartDashboard.putNumber("Ext Voltage", voltage);

  }

  /**
   * @return the relative position of the extension
   */
  public double getExtRelPos() {
    try {
        
        Logger.recordOutput("Extension/ExtPos", extRelEncoder.getPosition());
        return extRelEncoder.getPosition();
    } catch (NullPointerException e) {
        e.printStackTrace();
    }
    return 20;
  }

  /**
   * Toggles a boolean so that 
   * the periodic method runs the motor toward the lowerHardLimit at 10 volts, then 
   * will stop the motor and reset encoder after limit switch reached.
   * This method should be called in the init of the {@link ExtensionCommand}.
   */
    public void runAndResetExtEncoder() {
        runAndResetExt = true;
        setExtVoltage(3);
    }

    
    public void setExtensionGoal(double target){

        extensionTarget = target;
    }

    public double getExtensionGoal(){

        return extensionTarget;
    }

    public double calculateExtensionPID(){
        return extensionPid.calculate(getExtRelPos(), extensionTarget);
    }
    public void updateExtensionOutput(){
        
        double voltage = MathUtil.clamp(calculateExtensionPID(), -12.0, 12.0);
        double maxExtensionVoltage = 12.0;

        Logger.recordOutput("Extension/maxVoltage", maxExtensionVoltage);

        
        // SmartDashboard.putNumber("Extension Percent", percentOutput);

        // double voltage = 12 * percentOutput;

        // voltage = MathUtil.clamp(voltage /*+ ffValue*/, -6, 6);

         if (Math.abs(voltage) < maxExtensionVoltage) {
            setExtVoltage(voltage);
        } else {
            setExtVoltage(maxExtensionVoltage * Math.signum(voltage));
        }
    }
    








    /*   
     * 
     * Arm Methods
     * 
     */
 






        

    public double getAbsArmPos(){
        // Logger.recordOutput("Arm/AbsCurrentPos", armEncoder.getPosition());
        // Logger.recordOutput("Arm/RelCurrentPos", armRelEncoder.getPosition());
        SmartDashboard.putNumber("Arm Abs Pos", armEncoder.getPosition());
        try {
            return armEncoder.getPosition();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return 0.3;
    }

    public void updateRelArmPos() {
        setRelArmPos(getAbsArmPos());
    }

    public void setRelArmPos(double position) {
        armRelEncoder.setPosition(position);
    }
    /**
    * @return the relative position of the arm
    */
    public double getArmRelPos() {
        
        try {
            Logger.recordOutput("Arm/RelCurrentPos", armRelEncoder.getPosition());
            return armRelEncoder.getPosition();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return getAbsArmPos();
    }

    public double getAbsWristPos(){

        try { 
            Logger.recordOutput("Wrist/CurrentPos", wristEncoder.getPosition());

            return wristEncoder.getPosition();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return 0.5;
    }
 

    
    public void setArmVoltage(double voltage){
        // leftMotor.setVoltage(-voltage);
        // SmartDashboard.putNumber("Rotation Voltage", voltage);

        Logger.recordOutput("Arm/ArmVoltage", voltage);
        
       armFrontMotor.setVoltage(voltage);
       armBackMotor.setVoltage(voltage);
    }

    public void setMaxArmVoltage(double voltage) {
        maxArmVoltage = voltage;
    }

    public void setArmGoal(double target) {
        //rotation.setGoal(target);
        armAbsTarget = target;
    }

    public void setArmWristExtGoal(double armAbsTarget, double wristTarget, double extTarget){
        
            setArmGoal(armAbsTarget);
            setWristSetpoint(wristTarget);
            setExtensionGoal(extTarget);     
    }

    public void setArmWristGoal(double armAbsTarget, double wristTarget){
            setArmGoal(armAbsTarget);
            setWristSetpoint(wristTarget);   
    }

    public double getGoal(){
        // return armPid.getGoal().position;
        return armAbsTarget;
    }

    public double newGetAbsArmTarget(){
        return armAbsTarget;
    }


    public double calculateArmClimbPID() {
        return armClimbPid.calculate(getArmRelPos(), armAbsTarget);
    }
    public void updateArmClimbPID(){
        
        double ffValue = 0;
        double pidValue = calculateArmClimbPID();

        double voltage = pidValue + ffValue;
        SmartDashboard.putNumber("Rot Climb Voltage", voltage);

        //voltage = MathUtil.clamp(voltage, -4, 4);

        // double voltage = convertToVolts(percentOutput);
        // SmartDashboard.putNumber("percentOutput", percentOutput);
        // SmartDashboard.putNumber("Rot Climb FF", ffValue);
        SmartDashboard.putNumber("PIDRotate Climb", pidValue);
        
       // boolean limit = (getAbsArmPos() >= armHardUpperLimit && Math.signum(voltage) == 1.0) || (getAbsArmPos() <=  && Math.signum(voltage) == -1.0);
       // if(limit){
            //Technically should set a ff constant negative 
            //Mainly b/c of the limit on the chain rn(if gone can remove this if statment)
        //    setArmVoltage(0);
        //}else{
        if (Math.abs(voltage) < 12) {
            setArmVoltage(voltage);
        } else {
            setArmVoltage(12 * Math.signum(voltage));
        }

        //} 
    }


    
    public void updateRotationOutput () {
         if (!armPIDRun) {
            return;
        }
        double error = armAbsTarget - getArmRelPos();
        // double pidValue = calculateRotationPID();
        double pidValue = (error) * 55;
        
        // the complicated BigDecimal is to limit to 3 decimal places
        BigDecimal limDecimalPlaces = new BigDecimal(armAbsTarget).setScale(5, RoundingMode.HALF_UP);
        
        
        Logger.recordOutput("Arm/PIDValue", pidValue);
        Logger.recordOutput("Arm/maxVoltage", maxArmVoltage);
        Logger.recordOutput("Arm/armTarget", armAbsTarget);

        if (Math.abs(pidValue) < maxArmVoltage) { //10 volts good for tele
            setArmVoltage(pidValue);
        } else {
            setArmVoltage(maxArmVoltage * Math.signum(pidValue));
        }

        // basically if the difference between the abs and rel pos is enough, we know chain probably slipped
        // since the rel encoder is off from the abs differently from climb and store positions, we only look at climb
        // so if we are goign to the climb pos, and the error is within an amount, and the abs and rel are sufficiently off,
        // then updateRelArmPos
        
        if (limDecimalPlaces.doubleValue() == (armHardLowerLimit + Constants.Arm.offsetToClimb) && Math.abs(getAbsArmPos() - getArmRelPos()) > 0.01619 && Math.abs(error) < 0.01) {
            updateRelArmPos();
        } else if (limDecimalPlaces.doubleValue() == (armHardLowerLimit + Constants.Arm.offsetToClimb) && Math.abs(error) < 0.006 && Math.abs(armRelEncoder.getVelocity()) < 10) { // update every time going to climb pos
            // System.out.println("UPDATED ARM");
            updateRelArmPos();
        }
        // System.out.println("Arm Abs Target: " + limDecimalPlaces.doubleValue() + " Math.abs(error): " + Math.abs(error) + " Velocity: " + Math.abs(armRelEncoder.getVelocity()));
    }





    /*
     * 
     * Wrist Methods
     * 
     */







    double oldVel = 0;
    double oldTime = 0;
    double oldPos = 0;


    public void updateWristPos() {
        double goalPos = wristPIDController.getSetpoint();
        Logger.recordOutput("Wrist/WristTarget", goalPos);

        double pidValue = wristPIDController.calculate(getAbsWristPos(), goalPos); //change back to goalPos after testing
        double changeInTime = Timer.getFPGATimestamp() - oldTime;
        double velSetpoint = (getAbsWristPos() - oldPos) / changeInTime;
        double accel = (velSetpoint - oldVel) / (changeInTime); 
        double ffVal = wristFeedForward.calculate(velSetpoint, accel); //takes velocity, and acceleration
        
        double voltage = MathUtil.clamp(pidValue /*+ ffVal*/, -5.0, 5.0);
        double maxWristVoltage = 4.5; // magnitude of voltage
        
        Logger.recordOutput("Wrist/maxVoltage", maxWristVoltage);
        // SmartDashboard.putNumber("Wrist PID", pidValue);
        // SmartDashboard.putNumber("Wrist FF", ffVal);
        // SmartDashboard.putNumber("Wrist Voltage", voltage);
        // SmartDashboard.putNumber("Wrist Pos Error", wristPIDController.getPositionError());
      
    /*
        if (Math.abs(getWristAbsPos() - goalPos) <= 0.05) { // no voltage
          wrist.setVoltage(0);
        } else { // set voltage
          if (getWristAbsPos() <= wristHardLowerLimit && getWristAbsPos() >= wristHardUpperLimit) { // is between max and min
            if(getWristAbsPos() >= wristHardLowerLimit && getWristAbsPos() <= wristHardUpperLimit){
                wrist.setVoltage(-voltage); // could be negative voltage based upon direction of motor and gear
            }else{
                wrist.setVoltage(0);
            }
            
          }
        }
        */
         if (Math.abs(voltage) < maxWristVoltage) {
            setWristVoltage(-voltage);
        } else {
           setWristVoltage(-maxWristVoltage * Math.signum(voltage));
        }
        
        // update vars for determining acceleration later
        oldVel =  velSetpoint; 
        oldTime = Timer.getFPGATimestamp(); 
        oldPos = getAbsWristPos();
    }

    public void setWristVoltage(double voltage) {
        Logger.recordOutput("Wrist/WristVoltage", voltage);

        wrist.setVoltage(voltage);

    }


    public void setWristSetpoint(double goalPos) {
        wristPIDController.setSetpoint(goalPos);
        this.wristTarget = goalPos;
        
    }

/* *Could be used in future
    public void toggleWrist() {
        wristPIDRun = !wristPIDRun;
    }
    public void toggleArm() {
        armPIDRun = !armPIDRun;
    }
*/








/*
 * 
 *  Periodic
 * 
 */


   

    // --------------------------------------------------------------

    public void changeHeight(Height height){
        lastHeight = height;
    }

    public Height getHeight(){
        return lastHeight;
    }


    public boolean getWasSourceIntake(){
        return wasSourceIntake;
    }

    public void setWasSourceIntake(boolean intake){
        wasSourceIntake = intake;
    }
    
    public void setAllBrake(boolean brake) {
        if (brake) {
            armBackMotor.setIdleMode(IdleMode.kBrake);
            armFrontMotor.setIdleMode(IdleMode.kBrake);
            wrist.setIdleMode(IdleMode.kBrake);
            extensionMotor.setIdleMode(IdleMode.kBrake);

        } else {
            armBackMotor.setIdleMode(IdleMode.kCoast);
            armFrontMotor.setIdleMode(IdleMode.kCoast);
            wrist.setIdleMode(IdleMode.kCoast);
            extensionMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    
    @Override
    public void periodic(){
        

       
        if (isClimbing) {
            updateArmClimbPID();
        } else {
            updateRotationOutput();
            updateWristPos();
            


            if (!runAndResetExt) {
                updateExtensionOutput();
            }


        }


        /* Stops motor and resets encoder after limit switch reached */
        if (getLowerLimSwitch() && runAndResetExt) {
            extensionMotor.stopMotor();
            setExtVoltage(0);
            extRelEncoder.setPosition(0);
            runAndResetExt = false;

        }
    

        // SmartDashboard.putNumber("Max Arm Voltage", maxArmVoltage);
        SmartDashboard.putNumber("Arm Rel Position", getArmRelPos());
        SmartDashboard.putNumber("Ratio Abs/Rel arm", getAbsArmPos() / getArmRelPos());
        SmartDashboard.putNumber("Arm Setpoint", armAbsTarget);
        // SmartDashboard.putNumber("Ext Rel Pos", getExtRelPos());
        // SmartDashboard.putNumber("Front Arm Motor Current", armFrontMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Back Arm Motor Current", armBackMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Front Arm Pos", armFrontMotor.getEncoder().getPosition());
        // SmartDashboard.putNumber("Back Arm Pos", armBackMotor.getEncoder().getPosition());


    }



    //---------------------------------------------------

    
    // true is source, false is ground
    private boolean ampPos;
    public void goToPosition(Height pos) {
        if (pos == Height.CLIMB || pos == Height.TRAP) {
            setMaxArmVoltage(4); // 4
            if (pos == Height.CLIMB) {
                pos = Height.HOLD;
            }
        } else {
            setMaxArmVoltage(6); // 6
        }
      
        if(pause && reachPos){
            //do nothing
        }else{             

            if(!ampPos && pos == Height.AMP){
                lastHeight = Height.AMP;
                groundToAmp = true;

                // don't want to update lastHeight for amp
                
                // setArmWristExtGoal(0.531, 0.15, 0.25);
                setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToAmpFromGround, 
                                wristHardLowerLimit + Constants.Wrist.offsetToAmpFromGround, 
                                extHardLowerLimit - 5); // wrist from smallest 0.117

            }else if((ampPos || lastHeight == Height.HOLD) && pos == Height.AMP){
                if (lastHeight == Height.HOLD) {
                    holdToAmp = true;
                    setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToAmpFromGround, 
                                wristHardLowerLimit + Constants.Wrist.offsetToAmpFromSource_Hold, 
                                extHardLowerLimit - 5);
                } else {
                    setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToAmpFromGround, 
                                wristHardLowerLimit + Constants.Wrist.offsetToAmpFromSource_Hold, 
                                extHardLowerLimit + Constants.Extension.offsetToAmpFromSource_Hold);
                }

                lastHeight = Height.AMP;

            }else if(lastHeight == Height.HOLD && pos == Height.GROUND){
                /*  lastHeight gets updated for this in the periodic method */
                ampPos = false;

                // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
                // move arm to purpendicular (0.21), then move extension and wrist simultaneously while moving arm down
                // this boolean is a way to determine the "range" for when stuff starts moving after the arm, as well as some logic
                wristPIDRun = false;
                
                // the below will make arm go to 90 degree pos (logic in periodic method)
                setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToStraightOutPos, 
                                wristHardLowerLimit + Constants.Wrist.offsetToGround, 
                                extHardLowerLimit + Constants.Extension.offsetToGround); //extTarget = 0.387

            }else if(lastHeight == Height.GROUND && pos == Height.HOLD){
                /*  lastHeight gets updated for this in the periodic method */

                // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
                // move arm to purpendicular (0.21) while moving extension and wrist simultaneously, and then move arm down
                // this boolean is a way to determine the "range" for when stuff starts moving after the arm, as well as some logic
                wristPIDRun = false;
                
                // the below will make arm go to 90 degree pos (logic in periodic method)
                setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToStraightOutPos, 
                                wristHardLowerLimit + Constants.Wrist.offsetToHold, 
                                extHardLowerLimit + Constants.Extension.offsetToHold); //extTarget = 0.387

            } else if (pos == Height.GROUND && lastHeight == Height.AMP) {

                lastHeight = Height.GROUND;
                ampPos = false;

                ampToGround = true;

                setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToGround, 
                                wristHardLowerLimit + Constants.Wrist.offsetToGround, 
                                extHardLowerLimit - 5);
            } else if(pos == Height.GROUND){
                lastHeight = Height.GROUND;
                ampPos = false;
            
                // setArmWristExtGoal(0.1716, 0.51, 0.463); //extTarget = 0.387
                setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToGround, 
                                wristHardLowerLimit + Constants.Wrist.offsetToGround, 
                                extHardLowerLimit + Constants.Extension.offsetToGround); //extTarget = 0.387

            }else if(pos == Height.HOLD){
                lastHeight = Height.HOLD;
            
                // setArmWristExtGoal(0.13, 0.05, 0.73); 
                setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToHold, 
                                wristHardLowerLimit + Constants.Wrist.offsetToHold, 
                                extHardLowerLimit + Constants.Extension.offsetToHold); 

            }else if(pos == Height.SOURCE){
                ampPos = true;
                // setArmWristExtGoal(0.39, 0.42, 0.55); //extTarget = 0.5346 wristTarget = 0.33
                // setArmWristExtGoal(0.37, 0.387, 0.55); //extTarget = 0.5346
                
                if (firstStartingBot) {
                    setArmGoal(armHardLowerLimit + Constants.Arm.offsetToSource);
                    setWristSetpoint(wristHardLowerLimit + Constants.Wrist.offsetToSource);
                    firstStartingBot = false;
                } else if (lastHeight == Height.HOLD) {
                    holdToSource = true;
                    setArmGoal(armHardLowerLimit + Constants.Arm.offsetToSource);
                    setWristSetpoint(wristHardLowerLimit + Constants.Wrist.offsetToSource);
                } else {
                    setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToSource, 
                                wristHardLowerLimit + Constants.Wrist.offsetToSource, 
                                extHardLowerLimit + Constants.Extension.offsetToSource); //extTarget = 0.5346
                }
            
                lastHeight = Height.SOURCE;

                
                

            } else if (pos == Height.PRECLIMB) {
                lastHeight = Height.AMP;

                setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToClimb, 
                                wristHardLowerLimit + Constants.Wrist.offsetToAmpFromGround + 0.25, 
                                extHardLowerLimit - 5);

            } else if (pos == Height.TRAP) {
                lastHeight = Height.AMP;
                setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToAmpFromGround - 0.02, 
                                wristHardLowerLimit + Constants.Wrist.offsetToAmpFromGround, 
                                extHardLowerLimit + Constants.Extension.offsetToAmpFromGround - 12);
            }
            
            
            pause = !pause;

        }
    }

}