package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmWristSubsystem extends SubsystemBase{
    public enum Height{
        GROUND,
        HOLD,
        SOURCE,
        AMP,
    }
    private Height lastHeight = Height.HOLD;

    
    //Extension Limiter ----
    private double radiusX, radiusY, extensionLength, totalextensionX, totalextensionY, theta;
    //----
    private double armTarget = 0.37;//0.485;  //.453
    private double wristTarget = 0.387;
    private double extensionTarget = 0.72;

    private double armHardLowerLimit = 0.105;//0.08;
    private double armHardUpperLimit = 0.7;//0.51;
    private double wristHardLowerLimit = 0.141; // 0.033
    private double wristHardUpperLimit = 0.7785; // 0.632
    private double extHardLowerLimit = 0.749; 
    private double extHardUpperLimit = 0.059;

    private double armStraightUp = 0.46;
    private double armBalanced = 0.36;
    private double armPurpenGround = 0.206;
    private double armFFWhenPurpen = 0.42;
    
    boolean wristBrakeToggle;
    boolean wristPIDRun;
    boolean armPIDRun;

    

    /* Motors */
    private CANSparkMax armMotor = new CANSparkMax(Constants.armId, MotorType.kBrushless);
    private CANSparkFlex wrist = new CANSparkFlex(Constants.wristId, MotorType.kBrushless);
    private CANSparkMax extensionMotor = new CANSparkMax(Constants.extensionId, MotorType.kBrushless);

     /* Absolute Encoder */
    SparkAbsoluteEncoder wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
    SparkAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    SparkAbsoluteEncoder extEncoder = extensionMotor.getAbsoluteEncoder(Type.kDutyCycle);

    
    private boolean runStuff = true;

     // extension all the way in kg 0.12(0.89 for ext) 0.19 middle(0.47 for ext pos) and 0.24(0.145 for ext) full extended
    // Equation for this is ffVal = -0.16134 * ExtPos + 0.26427
    DoubleSupplier armFFkg = () -> 0.065;
    private ArmFeedforward armFF = new ArmFeedforward(0, 0.065,0); //0.027, 0.00001 => halfway is 0.013505
    public SimpleMotorFeedforward wristFeedForward = new SimpleMotorFeedforward(0.00001, 0.00003, 0.00001); 

    

    private final ProfiledPIDController armPid = new ProfiledPIDController(40, 0, 0, new Constraints(4, 3));//maxVel = 3.5 and maxAccel = 2.5
    private final PIDController wristPIDController = new PIDController(40, 0, 0); 
    private final PIDController extensionPid = new PIDController(45, 0,0);


    private double ticsPerArmRevolution = 144, ticsPerWristRevolution = /*172.8*/ 120, lowTics = (50/360) * ticsPerArmRevolution, midTics = (100/360) * ticsPerArmRevolution, highTics = (135/360) * ticsPerArmRevolution, groundTics = (37.4/360) * ticsPerArmRevolution;
    private boolean intialization = true;



    public ArmWristSubsystem(){
        armMotor.restoreFactoryDefaults();
        extensionMotor.restoreFactoryDefaults();
        wrist.restoreFactoryDefaults();


        armMotor.setSmartCurrentLimit(40);
        extensionMotor.setSmartCurrentLimit(40);

            
    
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
        // armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
        // armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
        // armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency

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


        wrist.setIdleMode(IdleMode.kBrake);
        armMotor.setIdleMode(IdleMode.kCoast);
        extensionMotor.setIdleMode(IdleMode.kCoast);

        wristPIDRun = true;
        armPIDRun = true;
        wristBrakeToggle = false;
        
        armMotor.burnFlash();
        wrist.burnFlash();
        extensionMotor.burnFlash();

        goToPosition(Height.SOURCE);


        SmartDashboard.putNumber("Change Arm Target", armTarget);
        SmartDashboard.putNumber("Change Wrist Target", Constants.midWristPos);
        SmartDashboard.putNumber("Change ArmFF", 0.065);
        SmartDashboard.putNumber("Change Arm Is Brake", 1);
        SmartDashboard.putNumber("Change Wrist Is Brake", 1);
        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Change Extension Target", 0.47);
        SmartDashboard.putNumber("Change Extension Is Brake", 1);

    }



    



    /*
     * 
     * Extension Methods
     * 
     */





    public void setExtensionVoltage(double voltage){
        extensionMotor.setVoltage(voltage);
    }
    public double getExtensionAbsPosition(){  
        return extEncoder.getPosition();
    }
    public void setExtensionGoal(double target){

        extensionTarget = target;
    }
    public double calculateExtensionPID(){
        return extensionPid.calculate(getExtensionAbsPosition(), extensionTarget);
    }
    public void updateExtensionOutput(){
        double ffValue = calculateExtensionFF();
        SmartDashboard.putNumber("Extension FF", ffValue);
        double voltage = MathUtil.clamp(calculateExtensionPID(), -12.0, 12.0);
        // SmartDashboard.putNumber("Extension Percent", percentOutput);

        // double voltage = 12 * percentOutput;

        // voltage = MathUtil.clamp(voltage /*+ ffValue*/, -6, 6);

        SmartDashboard.putNumber("Extension Voltage", voltage);

         if (Math.abs(voltage) < 8) {
            setExtensionVoltage(voltage);
        } else {
            setExtensionVoltage(8 * Math.signum(voltage));
        }
    }
    public double calculateExtensionFF() {
        //return (-1 * Math.Abs((1.44 * getArmPosition()) - 0.7632)) + 0.135;
        return 0;
    }








    /*   
     * 
     * Arm Methods
     * 
     */
 







    public double getAbsArmPos(){
        return armEncoder.getPosition();
    }
    public double getAbsWristPos(){
        return wristEncoder.getPosition();
    }
 
    
    
    /**
     * For equation derivation, see {@link https://www.desmos.com/calculator/ygpschqwqe}
     * @return returns a linear modifier from 1 to 1.4375 to multiply the armFF equation by to account for extension
     */
    public double updateArmFF_extension(){
        //return (-0.16134 * extPosition + armFFkg.getAsDouble());
        // return -0.56966 * getExtensionAbsPosition() + 1.39876302083; // old b value was 1.52295
        double slope = (1 - 1.4375) / (extHardLowerLimit - extHardUpperLimit);
        return slope * getExtensionAbsPosition() + (1 - slope * extHardLowerLimit);
    }
    

    // cosine equation FF, see https://www.desmos.com/calculator/ygpschqwqe
    public double getFFEquationVoltage() {
        
        return  armFFWhenPurpen * updateArmFF_extension() * Math.cos((2*Math.PI/((armBalanced - armPurpenGround)*4)) * (getAbsArmPos() - armPurpenGround)); 
    } 
    public void setArmVoltage(double voltage){
        // leftMotor.setVoltage(-voltage);

       armMotor.setVoltage(voltage);
    }
    public void downManual(){
        armTarget -= 0.01;

        //wrist.setGoal(2.57 - armTarget);
    }
    public void upManual(){
        armTarget += 0.01;

        //wrist.setGoal(2.57 - armTarget);
    }

    public void setArmGoal(double target) {
        //rotation.setGoal(target);
        armTarget = target;
    }

    public void setArmWristExtGoal(double armTarget, double wristTarget, double extTarget){
        setArmGoal(armTarget);
        setWristSetpoint(wristTarget);
        setExtensionGoal(extTarget);
    }
    public double getGoal(){
        return armPid.getGoal().position;
    }
    public double calculateRotationPID(){
        //return armPid.calculate(getArmPosition(), armTarget);
        return armPid.calculate(getAbsArmPos(), armTarget);
    }
    public void updateRotationOutput(){
        
        double ffValue = getFFEquationVoltage()/*calculateRotationFF()*/;
        double pidValue = calculateRotationPID();

        double voltage = pidValue + ffValue;
        SmartDashboard.putNumber("Rotation Voltage", voltage);

        //voltage = MathUtil.clamp(voltage, -4, 4);

        // double voltage = convertToVolts(percentOutput);
        // SmartDashboard.putNumber("percentOutput", percentOutput);
        SmartDashboard.putNumber("Rotation FF", ffValue);
        SmartDashboard.putNumber("PIDRotate", pidValue);
        
       // boolean limit = (getAbsArmPos() >= armHardUpperLimit && Math.signum(voltage) == 1.0) || (getAbsArmPos() <=  && Math.signum(voltage) == -1.0);
       // if(limit){
            //Technically should set a ff constant negative 
            //Mainly b/c of the limit on the chain rn(if gone can remove this if statment)
        //    setArmVoltage(0);
        //}else{
        if (Math.abs(voltage) < 4) {
            setArmVoltage(voltage);
        } else {
            setArmVoltage(4 * Math.signum(voltage));
        }

        //} 
    }
    public double calculateRotationFF(){
        // if(extensionTarget == 30){
        //     return armExtendedFF.calculate(getArmPosition(), armPid.getSetpoint().velocity);
        // }

        //return armFF.calculate(getArmPosition(), armPid.getSetpoint().velocity);
        //return armFF.calculate(getAbsArmPos(), armPid.getSetpoint().velocity);
        if (getAbsArmPos() >= 0.46) {
            return -armFF.calculate(getAbsArmPos(), 0);
        } else {
            return armFF.calculate(getAbsArmPos(), 0);
        }
    }
    
    private double convertToVolts(double percentOutput){
        return percentOutput * /*Robot.getInstance().getVoltage()*/ 12;
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
        double pidValue = wristPIDController.calculate(getWristAbsPos(), goalPos); //change back to goalPos after testing
        double changeInTime = Timer.getFPGATimestamp() - oldTime;
        double velSetpoint = (getWristAbsPos() - oldPos) / changeInTime;
        double accel = (velSetpoint - oldVel) / (changeInTime); 
        double ffVal = wristFeedForward.calculate(velSetpoint, accel); //takes velocity, and acceleration
        
        double voltage = MathUtil.clamp(pidValue /*+ ffVal*/, -5.0, 5.0);
        
        
        SmartDashboard.putNumber("Wrist PID", pidValue);
        SmartDashboard.putNumber("Wrist FF", ffVal);
        SmartDashboard.putNumber("Wrist Voltage", voltage);
        SmartDashboard.putNumber("Wrist Pos Error", wristPIDController.getPositionError());
      
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
         if (Math.abs(voltage) < 3) {
            wrist.setVoltage(-voltage);
        } else {
           wrist.setVoltage(-3 * Math.signum(voltage));
        }
        
        // update vars for determining acceleration later
        oldVel =  velSetpoint; 
        oldTime = Timer.getFPGATimestamp(); 
        oldPos = getWristAbsPos();
    }

    public void setWristSetpoint(double goalPos) {
        wristPIDController.setSetpoint(goalPos);
        
    }

    public void toggleWrist() {
        wristPIDRun = !wristPIDRun;
    }
    public void toggleArm() {
        armPIDRun = !armPIDRun;
    }


    public double getWristPos() { return wrist.getEncoder().getPosition(); }
    public double getWristAbsPos() { return wristEncoder.getPosition(); }
    public void stopWrist() { wrist.stopMotor(); }
    public void toggleWristBrake() { wrist.setIdleMode(wristBrakeToggle ? IdleMode.kBrake : IdleMode.kCoast); wristBrakeToggle = !wristBrakeToggle; }
    







/*
 * 
 *  Periodic
 * 
 */



   



    @Override
    public void periodic(){
    
                
        theta = (((Constants.Arm.intakeArmAngle - Constants.Arm.ampArmAngle)/(Constants.Arm.intakeArmTics - Constants.Arm.ampArmTics)) * getAbsArmPos()) + (Constants.Arm.intakeArmAngle - (((Constants.Arm.intakeArmAngle - Constants.Arm.ampArmAngle)/(Constants.Arm.intakeArmTics - Constants.Arm.ampArmTics)) * Constants.Arm.intakeArmTics));
        radiusX = Constants.Arm.armLength - Math.abs((getWristAbsPos() - Constants.Wrist.straightWristTics) / ((Constants.Wrist.upWristTics-Constants.Wrist.straightWristTics)/(Constants.Wrist.straightWristInches-Constants.Wrist.upWristInches))) + Constants.Wrist.straightWristInches;
        radiusY = radiusX + 1;
        extensionLength = (Constants.Extension.maxExtensionInches / (Constants.Extension.maxExtensionTics - Constants.Extension.zeroTics)) * getExtensionAbsPosition() - (Constants.Extension.zeroTics * (Constants.Extension.maxExtensionInches/(Constants.Extension.maxExtensionTics - Constants.Extension.zeroTics)));
        totalextensionX = ((radiusX + extensionLength) * Math.abs(Math.cos(theta))) - Constants.Chassis.pivotToFront;
        totalextensionY = ((radiusY + extensionLength) * Math.sin(theta));
        SmartDashboard.putNumber("totalExtensionX", totalextensionX);
        SmartDashboard.putNumber("totalExtensionY", totalextensionY);
        SmartDashboard.putNumber("extensionLength", extensionLength);
        SmartDashboard.putNumber("radiusX", radiusX);
        SmartDashboard.putNumber("theta", theta);
        SmartDashboard.putNumber("Wrist Extension", (-Math.abs((getWristAbsPos() - Constants.Wrist.straightWristTics) / ((Constants.Wrist.upWristTics-Constants.Wrist.straightWristTics)/(Constants.Wrist.straightWristInches-Constants.Wrist.upWristInches))) + Constants.Wrist.straightWristInches));



        
        if(runStuff){
            if(armPIDRun){
                updateRotationOutput();
            }else{
                armMotor.setVoltage(0);
            }
           
            if (wristPIDRun) {
                    updateWristPos();
                } else {
                    wrist.setVoltage(0);
            }

          

            if(Math.abs(getAbsArmPos() - armTarget) < 0.2){
                

                
                updateExtensionOutput();
            }
            

        
        }else{
            setArmVoltage(0);
            wrist.setVoltage(0);
        }
        //SmartDashboard.putNumber("LeftPosition", getLeftPosition());
        SmartDashboard.putNumber("Arm Position", getAbsArmPos());
        SmartDashboard.putNumber("Arm Target", armTarget);
        SmartDashboard.putNumber("Wrist Pos", getAbsWristPos());
        SmartDashboard.putBoolean("Wrist is Brake", wrist.getIdleMode() == IdleMode.kBrake ? true : false);
        SmartDashboard.putNumber("Arm Actual Voltage", armMotor.getOutputCurrent()*0.6);
        SmartDashboard.putNumber("ArmFF kg", armFF.kg);
        SmartDashboard.putNumber("FF Equation Value", getFFEquationVoltage());
        SmartDashboard.putBoolean("Wrist Enabled", wristPIDRun);
        SmartDashboard.putBoolean("Arm Enabled", armPIDRun);
        SmartDashboard.putNumber("Wrist Target", wristTarget);
        SmartDashboard.putNumber("Extension Position", getExtensionAbsPosition());
        SmartDashboard.putNumber("Extension Target", extensionTarget);
        if (SmartDashboard.getNumber("Change Wrist Is Brake", 1) == 1) {
            extensionMotor.setIdleMode(IdleMode.kBrake);
        } else {
            extensionMotor.setIdleMode(IdleMode.kCoast);
        }

        armFFkg = () -> SmartDashboard.getNumber("Change ArmFF", 0.065);
        if (SmartDashboard.getNumber("Change Arm Is Brake", 1) == 1) {
            armMotor.setIdleMode(IdleMode.kBrake);
        } else {
            armMotor.setIdleMode(IdleMode.kCoast);
        }
        if (SmartDashboard.getNumber("Change Wrist Is Brake", 1) == 1) {
            wrist.setIdleMode(IdleMode.kBrake);
        } else {
            wrist.setIdleMode(IdleMode.kCoast);
        }
        
       // wristTarget = SmartDashboard.getNumber("Change Wrist Target", 0.68);
        //armTarget = SmartDashboard.getNumber("Change Arm Target", armTarget);



        //SmartDashboard.putNumber("Wrist Error", wrist.getPositionError());
        //SmartDashboard.putNumber("Position Error", rotation.getPositionError());
        
        //SmartDashboard.putNumber("Wrist Position", getWristPosition());
        //SmartDashboard.putNumber("Arm current", rightMotor.getOutputCurrent());
         
    }

    
    
    


    // public void example(){
    //     Shuffleboard.getTab("Arm")
    //     .add("Arm P value", p)
    //     .withSize(2, 1)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("Min", 0, "Max", 0.025))
    //     .getEntry();
        
    // }
    

    public void anEmptyMethod() {
        // for testing
    }
    // public double getLeftRotPos() {
    //     return leftMotor.getEncoder().getPosition() ;
    // }

    public double convertToRads(double angle) {
        return angle/360*2*Math.PI;
    }

    // --------------------------------------------------------------

    public void changeHeight(Height height){
        lastHeight = height;
    }

    public Height getHeight(){
        return lastHeight;
    }

    public void goToPosition(Height pos) {

        if(lastHeight == Height.GROUND && pos == Height.AMP){
            // don't want to update lastHeight for amp

            // setArmWristExtGoal(0.531, 0.15, 0.25);
            setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToAmpFromGround, 
                            wristHardLowerLimit + Constants.Wrist.offsetToAmpFromGround, 
                            extHardLowerLimit + Constants.Extension.offsetToAmpFromGround); // wrist from smallest 0.117

        }else if((lastHeight == Height.SOURCE || lastHeight == Height.HOLD) && pos == Height.AMP){
            setArmWristExtGoal(0.511, 0.0487, 0.34); //extTarget = 0.387
            // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
            setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToAmpFromSource_Hold, 
                            wristHardLowerLimit + Constants.Wrist.offsetToAmpFromSource_Hold, 
                            extHardLowerLimit + Constants.Extension.offsetToAmpFromSource_Hold); //extTarget = 0.387

        }else if(lastHeight == Height.HOLD && pos == Height.GROUND){
            // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
            // move arm to purpendicular (0.21), then move extension and wrist simultaneously, and then move arm down
            setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToGroundFromHold, 
                            wristHardLowerLimit + Constants.Wrist.offsetToGround, 
                            extHardLowerLimit + Constants.Extension.offsetToGround); //extTarget = 0.387

        }else if(pos == Height.GROUND){
            lastHeight = Height.GROUND;
          
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
            setArmWristExtGoal(0.39, 0.42, 0.55); //extTarget = 0.5346 wristTarget = 0.33
            // setArmWristExtGoal(0.37, 0.387, 0.55); //extTarget = 0.5346
            setArmWristExtGoal(armHardLowerLimit + Constants.Arm.offsetToSource, 
                            wristHardLowerLimit + Constants.Wrist.offsetToSource, 
                            0.75/*extHardLowerLimit + Constants.Extension.offsetToSource*/ ); //extTarget = 0.5346

        }
       
    }

}