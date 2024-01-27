package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
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
        LOW,
        MID,
        HIGH,
        GROUND,
    }
    private Height currentHeight = Height.GROUND;

    private double armTarget = 0.453;//0.53;

    boolean wristBrakeToggle;
    boolean wristPIDRun;

    //double p = 0;

    //public CANSparkMax intake = new CANSparkMax(Constants.intakeId, MotorType.kBrushless);

    // private CANSparkMax leftMotor = new CANSparkMax(Constants.rotationLeftId,MotorType.kBrushless);
    private CANSparkMax armMotor = new CANSparkMax(Constants.armId, MotorType.kBrushless);
    private CANSparkMax wrist = new CANSparkMax(Constants.wristId, MotorType.kBrushless);

    
    
    private boolean runStuff = true;

    private final PIDController wristPIDController = new PIDController(0.9, 0, 0); 
    public SimpleMotorFeedforward wristFeedForward = new SimpleMotorFeedforward(0.00001, 0.00003, 0.00001); 

    /* Absolute Encoder */
  // if absolute encoder plugged into cansparkmax:
    //CANSparkMax wristmax = new CANSparkMax(Constants.wristId, MotorType.kBrushless);
    SparkAbsoluteEncoder wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
    SparkAbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private final ProfiledPIDController armPid = new ProfiledPIDController(6.1, 0, 0, new Constraints(1, 0.5));//maxVel = 3.5 and maxAccel = 2.5
    private final ArmFeedforward armFF = new ArmFeedforward(0, 0.1, 20); //0.027, 0.00001
    private final ArmFeedforward armExtendedFF = new ArmFeedforward(0, 0.03, 0.00001);

    //Extension

    private PIDController pid = new PIDController(0.1, 0, 0), downPID = new PIDController(0.0085, 0, 0);
    
    Timer timer = new Timer();
    private double ticsPerArmRevolution = 144, ticsPerWristRevolution = /*172.8*/ 120, lowTics = (50/360) * ticsPerArmRevolution, midTics = (100/360) * ticsPerArmRevolution, highTics = (135/360) * ticsPerArmRevolution, groundTics = (37.4/360) * ticsPerArmRevolution;
    private boolean intialization = true;

    DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);


    public ArmWristSubsystem(){
        armMotor.restoreFactoryDefaults();
        

        armMotor.setSmartCurrentLimit(40);
        
        armMotor.setIdleMode(IdleMode.kBrake);
        
        
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
    
        armPid.reset(getArmPosition());

        wrist.setIdleMode(IdleMode.kBrake);
        wristPIDRun = false;
        wristBrakeToggle = false;
        
        armMotor.burnFlash();
        wrist.burnFlash();
        setArmGoal(0.453);

    }

    public double getArmPosition(){
        return absEncoder.get();
    }
    public double getAbsArmPos(){
        return armEncoder.getPosition();
    }

    // private double getLeftPosition(){
    //     return leftMotor.getEncoder().getPosition() * -2.5 * Math.PI / 180;
    // }

    // private double getPosition(){
    //     return (getLeftPosition() + getRightPosition()) / 2;
    // }


    // public void setLeftVoltage(double voltage){
    //     leftMotor.setVoltage(-voltage);
    // }

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

    public void setArmWristGoal(double armTarget, double wristTarget){
        setArmGoal(armTarget);
        setWristSetpoint(wristTarget);

    }

    

    public double getGoal(){
        return armPid.getGoal().position;
    }

    public double calculateRotationPID(){
        //return armPid.calculate(getArmPosition(), armTarget);
        return armPid.calculate(getAbsArmPos(), armTarget);
    }
    
    

    
    



    

    public void updateRotationOutput(){
        double ffValue = calculateRotationFF();
        double pidValue = calculateRotationPID();
        double percentOutput = MathUtil.clamp(ffValue, -1, 1);
        double voltage = convertToVolts(percentOutput);
        SmartDashboard.putNumber("percentOutput", percentOutput);
        SmartDashboard.putNumber("Rotation FF", ffValue);
        SmartDashboard.putNumber("PIDRotate", pidValue);
        SmartDashboard.putNumber("Rotation Voltage", voltage);
        
        
        setArmVoltage(voltage);
        
        
    }
    



    

    public double calculateRotationFF(){
        // if(extensionTarget == 30){
        //     return armExtendedFF.calculate(getArmPosition(), armPid.getSetpoint().velocity);
        // }

        //return armFF.calculate(getArmPosition(), armPid.getSetpoint().velocity);
        //return armFF.calculate(getAbsArmPos(), armPid.getSetpoint().velocity);
        return armFF.calculate(getAbsArmPos(), 5);
    }
    

    private double convertToVolts(double percentOutput){
        return percentOutput * Robot.getInstance().getVoltage();
    }

    @Override
    public void periodic(){
        if(runStuff){
            updateRotationOutput();
           // updateWristPos();


        
        }else{
            setArmVoltage(0);
        }
        //SmartDashboard.putNumber("LeftPosition", getLeftPosition());
        SmartDashboard.putNumber("Arm Position", getAbsArmPos());
        SmartDashboard.putNumber("Target", armTarget);
        SmartDashboard.putNumber("Wrist Pos", getWristAbsPos());
        SmartDashboard.putBoolean("Wrist is Brake", wrist.getIdleMode() == IdleMode.kBrake ? true : false);

        


        //SmartDashboard.putNumber("Wrist Error", wrist.getPositionError());
        //SmartDashboard.putNumber("Position Error", rotation.getPositionError());
        
        //SmartDashboard.putNumber("Wrist Position", getWristPosition());
        //SmartDashboard.putNumber("Arm current", rightMotor.getOutputCurrent());
        

        
    }

    double oldVel = 0;
    double oldTime = 0;
    double oldPos = 0;


    public void updateWristPos() {
        double goalPos = wristPIDController.getSetpoint();
        double pidValue = wristPIDController.calculate(getWristAbsPos(), goalPos);
        double changeInTime = Timer.getFPGATimestamp() - oldTime;
        double velSetpoint = (getWristAbsPos() - oldPos) / changeInTime;
        double accel = (velSetpoint - oldVel) / (changeInTime); 
        double ffVal = wristFeedForward.calculate(velSetpoint, accel); //takes velocity, and acceleration
        
        double percentOutput = MathUtil.clamp(pidValue + ffVal, -1.0, 1.0);
        double voltage = convertToVolts(percentOutput);
        
        
        SmartDashboard.putNumber("PID Value", pidValue);
        SmartDashboard.putNumber("Feed Forward", ffVal);
        SmartDashboard.putNumber("Voltage", voltage);
        SmartDashboard.putNumber("Position error", wristPIDController.getPositionError());
      
    
        if (Math.abs(getWristAbsPos() - goalPos) <= 0.05) { // no voltage
          wrist.setVoltage(0);
        } else { // set voltage
          if (getWristAbsPos() <= Constants.maxWristPos && getWristAbsPos() >= Constants.minWristPos) { // is between max and min
            wrist.setVoltage(voltage); // could be negative voltage based upon direction of motor and gear
          }
        }
        
        // update vars for determining acceleration later
        oldVel =  velSetpoint; 
        oldTime = Timer.getFPGATimestamp(); 
        oldPos = getWristAbsPos();
    }

    public void setWristSetpoint(double goalPos) {
        wristPIDController.setSetpoint(goalPos);
        wristPIDRun = true;
    }


    public double getWristPos() { return wrist.getEncoder().getPosition(); }
    public double getWristAbsPos() { return wristEncoder.getPosition(); }
    public void stopWrist() { wrist.stopMotor(); }
    public void toggleWristBrake() { wrist.setIdleMode(wristBrakeToggle ? IdleMode.kBrake : IdleMode.kCoast); wristBrakeToggle = !wristBrakeToggle; }
    
    
    


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
        currentHeight = height;
    }

    public Height getHeight(){
        return currentHeight;
    }

    public void goToHeight() {
        if(currentHeight == Height.HIGH){
            setArmGoal(0.5); //Need to change
        }else if(currentHeight == Height.MID){
            setArmGoal(0.5); //Need to change
        }else if(currentHeight == Height.LOW){
            setArmGoal(0); //Need to change
        }else{
            setArmGoal(0.285);
        }
    }

    public double convertHeightToTics(){
        if(currentHeight == Height.HIGH){
            return highTics; //37.071

        }else if(currentHeight == Height.MID){
            return midTics; //27.118

        }else if(currentHeight == Height.LOW){
            return lowTics; //14.452

        }else{
            return groundTics; //0

        }
    }


    public void setPower(double power){
        // leftMotor.set(power);
        armMotor.set(power);
    }

}