package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase{
    public enum Height{
        LOW,
        MID,
        HIGH,
        GROUND,
    }
    private Height currentHeight = Height.GROUND;

    private double armTarget = 0.53;
    private double extensionTarget = 1;

    //double p = 0;

    //public CANSparkMax intake = new CANSparkMax(Constants.intakeId, MotorType.kBrushless);

    // private CANSparkMax leftMotor = new CANSparkMax(Constants.rotationLeftId,MotorType.kBrushless);
    private CANSparkMax armMotor = new CANSparkMax(Constants.armId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    
    
    private boolean runStuff = true;

    private CANSparkMax extensionMotor = new CANSparkMax(Constants.extensionId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    
    
    private final ProfiledPIDController rotation = new ProfiledPIDController(6.1, 0, 0, new Constraints(1, 0.5));//maxVel = 3.5 and maxAccel = 2.5
    private final ArmFeedforward rotationFF = new ArmFeedforward(0, 0.027, 0.00001); //0.027, 0.00001
    private final ArmFeedforward rotationExtendedFF = new ArmFeedforward(0, 0.03, 0.00001);

    //Extension
    private final PIDController extension = new PIDController(0.37, 0,0);

    private PIDController pid = new PIDController(0.1, 0, 0), downPID = new PIDController(0.0085, 0, 0);
    
    Timer timer = new Timer();
    private double ticsPerArmRevolution = 144, ticsPerWristRevolution = /*172.8*/ 120, lowTics = (50/360) * ticsPerArmRevolution, midTics = (100/360) * ticsPerArmRevolution, highTics = (135/360) * ticsPerArmRevolution, groundTics = (37.4/360) * ticsPerArmRevolution;
    private boolean intialization = true;

    DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);


    public ArmSubsystem(){
        //leftMotor.setSmartCurrentLimit(40);
        armMotor.setSmartCurrentLimit(40);
        armMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setIdleMode(IdleMode.kBrake);

        extensionMotor.getEncoder().setPosition(0);

        // leftMotor.restoreFactoryDefaults();
       


        // leftMotor.setIdleMode(IdleMode.kCoast);
        
        
        //leftMotor.setInverted(true);  
        //rightMotor.setInverted(false);  

        //extensionMotor.setIdleMode(IdleMode.kBrake);
        
        //Might need this line
        //intake.setIdleMode(IdleMode.kBrake);
        
        //leftMotor.getEncoder().setVelocityConversionFactor();
        // 144 revolutions of motor to 1 rev of arm
        // 360 / 144 = 2.5 degrees / arm revolution
        // 360 / 120 = 3 degrees / wrist revolution
        
        //leftMotor.setInverted(false);
        // leftMotor.getEncoder().setPosition(0);
        


        //timer.start();
        
        
        //setWristGoal(0);
        // if (intialization) {
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
        //rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);        //Duty Cycle Absolute Encoder Position and Abs angle
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency

        
        
        setArmGoal(0.53);

    }

    public double getPosition(){
        return absEncoder.get();
    }

    

    public void initialize(){
        armMotor.restoreFactoryDefaults();
        extensionMotor.restoreFactoryDefaults();


        extensionMotor.setIdleMode(IdleMode.kCoast);
        extensionMotor.getEncoder().setPosition(0);
        

        
        armMotor.setIdleMode(IdleMode.kBrake);

        //wristMotor.getEncoder().setPosition(0);

        //wrist.reset(getWristAngle());
        rotation.reset(getPosition());
        //setWristGoal(0);

        //initialSetWristEncoder();
           
        setArmGoal(0.285);
        setExtensionGoal(1);

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

    public void setExtensionVoltage(double voltage){
        extensionMotor.setVoltage(voltage);
    }

    public void downManual(){
        armTarget -= 0.01;

        //wrist.setGoal(2.57 - armTarget);
    }

    public void upManual(){
        armTarget += 0.01;

        //wrist.setGoal(2.57 - armTarget);
    }

    public double getExtensionPosition(){
        
        return extensionMotor.getEncoder().getPosition();
          
    }

    public void setArmGoal(double target) {
        //rotation.setGoal(target);
        armTarget = target;
    }

    public void setExtensionGoal(double target){
        extensionTarget = target;
    }

    public double getGoal(){
        return rotation.getGoal().position;
    }

    public double calculateRotationPID(){
        return rotation.calculate(getPosition(), armTarget);
    }
    
    public double calculateExtensionPID(){
        return extension.calculate(getExtensionPosition(), extensionTarget);
    }

    public double calculateExtensionFF() {
        return (-1 * Math.abs((1.44 * getPosition()) - 0.7632)) + 0.135;
    }

    public void updateExtensionOutput(){
        double ffValue = calculateExtensionFF();
        SmartDashboard.putNumber("Extension FF", ffValue);
        double percentOutput = MathUtil.clamp(calculateExtensionPID(), -1.0, 1.0);
        SmartDashboard.putNumber("Extension Percent", percentOutput);

        double voltage = convertToVolts(percentOutput);

        voltage = MathUtil.clamp(voltage /*+ ffValue*/, -6, 6);

        SmartDashboard.putNumber("Extension Voltage", voltage);

        
        setExtensionVoltage(voltage);
        
        
    }



    

    public void updateRotationOutput(){
        double ffValue = calculateRotationFF();
        double percentOutput = MathUtil.clamp(calculateRotationPID() + ffValue, -1.0, 1.0);
        double voltage = convertToVolts(percentOutput);

        SmartDashboard.putNumber("Rotation FF", ffValue);
        SmartDashboard.putNumber("Rotation Voltage", voltage);
        
        
        setArmVoltage(voltage);
        
        
    }
    



    

    public double calculateRotationFF(){
        if(extensionTarget == 30){
            return rotationExtendedFF.calculate(getPosition(), rotation.getSetpoint().velocity);
        }

        return rotationFF.calculate(getPosition(), rotation.getSetpoint().velocity);

        
    }
    

    private double convertToVolts(double percentOutput){
        return percentOutput * Robot.getInstance().getVoltage();
    }

    @Override
    public void periodic(){
        
        if(runStuff){
            updateRotationOutput();
            //updateWristOutput();
            updateExtensionOutput();
            //setArmVoltage(0);

        }else{
            setArmVoltage(0);
           
        }
        // SmartDashboard.putNumber("LeftPosition", getLeftPosition());
        // SmartDashboard.putNumber("Right Arm Position", getPosition());
        // SmartDashboard.putNumber("Target", armTarget);
        SmartDashboard.putNumber("Extension Position", getExtensionPosition());
        SmartDashboard.putNumber("Extension Target", extensionTarget);


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