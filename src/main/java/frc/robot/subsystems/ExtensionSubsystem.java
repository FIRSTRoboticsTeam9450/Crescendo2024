package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtensionSubsystem extends SubsystemBase{
    
    private Constants.Height lastHeight = Constants.Height.HOLD;

    private double extensionTarget = 0.72;

    private double extHardLowerLimit = 0.749; 
    private double extHardUpperLimit = 0.059;
    //Technically starting pos 14.5 INCHES DIFFERENCE  
    private double hardLowerLimit = 0.7; // all the way retracted 0.918
    private double hardUpperLimit = 0.1; // all the way extended 0.15
    // the actual hardUpperLimit is like negative if it could be
    
    private CANSparkMax extensionMotor = new CANSparkMax(Constants.extensionId, MotorType.kBrushless);
    private SparkAbsoluteEncoder extEncoder = extensionMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private final PIDController extensionPid = new PIDController(45, 0,0);

    boolean reachPos;
    private boolean runStuff;



    public ExtensionSubsystem(){
        extensionMotor.restoreFactoryDefaults();
        extensionMotor.setSmartCurrentLimit(40);
        extensionMotor.setIdleMode(IdleMode.kCoast);

        //If we use data port on extesnion, make sure to comment  the lines kstauts3-6
        extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
        extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); // For Motor Position
        // extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
        // extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
        // extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); //Duty Cycle Absolute Encoder Position and Abs angle
        // extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency
        extensionMotor.burnFlash();
        goToPosition(Constants.Height.SOURCE);

        SmartDashboard.putNumber("Change Extension Target", 0.47);
        SmartDashboard.putNumber("Change Extension Is Brake", 1);


    }
    
    public double getExtHardLowerLimit(){
        return extHardLowerLimit;
    }
    public double getExtHardUpperLimit(){
        return extHardUpperLimit;
    }
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






    @Override
    public void periodic(){

        reachPos = Math.abs(getExtensionAbsPosition() - this.extensionTarget) > 0.08;
        if(runStuff){
            updateExtensionOutput();

        }else{
            setExtensionVoltage(0);
        }
        SmartDashboard.putNumber("Extension Position", getExtensionAbsPosition());
        SmartDashboard.putNumber("Extension Target", extensionTarget);
        if (SmartDashboard.getNumber("Change Extension Is Brake", 1) == 1) {
            extensionMotor.setIdleMode(IdleMode.kBrake);
        } else {
            extensionMotor.setIdleMode(IdleMode.kCoast);
        }
        
    
    }

    public double convertToRads(double angle) {
        return angle/360*2*Math.PI;
    }
    public void toggleRun(){
        runStuff = !runStuff;
    }
    // --------------------------------------------------------------

    public void changeHeight(Constants.Height height){
        lastHeight = height;
    }

    public Constants.Height getHeight(){
        return lastHeight;
    }

    private boolean ground = false;
    private boolean source = true;
    public void goToPosition(Constants.Height pos) {
            if(ground && pos == Constants.Height.AMP){
                lastHeight = Constants.Height.AMP;
                // setArmWristExtGoal(0.531, 0.15, 0.25);
                setExtensionGoal(extHardLowerLimit + Constants.Extension.offsetToAmpFromGround); // wrist from smallest 0.117

            }else if(source && pos == Constants.Height.AMP){
                lastHeight = Constants.Height.AMP;
                // setArmWristExtGoal(0.511, 0.0487, 0.34); //extTarget = 0.387
                // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
                setExtensionGoal(extHardLowerLimit + Constants.Extension.offsetToAmpFromSource_Hold); //extTarget = 0.387

            }else if(pos == Constants.Height.HOLDTOGROUND){
                // lastHeight gets updated for this in the periodic method
                ground = true;
                source = false;
                lastHeight = Constants.Height.GROUND;
                // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
                // move arm to purpendicular (0.21), then move extension and wrist simultaneously while moving arm down
                // this boolean is a way to determine the "range" for when stuff starts moving after the arm, as well as some logic
                
                // the below will make arm go to 90 degree pos (logic in periodic method)
                setExtensionGoal(extHardLowerLimit + Constants.Extension.offsetToGround); //extTarget = 0.387

            }else if(pos == Constants.Height.GROUNDTOHOLD){
                // lastHeight gets updated for this in the periodic method
                ground = true;
                source = false;
                lastHeight = Constants.Height.HOLD;
                // setArmWristExtGoal(0.511, 0.0487, 0.47); //extTarget = 0.387
                // move arm to purpendicular (0.21) while moving extension and wrist simultaneously, and then move arm down
                // this boolean is a way to determine the "range" for when stuff starts moving after the arm, as well as some logic
                
                // the below will make arm go to 90 degree pos (logic in periodic method)
                setExtensionGoal(extHardLowerLimit + Constants.Extension.offsetToHold); //extTarget = 0.387

            }else if(pos == Constants.Height.GROUND){
                lastHeight = Constants.Height.GROUND;
                ground = true;
                source = false;
            
                // setArmWristExtGoal(0.1716, 0.51, 0.463); //extTarget = 0.387
                setExtensionGoal(extHardLowerLimit + Constants.Extension.offsetToGround); //extTarget = 0.387

            }else if(pos == Constants.Height.HOLD){
                lastHeight = Constants.Height.HOLD;
            
                // setArmWristExtGoal(0.13, 0.05, 0.73); 
                setExtensionGoal(extHardLowerLimit + Constants.Extension.offsetToHold); 

            }else if(pos == Constants.Height.SOURCE){
                lastHeight = Constants.Height.SOURCE;
                ground = false;
                source = true;
                // setArmWristExtGoal(0.39, 0.42, 0.55); //extTarget = 0.5346 wristTarget = 0.33
                // setArmWristExtGoal(0.37, 0.387, 0.55); //extTarget = 0.5346
                setExtensionGoal(extHardLowerLimit + Constants.Extension.offsetToSource); //extTarget = 0.5346

            }
            
            
        


    }
    
}