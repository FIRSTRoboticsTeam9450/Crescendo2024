package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
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

    private double extensionTarget = 0.47;

    private CANSparkMax extensionMotor = new CANSparkMax(Constants.extensionId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private SparkAbsoluteEncoder extEncoder = extensionMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private boolean runStuff = false;

    //Technically starting pos
    private double hardLowerLimit = 0.89;
    private double hardUpperLimit = 0.145;

    private final PIDController extensionPid = new PIDController(0.37, 0,0);




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
        setExtensionGoal(0.47);


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
        double percentOutput = MathUtil.clamp(calculateExtensionPID(), -1.0, 1.0);
        SmartDashboard.putNumber("Extension Percent", percentOutput);

        double voltage = 12 * percentOutput;

        voltage = MathUtil.clamp(voltage /*+ ffValue*/, -6, 6);

        SmartDashboard.putNumber("Extension Voltage", voltage);

        
        setExtensionVoltage(voltage);
        
        
    }

    public double calculateExtensionFF() {
        //return (-1 * Math.Abs((1.44 * getArmPosition()) - 0.7632)) + 0.135;
        return 0;
    }



    @Override
    public void periodic(){
        if(runStuff){
            updateExtensionOutput();

        }else{
            setExtensionVoltage(0);
        }
        
        SmartDashboard.putNumber("Extension Position", getExtensionAbsPosition());
        SmartDashboard.putNumber("Extension Target", extensionTarget);

    }
    
}