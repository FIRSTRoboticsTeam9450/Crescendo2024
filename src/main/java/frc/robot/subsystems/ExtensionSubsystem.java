package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtensionSubsystem extends SubsystemBase{

    private double extensionTarget = 1;

    private CANSparkMax extensionMotor = new CANSparkMax(Constants.extensionId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private boolean runStuff = true;


    private final PIDController extensionPid = new PIDController(0.37, 0,0);




    public ExtensionSubsystem(){
        extensionMotor.restoreFactoryDefaults();
        extensionMotor.setSmartCurrentLimit(40);

        extensionMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.getEncoder().setPosition(0);

        setExtensionGoal(1);


    }

    public void setExtensionVoltage(double voltage){
        extensionMotor.setVoltage(voltage);
    }

    public double getExtensionPosition(){  
        return extensionMotor.getEncoder().getPosition();
    }

    public void setExtensionGoal(double target){
        extensionTarget = target;
    }


    public double calculateExtensionPID(){
        return extensionPid.calculate(getExtensionPosition(), extensionTarget);
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
        //return (-1 * Math.abs((1.44 * getArmPosition()) - 0.7632)) + 0.135;
        return 0;
    }



    @Override
    public void periodic(){
        if(runStuff){
            updateExtensionOutput();

        }else{
            setExtensionVoltage(0);

        }

        SmartDashboard.putNumber("Extension Position", getExtensionPosition());
        SmartDashboard.putNumber("Extension Target", extensionTarget);

    }
    
}
