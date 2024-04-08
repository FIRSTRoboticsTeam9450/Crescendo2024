// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.ConstantDescs;
import java.math.BigDecimal;
import java.math.RoundingMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private double target = convertToRot(180);
    private double currentAbsPos;
    private boolean run = true;                                                                                                            


    /* Motors */
    private CANSparkMax motorFront = new CANSparkMax(Constants.armFrontId, MotorType.kBrushless);
    private CANSparkMax motorBack = new CANSparkMax(Constants.armBackId, MotorType.kBrushless);

    /* Encoders */
    private CANcoder encoderAbs = new CANcoder(Constants.armEncoderId); // fix this id

    /* Enums */
    @SuppressWarnings("unused")
    private Constants.RobotState state = Constants.RobotState.DEFAULT;

    /* PID Constants */
    private PIDConstants pidConstantsClimb = new PIDConstants(30, 4);
    private PIDConstants pidConstantsDefault = new PIDConstants(55, 4);
    private PIDConstants currentPIDConstants = pidConstantsDefault;

    public Arm() {

        motorFront.restoreFactoryDefaults();
        motorBack.restoreFactoryDefaults();
        
        /*smart current limits */
        motorFront.setSmartCurrentLimit(40);
        motorBack.setSmartCurrentLimit(40);
        
        /* periodic framerates */
        motorFront.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300); // For follower motors
        motorBack.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);

        /* motor idlemodes */
        motorFront.setIdleMode(IdleMode.kBrake);
        motorBack.setIdleMode(IdleMode.kBrake);
        
        /* motor inversions */
        motorFront.setInverted(true); // b/c we flipped direction of motor
        motorBack.setInverted(true);
        
        motorFront.burnFlash();
        motorBack.burnFlash();
        
        SmartDashboard.putNumber("Change Arm Target", target);
        SmartDashboard.putNumber("Max Arm Voltage", currentPIDConstants.maxVoltage);
    }

    /** @return the absolute position of the arm in degrees */
    public double getAbsPos(){
        Logger.recordOutput("Arm/AbsCurrentPos", encoderAbs.getAbsolutePosition().getValue());
        double angle = convertToDeg(currentAbsPos);
        Logger.recordOutput("Arm/AbsCurrentAngle", angle);
        // Logger.recordOutput("Arm/ArmCurrentPos(Converted)", convertToRot(angle));

        return angle;
              
    }

    public void setVoltage(double voltage){
        Logger.recordOutput("Arm/ArmVoltage", voltage);
        
        motorFront.setVoltage(voltage);
        motorBack.setVoltage(voltage);
    }

    private double convertToRot(double angle) {
        return angle * Constants.NewArm.RotateConversionFactor + Constants.NewArm.AbsEncoderShift;

    }

    private double convertToDeg(double rot){
        return (1/Constants.NewArm.RotateConversionFactor) * (rot - Constants.NewArm.AbsEncoderShift);
    }

    /** converts to and sets the position target for the arm */
    public void setTarget(double targetInches) {
        this.target = convertToRot(targetInches);
    }

    /** @return the target of the arm in degrees */
    public double getTarget(){
        return convertToDeg(target);
    }

    public void setState(Constants.RobotState state) {
        this.state = state;
        if (state.equals(Constants.RobotState.CLIMBING)) {
            currentPIDConstants = pidConstantsClimb;
        } else {
            currentPIDConstants = pidConstantsDefault;
        }
    }
  
    


    /**Default Arm PID */
    private void updatePID(double currentAbsPos) {
        double error = target - currentAbsPos;
        double pidValue = (error) * currentPIDConstants.kP;
        
        // the complicated BigDecimal is to limit to 3 decimal places
        BigDecimal limDecimalPlaces = new BigDecimal(target).setScale(5, RoundingMode.HALF_UP);
        
        double maxVoltage = currentPIDConstants.maxVoltage;
        
        Logger.recordOutput("Arm/PIDValue", pidValue);
        Logger.recordOutput("Arm/maxVoltage", maxVoltage);
        Logger.recordOutput("Arm/armTarget", target);

        if (Math.abs(currentAbsPos) < 0.1) { // because arm should never be lower than 0.1 pos
            setVoltage(0);
        } else {
            if (Math.abs(pidValue) < maxVoltage) { //10 volts good for tele
                setVoltage(pidValue);                
            } else {
                setVoltage(maxVoltage * Math.signum(pidValue));
            }
        }
    }

    @Override
    public void periodic() {
        

        currentAbsPos = encoderAbs.getAbsolutePosition().getValue();

        //Logger.recordOutput("Arm/AbsCurrentPos", currentAbsPos);
        getAbsPos();
        SmartDashboard.putNumber("Arm Abs Pos", currentAbsPos);

        if (run) {
            updatePID(currentAbsPos);    
        } else {
            motorBack.stopMotor();
            motorFront.stopMotor();
        }

        SmartDashboard.putNumber("Arm Setpoint", target);
    }

     /** changes the boolean for whether or not to run arm...if run == false, then stopMotor() is called */
     public void toggleArm(boolean run) {
        this.run = run;
    }

    
}
