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
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BitToStickyfaultString;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private double target = convertToRot(Constants.Arm.climbArmPosition);
    private double currentAbsPos;
    private boolean run = true;                                                                                                            

    /* Motors */
    private CANSparkFlex motorFront = new CANSparkFlex(Constants.armFrontId, MotorType.kBrushless);
    private CANSparkFlex motorBack = new CANSparkFlex(Constants.armBackId, MotorType.kBrushless);

    /* Encoders */
    private CANcoder encoderAbs = new CANcoder(Constants.armEncoderId); // fix this id

    /* Enums */
    @SuppressWarnings("unused")
    private Constants.RobotState state = Constants.RobotState.DEFAULT;

    /* PID Constants */
    private PIDConstants pidConstantsClimb = new PIDConstants(40, 10);
    private PIDConstants pidConstantsDefault = new PIDConstants(37.5, 8);
    private PIDConstants currentPIDConstants = pidConstantsDefault;

    Timer timer;

    double rampTime = 0.5;

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
        motorFront.setIdleMode(IdleMode.kCoast);
        motorBack.setIdleMode(IdleMode.kCoast);
        
        /* motor inversions */
        motorFront.setInverted(true); // b/c we flipped direction of motor
        motorBack.setInverted(true);
        
        motorFront.burnFlash();
        motorBack.burnFlash();
        
        SmartDashboard.putNumber("Change Arm Target", target);
        SmartDashboard.putNumber("Max Arm Voltage", currentPIDConstants.maxVoltage);

        timer = new Timer();

        timer.restart();
    }

    /** @return the absolute position of the arm in degrees */
    public double getAbsPos(){
        Logger.recordOutput("Arm/AbsCurrentPos", currentAbsPos);
        double angle = convertToDeg(currentAbsPos);
        Logger.recordOutput("Arm/AbsCurrentAngle", angle);
        // Logger.recordOutput("Arm/ArmCurrentPos(Converted)", convertToRot(angle));

        return angle;
              
    }

    public void logMotorFrontStickyFaults() {
        BitToStickyfaultString.getStickyFaultString(motorFront.getStickyFaults(), "Arm MotorFront");
        motorFront.clearFaults();
    }

    public void logMotorBackStickyFaults() {
        BitToStickyfaultString.getStickyFaultString(motorBack.getStickyFaults(), "Arm MotorBack");
        motorBack.clearFaults();
    }


    private void setVoltage(double voltage){
        Logger.recordOutput("Arm/ArmVoltage", voltage);
        
        motorFront.setVoltage(voltage);
        motorBack.setVoltage(voltage);
    }

    private double convertToRot(double angle) {
        return angle * Constants.Arm.RotateConversionFactor + Constants.Arm.AbsEncoderShift;

    }

    private double convertToDeg(double rot){
        return (1/Constants.Arm.RotateConversionFactor) * (rot - Constants.Arm.AbsEncoderShift);
    }

    /** converts to and sets the position target for the arm */
    public void setTarget(double targetDegrees) {
        double newTarget = Math.min(targetDegrees, Constants.Arm.armHardwareMax);
        newTarget = Math.max(newTarget, Constants.Arm.armHardwareMin);
        target = convertToRot(newTarget);

        timer.restart();
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
  
    public boolean finishedPID() {
        return Math.abs(target - currentAbsPos) < 0.03 ? true : false;
    }


    /**Default Arm PID */
    private void updatePID(double currentAbsPos) {
        double error = target - currentAbsPos;
        double pidValue = (error) * currentPIDConstants.kP;
                
        double maxVoltage = currentPIDConstants.maxVoltage;
        
        Logger.recordOutput("Arm/PIDValue", pidValue);
        Logger.recordOutput("Arm/maxVoltage", maxVoltage);
        Logger.recordOutput("Arm/armTarget", target);
        

        double outputVoltage = pidValue;
        
        if (Math.abs(pidValue) > maxVoltage) { //10 volts good for tele
            outputVoltage = maxVoltage * Math.signum(pidValue);
        }

        double currentTime = timer.get();
        double rampMultiplier = currentTime < rampTime ? currentTime / rampTime : 1;
        outputVoltage *= rampMultiplier;

        setVoltage(outputVoltage);
    }

    @Override
    public void periodic() {
        currentAbsPos = encoderAbs.getAbsolutePosition().getValue();

        //Logger.recordOutput("Arm/AbsCurrentPos", currentAbsPos);
        //getAbsPos();
        //SmartDashboard.putNumber("Arm Abs Pos", currentAbsPos);

        if (run) {
            updatePID(currentAbsPos);    
        } else {
            motorBack.stopMotor();
            motorFront.stopMotor();
        }

        //SmartDashboard.putNumber("Arm Setpoint", target);
    }

     /** changes the boolean for whether or not to run arm...if run == false, then stopMotor() is called */
     public void toggleArm(boolean run) {
        this.run = run;
    }

    
}
