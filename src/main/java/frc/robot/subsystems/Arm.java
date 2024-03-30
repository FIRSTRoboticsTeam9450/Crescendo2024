// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.math.BigDecimal;
import java.math.RoundingMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private double target = Constants.MovementLimits.armHardLowerLimit + Constants.Arm.offsetToPreClimb;
    double currentRelPos;
    double currentAbsPos;
                                                                                                            


    /* Motors */
    private CANSparkMax motorFront = new CANSparkMax(Constants.armFrontId, MotorType.kBrushless);
    private CANSparkMax motorBack = new CANSparkMax(Constants.armBackId, MotorType.kBrushless);

    /* Encoders */
    SparkAbsoluteEncoder encoderAbs = motorFront.getAbsoluteEncoder(Type.kDutyCycle);
    RelativeEncoder encoderRel;    

    /* Enums */
    @SuppressWarnings("unused")
    private Constants.RobotState state = Constants.RobotState.DEFAULT;

    /* PID Constants */
    private PIDConstants pidConstantsClimb = new PIDConstants(30, 12);
    private PIDConstants pidConstantsDefault = new PIDConstants(55, 6);
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
        
        /* encoders */
        encoderRel = motorBack.getEncoder();
        encoderRel.setPositionConversionFactor(1.0 / 103.85652383245);
        
        /* motor inversions */
        motorFront.setInverted(true); // b/c we flipped direction of motor
        motorBack.setInverted(true);
        
        motorFront.burnFlash();
        motorBack.burnFlash();
        
        updateRelPos();

        SmartDashboard.putNumber("Change Arm Target", target);
        SmartDashboard.putNumber("Max Arm Voltage", currentPIDConstants.maxVoltage);
    }


    public void updateRelPos() {
        setRelPos(getAbsPos());
    }

    public void setRelPos(double position) {
        encoderRel.setPosition(position);
    }
    
    /**
    * @return the relative position of the arm
    */
    public double getRelPos() {
        
        return currentRelPos;
       
    }

    private double getAbsPos(){
        
        
        return currentAbsPos;
        
    }

    public void setVoltage(double voltage){
        Logger.recordOutput("Arm/ArmVoltage", voltage);
        
        motorFront.setVoltage(voltage);
        motorBack.setVoltage(voltage);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getTarget(){
        return target;
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
    private void updatePID(double currentAbsPos, double currentRelPos) {
        double error = target - currentRelPos;
        double pidValue = (error) * currentPIDConstants.kP;
        
        // the complicated BigDecimal is to limit to 3 decimal places
        BigDecimal limDecimalPlaces = new BigDecimal(target).setScale(5, RoundingMode.HALF_UP);
        
        double maxVoltage = currentPIDConstants.maxVoltage;
        
        Logger.recordOutput("Arm/PIDValue", pidValue);
        Logger.recordOutput("Arm/maxVoltage", maxVoltage);
        Logger.recordOutput("Arm/armTarget", target);

        if (Math.abs(currentRelPos) < 0.1) { // because arm should never be lower than 0.1 pos
            setVoltage(0);
        } else {
            if (Math.abs(pidValue) < maxVoltage) { //10 volts good for tele
                setVoltage(pidValue);                
            } else {
                setVoltage(maxVoltage * Math.signum(pidValue));
            }
        }

        // basically if the difference between the abs and rel pos is enough, we know chain probably slipped
        // since the rel encoder is off from the abs differently from climb and store positions, we only look at climb
        // so if we are goign to the climb pos, and the error is within an amount, and the abs and rel are sufficiently off,
        // then updateRelArmPos
        
        if (limDecimalPlaces.doubleValue() == (Constants.MovementLimits.armHardLowerLimit + Constants.Arm.offsetToPreClimb) && Math.abs(currentAbsPos - currentRelPos) > 0.01619 && Math.abs(error) < 0.01) {
            updateRelPos();
        } else if (limDecimalPlaces.doubleValue() == (Constants.MovementLimits.armHardLowerLimit + Constants.Arm.offsetToPreClimb) && Math.abs(error) < 0.006 && Math.abs(encoderRel.getVelocity()) < 10) { // update every time going to climb pos
            // System.out.println("UPDATED ARM");
            updateRelPos();
        }
        // System.out.println("Arm Abs Target: " + limDecimalPlaces.doubleValue() + " Math.abs(error): " + Math.abs(error) + " Velocity: " + Math.abs(armRelEncoder.getVelocity()));
    }

    @Override
    public void periodic() {
        

        currentRelPos = encoderRel.getPosition();
        currentAbsPos = encoderAbs.getPosition();

        Logger.recordOutput("Arm/AbsCurrentPos", currentAbsPos);
        SmartDashboard.putNumber("Arm Abs Pos", currentAbsPos);
        Logger.recordOutput("Arm/RelCurrentPos", currentRelPos);

    
        updatePID(currentAbsPos, currentRelPos);



        SmartDashboard.putNumber("Arm Rel Position", getRelPos());
        SmartDashboard.putNumber("Ratio Abs/Rel arm", getAbsPos() / getRelPos());
        SmartDashboard.putNumber("Arm Setpoint", target);
    }

    
}