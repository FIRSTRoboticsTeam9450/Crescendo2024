// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
    /* Class Constants */
    private double target = convertToRot(Constants.Wrist.climbWristPosition);// 0.485;
    private boolean run = true;

    /* Motor */
    private CANSparkFlex motor = new CANSparkFlex(Constants.wristId, MotorType.kBrushless);

    /* Absolute Encoder */
    private SparkAbsoluteEncoder encoderAbs = motor.getAbsoluteEncoder(Type.kDutyCycle);

    /* Wrist FF */ // constants kinda tuned, final tuning still needed
    // private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.00001, 0.00003, 0.00001);

    /* PIDConstants */
    private PIDConstants pidConstantsDefault = new PIDConstants(40, 1);
    private PIDConstants currentPIDConstants = pidConstantsDefault;

    public Wrist() {
        motor.restoreFactoryDefaults();

        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300); // For follower motors

        motor.setIdleMode(IdleMode.kBrake);

        motor.burnFlash();
    }

    public double getAbsPos() {
        Logger.recordOutput("Wrist/CurrentPos", encoderAbs.getPosition());
        double angle = convertToDeg(encoderAbs.getPosition());
        Logger.recordOutput("Wrist/CurrentAngle", angle);
        // Logger.recordOutput("Wrist/CurrentPost(Converted)", convertToRot(angle));

        return angle;
    }

    // double oldVel = 0;
    // double oldTime = 0;
    // double oldPos = 0;

    public void updatePID() {
        Logger.recordOutput("Wrist/WristTarget", target);
        double error = target - convertToRot(getAbsPos());
        double pidValue = (error) * currentPIDConstants.kP; 
/*
        double changeInTime = Timer.getFPGATimestamp() - oldTime;
        double velSetpoint = (getAbsPos() - oldPos) / changeInTime;
        double accel = (velSetpoint - oldVel) / (changeInTime);
        double ffVal = wristFeedForward.calculate(velSetpoint, accel); // takes velocity, and acceleration
*/

        double maxVoltage = currentPIDConstants.maxVoltage;

        Logger.recordOutput("Wrist/maxVoltage", maxVoltage);

        // limits voltage
        if (Math.abs(pidValue) < maxVoltage) {
            setVoltage(-pidValue);
        } else {
            setVoltage(-maxVoltage * Math.signum(pidValue));
        }

        // update vars for determining acceleration later
/*      
        oldVel = velSetpoint;
        oldTime = Timer.getFPGATimestamp();
        oldPos = getAbsPos();
*/

    }

    public boolean finishedPID() {
        return Math.abs(target - encoderAbs.getPosition()) < 0.025 ? true : false;
    }

    private void setVoltage(double voltage) {
        Logger.recordOutput("Wrist/WristVoltage", voltage);

        motor.setVoltage(voltage);

    }

    private double convertToRot(double angle) {
        return angle * Constants.Wrist.RotateConversionFactor - Constants.Wrist.AbsEncoderShift;

    }

    // 0.5648 straight out, 0.8497 straight down
    private double convertToDeg(double rot){
        return (1/Constants.Wrist.RotateConversionFactor) * (rot + Constants.Wrist.AbsEncoderShift);
    }

    public void setTarget(double targetDegrees) {
        double newTarget = Math.min(targetDegrees, Constants.Wrist.wristHardwareMax);
        newTarget = Math.max(newTarget, Constants.Wrist.wristHardwareMin);
        target = convertToRot(newTarget);
    }

    public double getTarget(){
        return convertToDeg(target);
    }
    

    @Override
    public void periodic() {
        if (run) {
            updatePID();
        } else {
            motor.stopMotor();
        }

        
    }

    /** changes the boolean for whether or not to run wrist...if run == false, then stopMotor() is called */
    public void toggleWrist(boolean run) {
        this.run = run;
    }
}
