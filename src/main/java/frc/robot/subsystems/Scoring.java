// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Scoring extends SubsystemBase {
    /* Instantiate Low Level Subsystems */
    private Extension ext; // Extension Subsystem
    private Arm arm; // Arm Subsystem
    private Wrist wrist; // Wrist Subsystem

    /* Initializing Motors */
    private CANSparkFlex intake = new CANSparkFlex(Constants.intakeId, MotorType.kBrushless);

    /* Ramping variables */
/*  // If you want to ramp intake
    private boolean rampDownBool;
    private double currentVoltage;
    private double voltageTo;
    private double rampTime;
    private Timer rampDownTimer;
*/
    /* Laser */
    private LaserCan laser;
    private LaserCan.Measurement measurement;
    MedianFilter medianDistance = new MedianFilter(3);


    /* Enums */
    private Constants.ScoringPos state = Constants.ScoringPos.CLIMB;
    private Constants.ScoringPos stateRobotWhenIntaking = Constants.ScoringPos.SOURCE;
    private Constants.IntakeState stateOfIntake = Constants.IntakeState.NO_NOTE;
    private Constants.ScoringPos lastState = Constants.ScoringPos.CLIMB;
    

    public Scoring() {
        /* Initialize Low Level Subsystems */
        ext = new Extension(); // Extension Subsystem
        arm = new Arm(); // Arm Subsystem
        wrist = new Wrist(); // Wrist Subsystem

        /* Frame Periods / Current Limit */
        intake.setSmartCurrentLimit(20);
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300); // For follower motors
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); // For Motor Position
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Analog Sensor Voltage + Velocity + position
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Duty cycler velocity + pos
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50); // Duty Cycle Absolute Encoder Position and Abs angle
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Duty Cycle Absolute Encoder Velocity + Frequency
                                                                      
        intake.burnFlash();

        intake.setIdleMode(IdleMode.kBrake);

        // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
        laser = new LaserCan(Constants.laserId);
        try {
            laser.setRangingMode(LaserCan.RangingMode.LONG);
            laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Laser configuration failed! " + e);
        }
        measurement = laser.getMeasurement();
        medianDistance = new MedianFilter(3);
    } 
    
    /** Has a median filter applied in this method */
    public double getLaserDistance() {
        try {
            measurement = laser.getMeasurement();
            if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                return medianDistance.calculate(measurement.distance_mm);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return 10000; // return a large number if the laser fails
    }

    public void setIntakeVoltage(double voltage) { intake.setVoltage(-voltage); SmartDashboard.putNumber("Intake Voltage", -voltage);}
    public double getIntakeTemp(){ return intake.getMotorTemperature(); }
    
/*  // If you want to ramp intake
    // linear ramp here https://www.desmos.com/calculator/ygpschqwqe
    public void rampDownVoltage(double currentVoltage, double voltageTo, double rampTime) {
        this.currentVoltage = currentVoltage;
        this.voltageTo = voltageTo;
        this.rampTime = rampTime;
        rampDownTimer.restart();
        rampDownBool = true;
    }
*/
    public void setState(Constants.ScoringPos state) {
        this.state = state;
    }

    public Constants.ScoringPos getState() {
        return this.state; // Returns the state of the robot
    }
    
    private void setStateRobotWhenIntaking(Constants.ScoringPos state) {
        this.stateRobotWhenIntaking = state;
    }

    public Constants.ScoringPos getStateRobotWhenIntaking() {
        return this.stateRobotWhenIntaking;
    }

    private void setIntakeState(Constants.IntakeState stateOfIntake) {
        this.stateOfIntake = stateOfIntake;
    }

    public Constants.IntakeState getIntakeState() {
        return this.stateOfIntake;
    }

    private Constants.ScoringPos getLastState() {
        return lastState;
    }




    /* Movement Logic */

    private void goToGround() {
        
    }

    private void goToStore() {
        
    }
    
    private void goToSource() {
        
    }

    private void goToAmp() {
        
    }

    private void goToClimb() {
        
    }

    private void goToTrap() {
        
    }

    public void goToPosition(Constants.ScoringPos state) {
        switch (state) {
            case GROUND: 
                goToGround();
                break;
            case STORE: 
                goToStore();
                break;
            case SOURCE: 
                goToSource();
                break;
            case AMP: 
                goToAmp();
                break;
            case CLIMB: 
                goToClimb();
                break;
            case TRAP: 
                goToTrap();
                break;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Updates the state for whether the intake took a note or not
        if (getLaserDistance() <= 10 /*mm*/ && getIntakeState() != Constants.IntakeState.HAS_NOTE) {
            setIntakeState(Constants.IntakeState.HAS_NOTE);
            setStateRobotWhenIntaking(getState());
        } else {
            setIntakeState(Constants.IntakeState.NO_NOTE);
            setStateRobotWhenIntaking(Constants.ScoringPos.SOURCE); // default state
        }




/*      // If you want to ramp intake
        if (rampDownBool) {
            if (rampDownTimer.get() < rampTime) {
              setIntakeVoltage(-(currentVoltage - voltageTo) * rampDownTimer.get() / rampTime + currentVoltage);
            } else {
              stopIntake();
              rampDownBool = false;
              rampDownTimer.stop();
            }
          }
*/            
    }

    
  
}
