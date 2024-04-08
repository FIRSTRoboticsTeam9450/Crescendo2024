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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Scoring extends SubsystemBase {
    /* Instantiate Low Level Subsystems */
    public Extension ext; // Extension Subsystem
    public Arm arm; // Arm Subsystem
    public Wrist wrist; // Wrist Subsystem

    /* Initializing Motors */
    private CANSparkFlex intake = new CANSparkFlex(Constants.intakeId, MotorType.kBrushless);


    /* Constants  */
    private static final int ANGLEINDEX = 0; 
    private static final int EXTENSIONMININDEX = 1;
    private static final int WRISTMAXINDEX = 2;
    private static final int WRISTMININDEX = 3;
    private static final int EXTENSIONMAXINDEX = 4;
    private static final int WRISTMAXINDEX2 = 5;
    private static final int WRISTMININDEX2 = 6;


    /* Ramping variables */
  // If you want to ramp intake
    private boolean rampDownBool;
    private double currentVoltage;
    private double voltageTo;
    private double rampTime;
    private Timer rampDownTimer;

    /* Laser */
    private LaserCan laser;
    private LaserCan.Measurement measurement;
    MedianFilter medianDistance = new MedianFilter(3);


    /* Enums */
    private Constants.ScoringPos state = Constants.ScoringPos.CLIMB;
    private Constants.ScoringPos stateRobotWhenIntaking = Constants.ScoringPos.SOURCE;
    private Constants.IntakeState stateOfIntake = Constants.IntakeState.NO_NOTE;
    private Constants.ScoringPos lastState = Constants.ScoringPos.CLIMB;
    
    /* Limits */
    private double desiredArmAngle, desiredWristAngle, desiredExtensionLength;


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
        setState(Constants.ScoringPos.NONE);

        rampDownTimer = new Timer();
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
    
  // If you want to ramp intake
    // linear ramp here https://www.desmos.com/calculator/ygpschqwqe
    public void rampDownIntakeVoltage(double currentVoltage, double voltageTo, double rampTime) {
        this.currentVoltage = currentVoltage;
        this.voltageTo = voltageTo;
        this.rampTime = rampTime;
        rampDownTimer.restart();
        rampDownBool = true;
    }

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

    public Extension getExtSub(){
        return ext;
    }
    public Arm getArmSub(){
        return arm;
    }
    public Wrist getWristSub(){
        return wrist;
    }

    
    /* Movement Logic */

    private void goToGround() {
        desiredArmAngle = Constants.Arm.groundArmPosition;
        desiredWristAngle = Constants.Wrist.groundWristPosition;
        desiredExtensionLength = Constants.Extension.groundExtPosition;
    }   

    private void goToStore() {
        desiredArmAngle = Constants.Arm.storeArmPosition;
        desiredWristAngle = Constants.Wrist.storeWristPosition;
        desiredExtensionLength = Constants.Extension.storeExtPosition;
    }
    
    private void goToSource() {
        desiredArmAngle = Constants.Arm.sourceArmPosition;
        desiredWristAngle = Constants.Wrist.sourceWristPosition;
        desiredExtensionLength = Constants.Extension.sourceExtPosition;
    }

    private void goToAmp() {
        desiredArmAngle = Constants.Arm.ampArmPosition;
        desiredWristAngle = Constants.Wrist.ampWristPosition;
        desiredExtensionLength = Constants.Extension.ampExtPosition;
    }

    private void goToClimb() {
        desiredArmAngle = Constants.Arm.climbArmPosition;
        desiredWristAngle = Constants.Wrist.climbWristPosition;
        desiredExtensionLength = Constants.Extension.climbExtPosition;
    }

    private void goToTrap() {
        desiredArmAngle = Constants.Arm.trapArmPosition;
        desiredWristAngle = Constants.Wrist.trapWristPosition;
        desiredExtensionLength = Constants.Extension.trapExtPosition;
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
            case NONE:
                // do nothing
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

        // Sets the desired targets for each child subsystem
        ext.setTarget(desiredExtensionLength);
        arm.setTarget(desiredArmAngle);
        wrist.setTarget(desiredWristAngle);
       


      // If you want to ramp intake
        if (rampDownBool) {
            if (rampDownTimer.get() < rampTime) {
              setIntakeVoltage(-(currentVoltage - voltageTo) * rampDownTimer.get() / rampTime + currentVoltage);
            } else {
              intake.stopMotor();
              rampDownBool = false;
              rampDownTimer.stop();
            }
          }
            
    }

    /* Limiter Logic */
    // add limit code 

    //Hmmm maybe not yet(want to find shortest / fastest path kinda like a pathplanner thing?)
    public void smartLimiter(){

    }
    public void limit(){
        double armAngle = arm.getAbsPos(); // get the current arm angle
        double armTarget = arm.getTarget();
        double extLength = ext.getRelPos(); // get the current extension length
        double extTarget = -ext.getTarget();
        double wristAngle = wrist.getAbsPos(); // get the current wrist angle
        double wristTarget = wrist.getTarget();

        double currentArmMax = Math.abs(limits[limits.length - 1][ANGLEINDEX]);
        double currentArmMin = Math.abs(limits[0][ANGLEINDEX]);
        // if arm angle is between max and min arm angles
        if(Math.abs(armAngle) > currentArmMin && Math.abs(armAngle) < currentArmMax ){ 

            arm.toggleArm(true); // resume arm
            double currentExtMax = limits[(int) Math.round(armAngle) - 45][EXTENSIONMAXINDEX];
            double currentExtMin = limits[(int) Math.round(armAngle) - 45][EXTENSIONMININDEX];
            // if the ext amt is within error amt for max ext
            // optional version that accounts for final index if you want to look 1 angle ahead: limits[(int) Math.round(armAngle) - 45 + 1 > 235 ? 235 : armAngle - 45 + 1][4]
            // This part of the if statements basically checks if the extension is less than the max it can be at the current angle
            // the inside part is the max and min code for the wrist limits
            if (currentExtMax - extLength < 0.1) { // arm angle increments by 1 but starts by 45, so take armAngle - 45 to get proper index
                
                // extension is at the limit, and target isn't in opposite direction
                // so stop ext
                if (extTarget > currentExtMax - 0.1) { 
                    ext.toggleExt(false); // stop ext
                    System.out.println("EXTENSION STOPPED (MAX)");
                } else {
                    ext.toggleExt(true);
                }
               
                
                
            //This part of the if statement checks if the extension is greather than the mininum(or at the min ofc) it can be at its current angle
            // This inside is the max and min angle of the wrist.
            } else if (extLength - currentExtMin < 0.1) { // if the ext amt is within error amt for min ext
                
                // extension is at the limit, and target isn't in opposite direction
                // so stop ext
                if (extTarget < currentExtMin + 0.1) { 
                    ext.toggleExt(false); // stop ext
                    System.out.println("EXTENSION STOPPED (MIN)");
                } else {
                    ext.toggleExt(true);
                }
                


            } else {
                ext.toggleExt(true); // resume ext
            }

            double currentWristMax_ExtMax = limits[(int) Math.round(armAngle) - 45][WRISTMAXINDEX2];
            double currentWristMin_ExtMax = limits[(int) Math.round(armAngle) - 45][WRISTMININDEX2];
            
            // wrist is at the limit, and target isn't in opposite direction
            // so stop wrist
            if (currentWristMin_ExtMax + 2 /* 2 degree tolerance */ > wristAngle && wristTarget < currentWristMin_ExtMax + 2) {
                wrist.toggleWrist(false); // stop wrist
                System.out.println("WRIST STOPPED");
            } else if (wristAngle > currentWristMax_ExtMax - 2 && wristTarget > currentWristMax_ExtMax - 2) {
                wrist.toggleWrist(false); // stop wrist
                System.out.println("WRIST STOPPED");
            } else {
                wrist.toggleWrist(true); // resume wrist

            }

            double currentWristMax_ExtMin = limits[(int) Math.round(armAngle) - 45][WRISTMAXINDEX];
            double currentWristMin_ExtMin = limits[(int) Math.round(armAngle) - 45][WRISTMININDEX];
            
            // wrist is at the limit, and target isn't in opposite direction
            // so stop wrist
            if (currentWristMin_ExtMin + 2 /* 2 degree tolerance */ > wristAngle && wristTarget < currentWristMin_ExtMin + 2) {
                wrist.toggleWrist(false); // stop wrist
                System.out.println("WRIST STOPPED");
            } else if (wristAngle > currentWristMax_ExtMin - 2 && wristTarget > currentWristMax_ExtMin - 2) {
                wrist.toggleWrist(false); // stop wrist
                System.out.println("WRIST STOPPED");
            } else {
                wrist.toggleWrist(true); // resume wrist

            }

        }else{
            if (armTarget < currentArmMin || armTarget > currentArmMax) {
                arm.toggleArm(false); // stop arm
                System.out.println("SOMETHING VERY WRONG WITH ARM");
            } else {
                arm.toggleArm(true);
            }
            
        }
    }
    
    

    private double[][] limits = new double[][]{
        //Arm Angle		Extension Min		Wrist Max		Wrist min		Extension max		Wrist Max		Wrist Min
        {51,0,270,250,0,270,250},
        {52,0,270,245.7,0.093,270,245.7},
        {53,0,270,241.4,0.186,270,241.4},
        {54,0,270,237.1,0.279,270,237.1},
        {55,0,270,232.8,0.372,270,232.8},
        {56,0,270,228.5,0.465,270,228.5},
        {57,0,270,224.2,0.558,270,224.2},
        {58,0,270,219.9,0.651,270,219.9},
        {59,0,270,215.6,0.744,270,215.6},
        {60,0,270,211.3,0.837,270,211.3},
        {61,0,270,207,0.93,270,207},
        {62,0,270,202.7,1.023,270,202.7},
        {63,0,270,198.4,1.116,270,198.4},
        {64,0,270,194.1,1.209,270,194.1},
        {65,0,270,189.8,1.302,270,189.8},
        {66,0,270,185.5,1.395,270,185.5},
        {67,0,270,181.2,1.488,270,181.2},
        {68,0,270,176.9,1.581,270,176.9},
        {69,0,270,172.6,1.674,270,172.6},
        {70,0,270,168.3,1.767,270,168.3},
        {71,0,270,164,1.86,270,164},
        {72,0,270,159.7,1.953,270,159.7},
        {73,0,270,155.4,2.046,270,155.4},
        {74,0,270,151.1,2.139,270,151.1},
        {75,0,270,146.8,2.232,270,146.8},
        {76,0,270,142.5,2.325,270,142.5},
        {77,0,270,138.2,2.418,270,138.2},
        {78,0,270,133.9,2.511,270,133.9},
        {79,0,270,129.6,2.604,270,129.6},
        {80,0,270,125.3,2.697,270,125.3},
        {81,0,270,121,2.79,270,121},
        {82,0,270,116.7,2.883,270,116.7},
        {83,0,270,112.4,2.976,270,112.4},
        {84,0,270,108.1,3.069,270,108.1},
        {85,0,270,103.8,3.162,270,103.8},
        {86,0,270,99.5,3.255,270,99.5},
        {87,0,270,95.2,3.348,270,95.2},
        {88,0,270,90.9,3.441,270,90.9},
        {89,0,270,90,3.5,270,90},
        {90,0,270,90,3.5,270,90},
        {91,0,270,90,3.5,270,90},
        {92,0,270,90,3.5,270,90},
        {93,0,270,90,3.5,270,90},
        {94,0,270,90,3.5,270,90},
        {95,0,270,90,3.54,270,90},
        {96,0,270,90,3.58,270,90},
        {97,0,270,90,3.62,270,90},
        {98,0,270,90,3.66,270,90},
        {99,0,270,90,3.7,270,90},
        {100,0,270,90,3.74,270,90},
        {101,0,270,90,3.78,270,90},
        {102,0,270,90,3.82,270,90},
        {103,0,270,90,3.86,270,90},
        {104,0,270,90,3.9,270,90},
        {105,0,270,90,3.94,270,90},
        {106,0,270,90,3.98,270,90},
        {107,0,270,90,4.02,270,90},
        {108,0,270,90,4.06,270,90},
        {109,0,270,90,4.1,270,90},
        {110,0,270,90,4.14,270,90},
        {111,0,270,90,4.18,270,90},
        {112,0,270,90,4.22,270,90},
        {113,0,270,90,4.26,270,90},
        {114,0,270,90,4.3,270,90},
        {115,0,270,90,4.34,270,90},
        {116,0,270,90,4.38,270,90},
        {117,0,270,90,4.42,270,90},
        {118,0,270,90,4.46,270,90},
        {119,0,270,90,4.5,270,90},
        {120,0,270,90,4.54,270,90},
        {121,0,270,90,4.58,270,90},
        {122,0,270,90,4.62,270,90},
        {123,0,270,90,4.66,270,90},
        {124,0,270,90,4.7,270,90},
        {125,0,270,90,4.74,270,90},
        {126,0,270,90,4.78,270,90},
        {127,0,270,90,4.82,270,90},
        {128,0,270,90,4.86,270,90},
        {129,0,270,90,4.9,270,90},
        {130,0,270,90,4.94,270,90},
        {131,0,270,90,4.98,270,90},
        {132,0,270,90,5.02,270,90},
        {133,0,270,90,5.06,270,90},
        {134,0,270,90,5.1,270,90},
        {135,0,270,90,5.14,270,90},
        {136,0,270,90,5.18,270,90},
        {137,0,270,90,5.22,270,90},
        {138,0,270,90,5.26,270,90},
        {139,0,270,90,5.3,270,90},
        {140,0,270,90,5.34,270,90},
        {141,0,270,90,5.38,270,90},
        {142,0,270,90,5.42,270,90},
        {143,0,270,90,5.46,270,90},
        {144,0,270,90,5.5,270,90},
        {145,0,270,90,5.54,270,90},
        {146,0,270,90,5.58,270,90},
        {147,0,270,90,5.62,270,90},
        {148,0,270,90,5.66,270,90},
        {149,0,270,90,5.7,270,90},
        {150,0,270,90,5.74,270,90},
        {151,0,270,90,5.78,270,90},
        {152,0,270,90,5.82,270,90},
        {153,0,270,90,5.86,270,90},
        {154,0,270,90,5.9,270,90},
        {155,0,270,90,5.94,270,90},
        {156,0,270,90,5.98,270,90},
        {157,0,270,90,6.02,270,90},
        {158,0,270,90,6.06,270,90},
        {159,0,270,90,6.1,270,90},
        {160,0,270,90,6.14,270,90},
        {161,0,270,90,6.18,270,90},
        {162,0,270,90,6.22,270,90},
        {163,0,270,90,6.26,270,90},
        {164,0,270,90,6.3,270,90},
        {165,0,270,90,6.34,270,90},
        {166,0,270,90,6.38,270,90},
        {167,0,270,90,6.42,270,90},
        {168,0,270,90,6.46,270,90},
        {169,0,270,90,6.5,270,90},
        {170,0,270,90,6.54,270,90},
        {171,0,270,90,6.58,270,90},
        {172,0,270,90,6.62,270,90},
        {173,0,270,90,6.66,270,90},
        {174,0,270,90,6.7,270,90},
        {175,0,270,90,6.74,270,90},
        {176,0,270,90,6.78,270,90},
        {177,0,270,90,6.82,270,90},
        {178,0,270,90,6.86,270,90},
        {179,0,270,90,6.9,270,90},
        {180,0,270,90,7,270,90},
        {181,0,270,90,6.96,270,90},
        {182,0,270,90,6.92,270,90},
        {183,0,270,90,6.88,270,90},
        {184,0,270,90,6.84,270,90},
        {185,0,270,90,6.8,270,90},
        {186,0,270,90,6.76,270,90},
        {187,0,270,90,6.72,270,90},
        {188,0,270,90,6.68,270,90},
        {189,0,270,90,6.64,270,90},
        {190,0,270,90,6.6,270,90},
        {191,0,270,90,6.56,270,90},
        {192,0,270,90,6.52,270,90},
        {193,0,270,90,6.48,270,90},
        {194,0,270,90,6.44,270,90},
        {195,0,270,90,6.4,270,90},
        {196,0,270,90,6.36,270,90},
        {197,0,270,90,6.32,270,90},
        {198,0,270,90,6.28,270,90},
        {199,0,270,90,6.24,270,90},
        {200,0,270,90,6.2,270,90},
        {201,0,270,90,6.16,270,90},
        {202,0,270,90,6.12,270,90},
        {203,0,270,90,6.08,270,90},
        {204,0,270,90,6.04,270,90},
        {205,0,270,90,6,270,90},
        {206,0,270,90,5.96,270,90},
        {207,0,270,90,5.92,270,90},
        {208,0,270,90,5.88,270,90},
        {209,0,270,90,5.84,270,90},
        {210,0,270,90,5.8,270,90},
        {211,0,270,90,5.76,270,90},
        {212,0,270,90,5.72,270,90},
        {213,0,270,90,5.68,270,90},
        {214,0,270,90,5.64,270,90},
        {215,0,270,90,5.6,270,90},
        {216,0,270,90,5.56,270,90},
        {217,0,270,90,5.52,270,90},
        {218,0,270,90,5.48,270,90},
        {219,0,270,90,5.44,270,90},
        {220,0,270,90,5.4,270,90},
        {221,0,270,90,5.36,270,90},
        {222,0,270,90,5.32,270,90},
        {223,0,270,90,5.28,270,90},
        {224,0,270,90,5.24,270,90},
        {225,0,270,90,5.2,270,90},
        {226,0,270,90,5.16,270,90},
        {227,0,270,90,5.12,270,90},
        {228,0,270,90,5.08,270,90},
        {229,0,270,90,5.04,270,90},
        {230,0,270,90,5,270,90},
        {231,0,270,90,4.96,270,90},
        {232,0,270,90,4.92,270,90},
        {233,0,270,90,4.88,270,90},
        {234,0,270,90,4.84,270,90},
        {235,0,270,90,4.8,270,90},
    };
  public void togglePIDs(boolean run) {
    arm.toggleArm(run);
    ext.toggleExt(run);
    wrist.toggleWrist(run);
  }
}
