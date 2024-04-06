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


    /* Constants  */
    private static final int ANGLEINDEX = 0; 
    private static final int EXTENSIONMININDEX = 1;
    private static final int WRISTMAXINDEX = 2;
    private static final int WRISTMININDEX = 3;
    private static final int EXTENSIONMAXINDEX = 4;
    private static final int WRISTMAXINDEX2 = 5;
    private static final int WRISTMININDEX2 = 6;


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
        desiredArmAngle = Constants.NewArm.groundArmPosition;
        desiredWristAngle = Constants.NewWrist.groundWristPosition;
        desiredExtensionLength = Constants.NewExtension.groundExtPosition;
    }   

    private void goToStore() {
        desiredArmAngle = Constants.NewArm.storeArmPosition;
        desiredWristAngle = Constants.NewWrist.storeWristPosition;
        desiredExtensionLength = Constants.NewExtension.storeExtPosition;
    }
    
    private void goToSource() {
        desiredArmAngle = Constants.NewArm.sourceArmPosition;
        desiredWristAngle = Constants.NewWrist.sourceWristPosition;
        desiredExtensionLength = Constants.NewExtension.sourceExtPosition;
    }

    private void goToAmp() {
        desiredArmAngle = Constants.NewArm.ampArmPosition;
        desiredWristAngle = Constants.NewWrist.ampWristPosition;
        desiredExtensionLength = Constants.NewExtension.ampExtPosition;
    }

    private void goToClimb() {
        desiredArmAngle = Constants.NewArm.climbArmPosition;
        desiredWristAngle = Constants.NewWrist.climbWristPosition;
        desiredExtensionLength = Constants.NewExtension.climbExtPosition;
    }

    private void goToTrap() {
        desiredArmAngle = Constants.NewArm.trapArmPosition;
        desiredWristAngle = Constants.NewWrist.trapWristPosition;
        desiredExtensionLength = Constants.NewExtension.trapExtPosition;
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
        ext.setTarget(desiredArmAngle);
        arm.setTarget(desiredWristAngle);
        wrist.setTarget(desiredExtensionLength);
       


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

    /* Limiter Logic */
    // add limit code 

    //Hmmm maybe not yet(want to find shortest / fastest path kinda like a pathplanner thing?)
    public void smartLimiter(){

    }
    public void limit(){
        double armAngle = arm.getRelPos(); // get the current arm angle
        double extLength = ext.getRelPos(); // get the current extension length
        double wristAngle = wrist.getAbsPos(); // get the current wrist angle
        // if arm angle is between max and min arm angles
        if(Math.abs(armAngle) > Math.abs(limits[0][ANGLEINDEX]) && Math.abs(armAngle) < Math.abs(limits[190][ANGLEINDEX]) ){ 

            arm.toggleArm(true); // resume arm

            // if the ext amt is within error amt for max ext
            // optional version that accounts for final index if you want to look 1 angle ahead: limits[(int) Math.round(armAngle) - 45 + 1 > 235 ? 235 : armAngle - 45 + 1][4]
            // This part of the if statements basically checks if the extension is less than the max it can be at the current angle
            // the inside part is the max and min code for the wrist limits
            if (limits[(int) Math.round(armAngle) - 45][EXTENSIONMAXINDEX] - extLength < 0.1) { // arm angle increments by 1 but starts by 45, so take armAngle - 45 to get proper index
                // extension is at the limit, so stop ext
                ext.toggleExt(false); // stop ext
                System.out.println("EXTENSION STOPPED (MAX)");

                // if the wrist amt is outside of the limits
                if (limits[(int) Math.round(armAngle) - 45][WRISTMININDEX2] + 2 /* 2 degree tolerance */ > wristAngle || wristAngle > limits[(int) Math.round(armAngle) - 45][WRISTMAXINDEX2] - 2) { 
                    // wrist is at the limit, so stop wrist
                    wrist.toggleWrist(false); // stop wrist
                    System.out.println("WRIST STOPPED");
                } else {
                    wrist.toggleWrist(true); // resume wrist
                }

            //This part of the if statement checks if the extension is greather than the mininum(or at the min ofc) it can be at its current angle
            // This inside is the max and min angle of the wrist.
            } else if (extLength - limits[(int) Math.round(armAngle) - 45][EXTENSIONMININDEX] < 0.1) { // if the ext amt is within error amt for min ext
                ext.toggleExt(false); // stop ext
                System.out.println("EXTENSION STOPPED (MIN)");

                // if the wrist amt is outside of the limits
                if (limits[(int) Math.round(armAngle) - 45][WRISTMININDEX2] + 2 /* 2 degree tolerance */ > wristAngle || wristAngle > limits[(int) Math.round(armAngle) - 45][WRISTMAXINDEX2] - 2) { 
                    // wrist is at the limit, so stop wrist
                    wrist.toggleWrist(false); // stop wrist
                    System.out.println("WRIST STOPPED");
                } else {
                    wrist.toggleWrist(true); // resume wrist
                }

            } else {
                ext.toggleExt(true); // resume ext
            }

        }else{
            arm.toggleArm(false); // stop arm
            System.out.println("SOMETHING VERY WRONG WITH ARM");
        }
    }
    
    

    private double[][] limits = new double[][]{
        //Arm Angle		Extension Min		Wrist Max		Wrist min		Extension max		Wrist Max		Wrist Min
        {45,0,270,250,0,270,250},
        {46,0,270,245,0.23,270,246.5},
        {47,0,270,240,0.46,270,243},
        {48,0,270,235,0.69,270,239.5},
        {49,0,270,230,0.92,270,236},
        {50,0,270,225,1.15,270,232.5},
        {51,0,270,220,1.38,270,229},
        {52,0,270,215,1.61,270,225.5},
        {53,0,270,210,1.84,270,222},
        {54,0,270,205,2.07,270,218.5},
        {55,0,270,200,2.3,270,215},
        {56,0,270,195,2.53,270,211.5},
        {57,0,270,190,2.76,270,208},
        {58,0,270,185,2.99,270,204.5},
        {59,0,270,180,3.22,270,201},
        {60,0,270,175,3.45,270,197.5},
        {61,0,270,170,3.68,270,194},
        {62,0,270,165,3.91,270,190.5},
        {63,0,270,160,4.14,270,187},
        {64,0,270,150,4.25,270,180},
        {65,0,270,147.6,4.22,270,176.5},
        {66,0,270,145.2,4.19,270,173},
        {67,0,270,142.8,4.16,270,169.5},
        {68,0,270,140.4,4.13,270,166},
        {69,0,270,138,4.10,270,162.5},
        {70,0,270,135.6,4.07,270,159},
        {71,0,270,133.2,4.04,270,155.5},
        {72,0,270,130.8,4.01,270,152},
        {73,0,270,128.4,3.98,270,148.5},
        {74,0,270,126,3.95,270,145},
        {75,0,270,123.6,3.92,270,141.5},
        {76,0,270,121.2,3.89,270,138},
        {77,0,270,118.8,3.86,270,134.5},
        {78,0,270,116.4,3.83,270,131},
        {79,0,270,114,3.80,270,127.5},
        {80,0,270,111.6,3.77,270,124},
        {81,0,270,109.2,3.74,270,120.5},
        {82,0,270,106.8,3.71,270,117},
        {83,0,270,104.4,3.68,270,113.5},
        {84,0,270,102,3.65,270,110},
        {85,0,270,99.6,3.62,270,106.5},
        {86,0,270,97.2,3.59,270,103},
        {87,0,270,94.8,3.56,270,99.5},
        {88,0,270,92.4,3.53,270,96},
        {89,0,270,90,3.5,270,90},
        {90,0,270,90,3.5,270,90},
        {91,0,270,90,3.5,270,90},
        {92,0,270,90,3.5,270,90},
        {93,0,270,90,3.5,270,90},
        {94,0,270,90,3.5,270,90},
        {95,0,270,90,3.5,270,90},
        {96,0,270,90,3.5,270,90},
        {97,0,270,90,3.5,270,90},
        {98,0,270,90,3.5,270,90},
        {99,0,270,90,3.5,270,90},
        {100,0,270,90,3.5,270,90},
        {101,0,270,90,3.5,270,90},
        {102,0,270,90,3.5,270,90},
        {103,0,270,90,3.5,270,90},
        {104,0,270,90,3.5,270,90},
        {105,0,270,90,3.5,270,90},
        {106,0,270,90,3.5,270,90},
        {107,0,270,90,3.5,270,90},
        {108,0,270,90,3.5,270,90},
        {109,0,270,90,3.5,270,90},
        {110,0,270,90,3.5,270,90},
        {111,0,270,90,3.5,270,90},
        {112,0,270,90,3.5,270,90},
        {113,0,270,90,3.5,270,90},
        {114,0,270,90,3.5,270,90},
        {115,0,270,90,3.5,270,90},
        {116,0,270,90,3.5,270,90},
        {117,0,270,90,3.5,270,90},
        {118,0,270,90,3.5,270,90},
        {119,0,270,90,3.5,270,90},
        {120,0,270,90,3.5,270,90},
        {121,0,270,90,3.5,270,90},
        {122,0,270,90,3.5,270,90},
        {123,0,270,90,3.5,270,90},
        {124,0,270,90,3.5,270,90},
        {125,0,270,90,3.5,270,90},
        {126,0,270,90,3.5,270,90},
        {127,0,270,90,3.5,270,90},
        {128,0,270,90,3.5,270,90},
        {129,0,270,90,3.5,270,90},
        {130,0,270,90,3.5,270,90},
        {131,0,270,90,3.5,270,90},
        {132,0,270,90,3.5,270,90},
        {133,0,270,90,3.5,270,90},
        {134,0,270,90,3.5,270,90},
        {135,0,270,90,3.5,270,90},
        {136,0,270,90,3.5,270,90},
        {137,0,270,90,3.5,270,90},
        {138,0,270,90,3.5,270,90},
        {139,0,270,90,3.5,270,90},
        {140,0,270,90,3.5,270,90},
        {141,0,270,90,3.5,270,90},
        {142,0,270,90,3.5,270,90},
        {143,0,270,90,3.5,270,90},
        {144,0,270,90,3.5,270,90},
        {145,0,270,90,3.5,270,90},
        {146,0,270,90,3.5,270,90},
        {147,0,270,90,3.5,270,90},
        {148,0,270,90,3.5,270,90},
        {149,0,270,90,3.5,270,90},
        {150,0,270,90,3.5,270,90},
        {151,0,270,90,3.5,270,90},
        {152,0,270,90,3.5,270,90},
        {153,0,270,90,3.5,270,90},
        {154,0,270,90,3.5,270,90},
        {155,0,270,90,3.5,270,90},
        {156,0,270,90,3.5,270,90},
        {157,0,270,90,3.5,270,90},
        {158,0,270,90,3.5,270,90},
        {159,0,270,90,3.5,270,90},
        {160,0,270,90,3.5,270,90},
        {161,0,270,90,3.5,270,90},
        {162,0,270,90,3.5,270,90},
        {163,0,270,90,3.5,270,90},
        {164,0,270,90,3.5,270,90},
        {165,0,270,90,3.5,270,90},
        {166,0,270,90,3.5,270,90},
        {167,0,270,90,3.5,270,90},
        {168,0,270,90,3.5,270,90},
        {169,0,270,90,3.5,270,90},
        {170,0,270,90,3.5,270,90},
        {171,0,270,90,3.5,270,90},
        {172,0,270,90,3.5,270,90},
        {173,0,270,90,3.5,270,90},
        {174,0,270,90,3.5,270,90},
        {175,0,270,90,3.5,270,90},
        {176,0,270,90,3.5,270,90},
        {177,0,270,90,3.5,270,90},
        {178,0,270,90,3.5,270,90},
        {179,0,270,90,3.5,270,90},
        {180,0,270,90,3.5,270,90},
        {181,0,270,90,3.5,270,90},
        {182,0,270,90,3.5,270,90},
        {183,0,270,90,3.5,270,90},
        {184,0,270,90,3.5,270,90},
        {185,0,270,90,3.5,270,90},
        {186,0,270,90,3.5,270,90},
        {187,0,270,90,3.5,270,90},
        {188,0,270,90,3.5,270,90},
        {189,0,270,90,3.5,270,90},
        {190,0,270,90,3.5,270,90},
        {191,0,270,90,3.5,270,90},
        {192,0,270,90,3.5,270,90},
        {193,0,270,90,3.5,270,90},
        {194,0,270,90,3.5,270,90},
        {195,0,270,90,3.5,270,90},
        {196,0,270,90,3.5,270,90},
        {197,0,270,90,3.5,270,90},
        {198,0,270,90,3.5,270,90},
        {199,0,270,90,3.5,270,90},
        {200,0,270,90,3.5,270,90},
        {201,0,270,90,3.5,270,90},
        {202,0,270,90,3.5,270,90},
        {203,0,270,90,3.5,270,90},
        {204,0,270,90,3.5,270,90},
        {205,0,270,90,3.5,270,90},
        {206,0,270,90,3.5,270,90},
        {207,0,270,90,3.5,270,90},
        {208,0,270,90,3.5,270,90},
        {209,0,270,90,3.5,270,90},
        {210,0,270,90,3.5,270,90},
        {211,0,270,90,3.5,270,90},
        {212,0,270,90,3.5,270,90},
        {213,0,270,90,3.5,270,90},
        {214,0,270,90,3.5,270,90},
        {215,0,270,90,3.5,270,90},
        {216,0,270,90,3.5,270,90},
        {217,0,270,90,3.5,270,90},
        {218,0,270,90,3.5,270,90},
        {219,0,270,90,3.5,270,90},
        {220,0,270,90,3.5,270,90},
        {221,0,270,90,3.5,270,90},
        {222,0,270,90,3.5,270,90},
        {223,0,270,90,3.5,270,90},
        {224,0,270,90,3.5,270,90},
        {225,0,270,90,3.5,270,90},
        {226,0,270,90,3.5,270,90},
        {227,0,270,90,3.5,270,90},
        {228,0,270,90,3.5,270,90},
        {229,0,270,90,3.5,270,90},
        {230,0,270,90,3.5,270,90},
        {231,0,270,90,3.5,270,90},
        {232,0,270,90,3.5,270,90},
        {233,0,270,90,3.5,270,90},
        {234,0,270,90,3.5,270,90},
        {235,0,270,90,3.5,270,90}
    };
  
}
