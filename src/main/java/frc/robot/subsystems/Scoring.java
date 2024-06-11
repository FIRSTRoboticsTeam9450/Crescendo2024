// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BitToStickyfaultString;
import frc.robot.Constants;

public class Scoring extends SubsystemBase {
    /* Instantiate Low Level Subsystems */
    public Extension ext; // Extension Subsystem
    public Arm arm; // Arm Subsystem
    public Wrist wrist; // Wrist Subsystem

    /* Initializing Motors */
    private CANSparkFlex intake = new CANSparkFlex(Constants.intakeId, MotorType.kBrushless);

    /* Constants */
    private static final int EXTENSIONMININDEX = 1;
    private static final int WRISTMAXINDEX = 2;
    private static final int WRISTMININDEX = 3;
    private static final int EXTENSIONMAXINDEX = 4;
    private static final int WRISTMAXINDEX2 = 5;
    private static final int WRISTMININDEX2 = 6;


    private boolean laserIsDead;
    private boolean useVelocityIntake;

    /* Ramping variables */
    // If you want to ramp intake
    private boolean rampDownBool;
    private boolean isIntaking;
    private boolean intakeAtSpeed;

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
        //intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); // For Motor Position
        //intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Analog Sensor Voltage + Velocity + position
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Duty cycler velocity + pos
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50); // Duty Cycle Absolute Encoder Position and Abs angle
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Duty Cycle Absolute Encoder Velocity +
                                                                      // Frequency

        intake.burnFlash();

        intake.setIdleMode(IdleMode.kBrake);

        desiredArmAngle = Constants.Arm.climbArmPosition;
        desiredWristAngle = Constants.Wrist.climbWristPosition;
        desiredExtensionLength = Constants.Extension.climbExtPosition;

        // Optionally initialise the settings of the LaserCAN, if you haven't already
        // done so in GrappleHook
        laser = new LaserCan(Constants.laserId);
        try {
            laser.setRangingMode(LaserCan.RangingMode.LONG);
            laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Laser configuration failed! " + e);
        }
        measurement = laser.getMeasurement();
        setState(Constants.ScoringPos.NONE);

        rampDownTimer = new Timer();
        laserIsDead = false;
        useVelocityIntake = false; // set to true if you want to use velocity intake

    }

    public void logStickyFaults() {
        wrist.logMotorStickyFaults();
        arm.logMotorBackStickyFaults();
        arm.logMotorFrontStickyFaults();
        ext.logMotorStickyFaults();
        logMotorStickyFaults(); // intake
    }

    public void logMotorStickyFaults() {
        BitToStickyfaultString.getStickyFaultStringRevMotor(intake.getStickyFaults(), "Intake Motor");
        intake.clearFaults();
    }


    /** @return true when the laser is dead or when we want to use velocity intake */
    public boolean useVelocityIntake() {
        return this.laserIsDead || this.useVelocityIntake;
    }

    public void setUseVelocityIntake(boolean useVelocityIntake) {
        this.useVelocityIntake = useVelocityIntake;
    }

    public boolean getUseVelocityIntake() {
        return this.useVelocityIntake;
    }


    /** Has a median filter applied in this method */
    public double getLaserDistance() {
        try {
            measurement = laser.getMeasurement();
            if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                
                Logger.recordOutput("Laser/isDead", false);
                laserIsDead = false;
                return medianDistance.calculate(measurement.distance_mm);
            }
        } catch (Exception e) {
            Logger.recordOutput("Laser/isDead", true);

            laserIsDead = true;
            e.printStackTrace();
        }
        return 10000; // return a large number if the laser fails
    }

    // when outtaking set state to no note
    public void setIntakeVoltage(double voltage) {
        if (voltage < 0 && getUseVelocityIntake()) {
            setIntakeState(Constants.IntakeState.NO_NOTE);
        }
        intake.setVoltage(-voltage);
        SmartDashboard.putNumber("Intake Voltage", -voltage);
    }

    public double getIntakeTemp() {
        return intake.getMotorTemperature();
    }

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

    @SuppressWarnings("unused")
    private Constants.ScoringPos getLastState() {
        return lastState;
    }

    public Extension getExtSub() {
        return ext;
    }

    public Arm getArmSub() {
        return arm;
    }

    public Wrist getWristSub() {
        return wrist;
    }

    public void increaseExtBy(double inches) {
        desiredExtensionLength += inches;
    }

    public void increaseArmBy(double angle) {
        desiredArmAngle += angle;
    }

    public void increaseWristBy(double angle) {
        desiredWristAngle += angle;
    }

    public void setIntaking() {
        isIntaking = true;
    }

    public double getIntakeVelocity() {
        return intake.getEncoder().getVelocity();
    }

    /* Movement Logic */

    private void goToGround() {
        desiredArmAngle = Constants.Arm.groundArmPosition;
        desiredWristAngle = Constants.Wrist.groundWristPosition;
        desiredExtensionLength = Constants.Extension.groundExtPosition;

        // Sets the desired targets for each child subsystem
        ext.setTargetInches(desiredExtensionLength);
        arm.setTarget(desiredArmAngle);
        wrist.setTarget(desiredWristAngle);
    }

    private void goToStore() {
        desiredArmAngle = Constants.Arm.storeArmPosition;
        desiredWristAngle = Constants.Wrist.storeWristPosition;
        desiredExtensionLength = Constants.Extension.storeExtPosition;

        // Sets the desired targets for each child subsystem
        ext.setTargetInches(desiredExtensionLength);
        arm.setTarget(desiredArmAngle);
        wrist.setTarget(desiredWristAngle);
    }

    private void goToSource() {
        desiredArmAngle = Constants.Arm.sourceArmPosition;
        desiredWristAngle = Constants.Wrist.sourceWristPosition;
        desiredExtensionLength = Constants.Extension.sourceExtPosition;

        // Sets the desired targets for each child subsystem
        ext.setTargetInches(desiredExtensionLength);
        arm.setTarget(desiredArmAngle);
        wrist.setTarget(desiredWristAngle);
    }

    private void goToAmp() {
        if (stateRobotWhenIntaking.equals(Constants.ScoringPos.SOURCE)) {
            desiredArmAngle = Constants.Arm.ampArmPositionFromSource;
            desiredWristAngle = Constants.Wrist.ampWristPositionFromSource;
            desiredExtensionLength = Constants.Extension.ampExtPositionFromSource;
        } else {
            desiredArmAngle = Constants.Arm.ampArmPositionFromGround;
            desiredWristAngle = Constants.Wrist.ampWristPositionFromGround;
            desiredExtensionLength = Constants.Extension.ampExtPositionFromGround;
        }

        // Sets the desired targets for each child subsystem
        ext.setTargetInches(desiredExtensionLength);
        arm.setTarget(desiredArmAngle);
        wrist.setTarget(desiredWristAngle);
    }

    private void goToClimb() {
        desiredArmAngle = Constants.Arm.climbArmPosition;
        desiredWristAngle = Constants.Wrist.climbWristPosition;
        desiredExtensionLength = Constants.Extension.climbExtPosition;

        // Sets the desired targets for each child subsystem
        ext.setTargetInches(desiredExtensionLength);
        arm.setTarget(desiredArmAngle);
        wrist.setTarget(desiredWristAngle);
    }

    private void goToTrap() {
        desiredArmAngle = Constants.Arm.trapArmPosition;
        desiredWristAngle = Constants.Wrist.trapWristPosition;
        desiredExtensionLength = Constants.Extension.trapExtPosition;

        // Sets the desired targets for each child subsystem
        ext.setTargetInches(desiredExtensionLength);
        arm.setTarget(desiredArmAngle);
        wrist.setTarget(desiredWristAngle);
    }

    private void goToTemp() {
        desiredArmAngle = Constants.Arm.tempArmPosition;
        desiredWristAngle = 200;
        desiredExtensionLength = 0;

        // Sets the desired targets for each child subsystem
        ext.setTargetInches(desiredExtensionLength);
        arm.setTarget(desiredArmAngle);
        wrist.setTarget(desiredWristAngle);
    }

    /** only run once per button press, or logic breaks */
    public void goToPosition(Constants.ScoringPos nextState) {
        if (nextState.equals(this.state)) {
            return;
        }

        lastState = this.state;
        this.state = nextState;

        Logger.recordOutput("Score/State", state);
        switch (nextState) {
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
            case TEMP:
                goToTemp();
            case NONE:
                // do nothing
                break;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        limit(); //this is spamming the subsystems
        // SmartDashboard.putBoolean("Has Note", getIntakeState() == Constants.IntakeState.HAS_NOTE);
        // SmartDashboard.putBoolean("Using Laser", !getUseVelocityIntake());

        // Updates the state for whether the intake took a note or not
        Logger.recordOutput("Last Intake", stateRobotWhenIntaking);
        Logger.recordOutput("Intake State", stateOfIntake);
        Logger.recordOutput("Intake/Laser Dist", getLaserDistance());

        double armPosRads = Math.toRadians(arm.getAbsPos());
        // double wristPosRads = Math.toRadians(wrist.getAbsPos());

        // double height = 15 - (17 + ext.getRelPos()) * Math.cos(armPosRads) + 9 * Math.cos(armPosRads - wristPosRads);

        Logger.recordOutput("Robot Height", (17 + ext.getRelPos()) * Math.cos(armPosRads));

        if (useVelocityIntake()) {
            if (isIntaking && getIntakeVelocity() < -2500 && getIntakeState() != Constants.IntakeState.HAS_NOTE) {
                intakeAtSpeed = true;
            }
            if (intakeAtSpeed) {
                isIntaking = false;
                if (getIntakeVelocity() > -700) {
                    // intook a note
                    setIntakeState(Constants.IntakeState.HAS_NOTE);
                    setStateRobotWhenIntaking(getState());
                    intakeAtSpeed = false;
                    isIntaking = false;
                }
            }
        } else {
            if (getLaserDistance() <= 50 /* mm */ && getIntakeState() != Constants.IntakeState.HAS_NOTE) {
                setIntakeState(Constants.IntakeState.HAS_NOTE);
                setStateRobotWhenIntaking(getState());
            } else if (getLaserDistance() > 50) {
                setIntakeState(Constants.IntakeState.NO_NOTE);
                // setStateRobotWhenIntaking(Constants.ScoringPos.SOURCE); // default state
            }
        }
        

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

    // Hmmm maybe not yet(want to find shortest / fastest path kinda like a
    // pathplanner thing?)
    public void smartLimiter() {

    }

    public boolean isFinished() {
        return arm.finishedPID() && ext.finishedPID() && wrist.finishedPID();
    }

    public double getArmAngle() {
        return arm.getAbsPos();
    }

    public void zeroExtension() {
        ext.runAndResetEncoder();
    }

    public void limit() {
        double armAngle = arm.getAbsPos(); // get the current arm angle
        // armIndex is capped between min and max indexes of array
        int armIndex = (int) Math.min(Math.max(armAngle, limits[0][0]), limits[limits.length - 1][0])
                - (int) limits[0][0];
        double extLength = ext.getRelPos(); // get the current extension length
        // double wristAngle = wrist.getAbsPos(); // get the current wrist angle

        // gets index for arm currently at
        double currentExtMax = limits[armIndex][EXTENSIONMAXINDEX];
        double currentExtMin = limits[armIndex][EXTENSIONMININDEX];

        // set target to the max it can be at the current angle if it is over max, and
        // otherwise, be regular
        ext.setTargetInches(Math.max(Math.min(desiredExtensionLength, currentExtMax), currentExtMin));

        double currentWristMax_ExtMax = limits[armIndex][WRISTMAXINDEX2];
        double currentWristMin_ExtMax = limits[armIndex][WRISTMININDEX2];

        double currentWristMax_ExtMin = limits[armIndex][WRISTMAXINDEX];
        double currentWristMin_ExtMin = limits[armIndex][WRISTMININDEX];

        // wrist is at the limit, and target isn't in opposite direction
        // so stop wrist
        wrist.setTarget(currentExtMax - extLength < 0.1
                ? Math.max(Math.min(desiredWristAngle, currentWristMax_ExtMax), currentWristMin_ExtMax)
                : Math.max(Math.min(desiredWristAngle, currentWristMax_ExtMin), currentWristMin_ExtMin));

    }

    private double[][] limits = new double[][] {
            // Arm Angle Extension Min Wrist Max Wrist min Extension max Wrist Max Wrist Min
            { 51, 0, 270, 90, 0, 270, 90 },
            { 52, 0, 270, 90, 0.093, 270, 90 },
            { 53, 0, 270, 90, 0.186, 270, 90 },
            { 54, 0, 270, 90, 0.279, 270, 90 },
            { 55, 0, 270, 90, 0.372, 270, 90 },
            { 56, 0, 270, 90, 0.465, 270, 90 },
            { 57, 0, 270, 90, 0.558, 270, 90 },
            { 58, 0, 270, 90, 0.651, 270, 90 },
            { 59, 0, 270, 90, 0.744, 270, 90 },
            { 60, 0, 270, 90, 0.837, 270, 90 },
            { 61, 0, 270, 90, 0.93, 270, 90 },
            { 62, 0, 270, 90, 1.023, 270, 90 },
            { 63, 0, 270, 90, 1.116, 270, 90 },
            { 64, 0, 270, 90, 1.209, 270, 90 },
            { 65, 0, 270, 90, 5.6, 270, 90 },
            { 66, 0, 270, 90, 5.6, 270, 90 },
            { 67, 0, 270, 90, 5.6, 270, 90 },
            { 68, 0, 270, 90, 5.6, 270, 90 },
            { 69, 0, 270, 90, 5.6, 270, 90 },
            { 70, 0, 270, 90, 5.6, 270, 90 },
            { 71, 0, 270, 90, 5.6, 270, 90 },
            { 72, 0, 270, 90, 5.6, 270, 90 },
            { 73, 0, 270, 90, 5.6, 270, 90 },
            { 74, 0, 270, 90, 5.6, 270, 90 },
            { 75, 0, 270, 90, 5.6, 270, 90 },
            { 76, 0, 270, 90, 5.6, 270, 90 },
            { 77, 0, 270, 90, 5.6, 270, 90 },
            { 78, 0, 270, 90, 5.6, 270, 90 },
            { 79, 0, 270, 90, 5.6, 270, 90 },
            { 80, 0, 270, 90, 5.6, 270, 90 },
            { 81, 0, 270, 90, 2.79, 270, 90 },
            { 82, 0, 270, 90, 2.883, 270, 90 },
            { 83, 0, 270, 90, 2.976, 270, 90 },
            { 84, 0, 270, 90, 3.069, 270, 90 },
            { 85, 0, 270, 90, 3.162, 270, 90 },
            { 86, 0, 270, 90, 3.255, 270, 90 },
            { 87, 0, 270, 90, 3.348, 270, 90 },
            { 88, 0, 270, 90, 3.441, 270, 90 },
            { 89, 0, 270, 90, 3.5, 270, 90 },
            { 90, 0, 270, 90, 3.5, 270, 90 },
            { 91, 0, 270, 90, 3.5, 270, 90 },
            { 92, 0, 270, 90, 3.5, 270, 90 },
            { 93, 0, 270, 90, 3.5, 270, 90 },
            { 94, 0, 270, 90, 3.5, 270, 90 },
            { 95, 0, 270, 90, 3.54, 270, 90 },
            { 96, 0, 270, 90, 3.58, 270, 90 },
            { 97, 0, 270, 90, 3.62, 270, 90 },
            { 98, 0, 270, 90, 3.66, 270, 90 },
            { 99, 0, 270, 90, 3.7, 270, 90 },
            { 100, 0, 270, 90, 3.74, 270, 90 },
            { 101, 0, 270, 90, 3.78, 270, 90 },
            { 102, 0, 270, 90, 3.82, 270, 90 },
            { 103, 0, 270, 90, 3.86, 270, 90 },
            { 104, 0, 270, 90, 3.9, 270, 90 },
            { 105, 0, 270, 90, 3.94, 270, 90 },
            { 106, 0, 270, 90, 3.98, 270, 90 },
            { 107, 0, 270, 90, 4.02, 270, 90 },
            { 108, 0, 270, 90, 4.06, 270, 90 },
            { 109, 0, 270, 90, 4.1, 270, 90 },
            { 110, 0, 270, 90, 4.14, 270, 90 },
            { 111, 0, 270, 90, 4.18, 270, 90 },
            { 112, 0, 270, 90, 4.22, 270, 90 },
            { 113, 0, 270, 90, 4.26, 270, 90 },
            { 114, 0, 270, 90, 4.3, 270, 90 },
            { 115, 0, 270, 90, 4.34, 270, 90 },
            { 116, 0, 270, 90, 4.38, 270, 90 },
            { 117, 0, 270, 90, 4.42, 270, 90 },
            { 118, 0, 270, 90, 4.46, 270, 90 },
            { 119, 0, 270, 90, 4.5, 270, 90 },
            { 120, 0, 270, 90, 4.54, 270, 90 },
            { 121, 0, 270, 90, 4.58, 270, 90 },
            { 122, 0, 270, 90, 4.62, 270, 90 },
            { 123, 0, 270, 90, 4.66, 270, 90 },
            { 124, 0, 270, 90, 4.7, 270, 90 },
            { 125, 0, 270, 90, 4.74, 270, 90 },
            { 126, 0, 270, 90, 4.78, 270, 90 },
            { 127, 0, 270, 90, 4.82, 270, 90 },
            { 128, 0, 270, 90, 4.86, 270, 90 },
            { 129, 0, 270, 90, 4.9, 270, 90 },
            { 130, 0, 270, 90, 4.94, 270, 90 },
            { 131, 0, 270, 90, 4.98, 270, 90 },
            { 132, 0, 270, 90, 5.02, 270, 90 },
            { 133, 0, 270, 90, 5.06, 270, 90 },
            { 134, 0, 270, 90, 5.1, 270, 90 },
            { 135, 0, 270, 90, 5.14, 270, 90 },
            { 136, 0, 270, 90, 5.18, 270, 90 },
            { 137, 0, 270, 90, 5.22, 270, 90 },
            { 138, 0, 270, 90, 5.26, 270, 90 },
            { 139, 0, 270, 90, 5.3, 270, 90 },
            { 140, 0, 270, 90, 5.34, 270, 90 },
            { 141, 0, 270, 90, 5.38, 270, 90 },
            { 142, 0, 270, 90, 5.42, 270, 90 },
            { 143, 0, 270, 90, 5.46, 270, 90 },
            { 144, 0, 270, 90, 5.5, 270, 90 },
            { 145, 0, 270, 90, 5.54, 270, 90 },
            { 146, 0, 270, 90, 5.58, 270, 90 },
            { 147, 0, 270, 90, 5.62, 270, 90 },
            { 148, 0, 270, 90, 5.66, 270, 90 },
            { 149, 0, 270, 90, 5.7, 270, 90 },
            { 150, 0, 270, 90, 5.74, 270, 90 },
            { 151, 0, 270, 90, 5.78, 270, 90 },
            { 152, 0, 270, 90, 5.82, 270, 90 },
            { 153, 0, 270, 90, 5.86, 270, 90 },
            { 154, 0, 270, 90, 5.9, 270, 90 },
            { 155, 0, 270, 90, 5.94, 270, 90 },
            { 156, 0, 270, 90, 5.98, 270, 90 },
            { 157, 0, 270, 90, 6.02, 270, 90 },
            { 158, 0, 270, 90, 6.06, 270, 90 },
            { 159, 0, 270, 90, 6.1, 270, 90 },
            { 160, 0, 270, 90, 6.14, 270, 90 },
            { 161, 0, 270, 90, 6.18, 270, 90 },
            { 162, 0, 270, 90, 6.22, 270, 90 },
            { 163, 0, 270, 90, 6.26, 270, 90 },
            { 164, 0, 270, 90, 6.3, 270, 90 },
            { 165, 0, 270, 90, 6.34, 270, 90 },
            { 166, 0, 270, 90, 6.38, 270, 90 },
            { 167, 0, 270, 90, 6.42, 270, 90 },
            { 168, 0, 270, 90, 6.46, 270, 90 },
            { 169, 0, 270, 90, 6.5, 270, 90 },
            { 170, 0, 270, 90, 6.54, 270, 90 },
            { 171, 0, 270, 90, 6.58, 270, 90 },
            { 172, 0, 270, 90, 6.62, 270, 90 },
            { 173, 0, 270, 90, 6.66, 270, 90 },
            { 174, 0, 270, 90, 6.7, 270, 90 },
            { 175, 0, 270, 90, 6.74, 270, 90 },
            { 176, 0, 270, 90, 6.78, 270, 90 },
            { 177, 0, 270, 90, 6.82, 270, 90 },
            { 178, 0, 270, 90, 6.86, 270, 90 },
            { 179, 0, 270, 90, 6.9, 270, 90 },
            { 180, 0, 270, 90, 7, 270, 90 },
            { 181, 0, 270, 90, 6.96, 270, 90 },
            { 182, 0, 270, 90, 6.92, 270, 90 },
            { 183, 0, 270, 90, 6.88, 270, 90 },
            { 184, 0, 270, 90, 6.84, 270, 90 },
            { 185, 0, 270, 90, 7, 270, 90 },
            { 186, 0, 270, 90, 7, 270, 90 },
            { 187, 0, 270, 90, 7.5, 270, 90 },
            { 188, 0, 270, 90, 8, 270, 90 },
            { 189, 0, 270, 90, 8.5, 270, 90 },
            { 190, 0, 270, 90, 9, 270, 90 },
            { 191, 0, 270, 90, 9.5, 270, 90 },
            { 192, 0, 270, 90, 10, 270, 90 },
            { 193, 0, 270, 90, 10.5, 270, 90 },
            { 194, 0, 270, 90, 11, 270, 90 },
            { 195, 0, 270, 90, 11.5, 270, 90 },
            { 196, 0, 270, 90, 12, 270, 90 },
            { 197, 0, 270, 90, 12, 270, 90 },
            { 198, 0, 270, 90, 12, 270, 90 },
            { 199, 0, 270, 90, 12, 270, 90 },
            { 200, 0, 270, 90, 12, 270, 90 },
            { 201, 0, 270, 90, 12, 270, 90 },
            { 202, 0, 270, 90, 12, 270, 90 },
            { 203, 0, 270, 90, 12, 270, 90 },
            { 204, 0, 270, 90, 12, 270, 90 },
            { 205, 0, 270, 90, 12, 270, 90 },
            { 206, 0, 270, 90, 5.96, 270, 90 },
            { 207, 0, 270, 90, 5.92, 270, 90 },
            { 208, 0, 270, 90, 5.88, 270, 90 },
            { 209, 0, 270, 90, 5.84, 270, 90 },
            { 210, 0, 270, 90, 5.8, 270, 90 },
            { 211, 0, 270, 90, 5.76, 270, 90 },
            { 212, 0, 270, 90, 5.72, 270, 90 },
            { 213, 0, 270, 90, 5.68, 270, 90 },
            { 214, 0, 270, 90, 5.64, 270, 90 },
            { 215, 0, 270, 90, 5.6, 270, 90 },
            { 216, 0, 270, 90, 5.56, 270, 90 },
            { 217, 0, 270, 90, 5.52, 270, 90 },
            { 218, 0, 270, 90, 5.48, 270, 90 },
            { 219, 0, 270, 90, 5.44, 270, 90 },
            { 220, 0, 270, 90, 5.4, 270, 90 },
            { 221, 0, 270, 90, 5.36, 270, 90 },
            { 222, 0, 270, 90, 5.32, 270, 90 },
            { 223, 0, 270, 90, 5.28, 270, 90 },
            { 224, 0, 270, 90, 5.24, 270, 90 },
            { 225, 0, 270, 90, 5.2, 270, 90 },
            { 226, 0, 270, 90, 5.16, 270, 90 },
            { 227, 0, 270, 90, 5.12, 270, 90 },
            { 228, 0, 270, 90, 5.08, 270, 90 },
            { 229, 0, 270, 90, 5.04, 270, 90 },
            { 230, 0, 270, 90, 5, 270, 90 },
            { 231, 0, 270, 90, 4.96, 270, 90 },
            { 232, 0, 270, 90, 4.92, 270, 90 },
            { 233, 0, 270, 90, 4.88, 270, 90 },
            { 234, 0, 270, 90, 4.84, 270, 90 },
            { 235, 0, 270, 90, 4.8, 270, 90 },
    };

    public void togglePIDs(boolean run) {
        arm.toggleArm(run);
        ext.toggleExt(run);
        wrist.toggleWrist(run);
    }
}
