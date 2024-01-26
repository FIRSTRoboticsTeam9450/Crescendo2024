// package frc.robot.subsystems;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Robot;

// public class ArmSubsystem extends SubsystemBase{
//     public enum Height{
//         LOW,
//         MID,
//         HIGH,
//         GROUND,
//     }
//     private Height currentHeight = Height.GROUND;

//     private double armTarget = 0.53;
//     private double extensionTarget = 1;

//     //double p = 0;

//     //public CANSparkMax intake = new CANSparkMax(Constants.intakeId, MotorType.kBrushless);

//     // private CANSparkMax leftMotor = new CANSparkMax(Constants.rotationLeftId,MotorType.kBrushless);
//     private CANSparkMax armMotor = new CANSparkMax(Constants.armId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    
    
//     private boolean runStuff = true;

//     private CANSparkMax extensionMotor = new CANSparkMax(Constants.extensionId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    
    
//     private final ProfiledPIDController armPid = new ProfiledPIDController(6.1, 0, 0, new Constraints(1, 0.5));//maxVel = 3.5 and maxAccel = 2.5
//     private final ArmFeedforward armFF = new ArmFeedforward(0, 0.027, 0.00001); //0.027, 0.00001
//     private final ArmFeedforward armExtendedFF = new ArmFeedforward(0, 0.03, 0.00001);

//     //Extension
//     private final PIDController extensionPid = new PIDController(0.37, 0,0);

//     private PIDController pid = new PIDController(0.1, 0, 0), downPID = new PIDController(0.0085, 0, 0);
    
//     Timer timer = new Timer();
//     private double ticsPerArmRevolution = 144, ticsPerWristRevolution = /*172.8*/ 120, lowTics = (50/360) * ticsPerArmRevolution, midTics = (100/360) * ticsPerArmRevolution, highTics = (135/360) * ticsPerArmRevolution, groundTics = (37.4/360) * ticsPerArmRevolution;
//     private boolean intialization = true;

//     DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);


//     public ArmSubsystem(){
//         armMotor.restoreFactoryDefaults();
//         extensionMotor.restoreFactoryDefaults();

//         armMotor.setSmartCurrentLimit(40);
//         extensionMotor.setSmartCurrentLimit(40);
//         armMotor.setIdleMode(IdleMode.kBrake);
//         extensionMotor.setIdleMode(IdleMode.kBrake);

//         extensionMotor.getEncoder().setPosition(0);
        
    
//         armPid.reset(getArmPosition());
        
//         setArmGoal(0.53);
//         setExtensionGoal(1);

//     }

//     public double getArmPosition(){
//         return absEncoder.get();
//     }

//     // private double getLeftPosition(){
//     //     return leftMotor.getEncoder().getPosition() * -2.5 * Math.PI / 180;
//     // }

//     // private double getPosition(){
//     //     return (getLeftPosition() + getRightPosition()) / 2;
//     // }


//     // public void setLeftVoltage(double voltage){
//     //     leftMotor.setVoltage(-voltage);
//     // }

//     public void setArmVoltage(double voltage){
//         // leftMotor.setVoltage(-voltage);
//         armMotor.setVoltage(voltage);

//     }

//     public void setExtensionVoltage(double voltage){
//         extensionMotor.setVoltage(voltage);
//     }

//     public void downManual(){
//         armTarget -= 0.01;

//         //wrist.setGoal(2.57 - armTarget);
//     }

//     public void upManual(){
//         armTarget += 0.01;

//         //wrist.setGoal(2.57 - armTarget);
//     }

//     public double getExtensionPosition(){
        
//         return extensionMotor.getEncoder().getPosition();
          
//     }

//     public void setArmGoal(double target) {
//         //rotation.setGoal(target);
//         armTarget = target;
//     }

//     public void setExtensionGoal(double target){
//         extensionTarget = target;
//     }

//     public double getGoal(){
//         return armPid.getGoal().position;
//     }

//     public double calculateRotationPID(){
//         return armPid.calculate(getArmPosition(), armTarget);
//     }
    
//     public double calculateExtensionPID(){
//         return extensionPid.calculate(getExtensionPosition(), extensionTarget);
//     }

//     public double calculateExtensionFF() {
//         return (-1 * Math.abs((1.44 * getArmPosition()) - 0.7632)) + 0.135;
//     }

//     public void updateExtensionOutput(){
//         double ffValue = calculateExtensionFF();
//         SmartDashboard.putNumber("Extension FF", ffValue);
//         double percentOutput = MathUtil.clamp(calculateExtensionPID(), -1.0, 1.0);
//         SmartDashboard.putNumber("Extension Percent", percentOutput);

//         double voltage = convertToVolts(percentOutput);

//         voltage = MathUtil.clamp(voltage /*+ ffValue*/, -6, 6);

//         SmartDashboard.putNumber("Extension Voltage", voltage);

        
//         setExtensionVoltage(voltage);
        
        
//     }



    

//     public void updateRotationOutput(){
//         double ffValue = calculateRotationFF();
//         double percentOutput = MathUtil.clamp(calculateRotationPID() + ffValue, -1.0, 1.0);
//         double voltage = convertToVolts(percentOutput);

//         SmartDashboard.putNumber("Rotation FF", ffValue);
//         SmartDashboard.putNumber("Rotation Voltage", voltage);
        
        
//         setArmVoltage(voltage);
        
        
//     }
    



    

//     public double calculateRotationFF(){
//         if(extensionTarget == 30){
//             return armExtendedFF.calculate(getArmPosition(), armPid.getSetpoint().velocity);
//         }

//         return armFF.calculate(getArmPosition(), armPid.getSetpoint().velocity);

        
//     }
    

//     private double convertToVolts(double percentOutput){
//         return percentOutput * Robot.getInstance().getVoltage();
//     }

//     @Override
//     public void periodic(){
        
//         if(runStuff){
//             updateRotationOutput();
//             //updateWristOutput();
//             updateExtensionOutput();
//             //setArmVoltage(0);

//         }else{
//             setArmVoltage(0);
//             setExtensionVoltage(0);
//         }
//         // SmartDashboard.putNumber("LeftPosition", getLeftPosition());
//         SmartDashboard.putNumber("Arm Position", getArmPosition());
//         SmartDashboard.putNumber("Target", armTarget);
//         SmartDashboard.putNumber("Extension Position", getExtensionPosition());
//         SmartDashboard.putNumber("Extension Target", extensionTarget);


//         //SmartDashboard.putNumber("Wrist Error", wrist.getPositionError());
//         //SmartDashboard.putNumber("Position Error", rotation.getPositionError());
        
//         //SmartDashboard.putNumber("Wrist Position", getWristPosition());
//         //SmartDashboard.putNumber("Arm current", rightMotor.getOutputCurrent());
        

        
//     }
    
    


//     // public void example(){
//     //     Shuffleboard.getTab("Arm")
//     //     .add("Arm P value", p)
//     //     .withSize(2, 1)
//     //     .withWidget(BuiltInWidgets.kNumberSlider)
//     //     .withProperties(Map.of("Min", 0, "Max", 0.025))
//     //     .getEntry();
        
//     // }
    












//     public void anEmptyMethod() {
//         // for testing
//     }
//     // public double getLeftRotPos() {
//     //     return leftMotor.getEncoder().getPosition() ;
//     // }

//     public double convertToRads(double angle) {
//         return angle/360*2*Math.PI;
//     }

//     // --------------------------------------------------------------

//     public void changeHeight(Height height){
//         currentHeight = height;
//     }

//     public Height getHeight(){
//         return currentHeight;
//     }

//     public void goToHeight() {
//         if(currentHeight == Height.HIGH){
//             setArmGoal(0.5); //Need to change
//         }else if(currentHeight == Height.MID){
//             setArmGoal(0.5); //Need to change
//         }else if(currentHeight == Height.LOW){
//             setArmGoal(0); //Need to change
//         }else{
//             setArmGoal(0.285);
//         }
//     }

//     public double convertHeightToTics(){
//         if(currentHeight == Height.HIGH){
//             return highTics; //37.071

//         }else if(currentHeight == Height.MID){
//             return midTics; //27.118

//         }else if(currentHeight == Height.LOW){
//             return lowTics; //14.452

//         }else{
//             return groundTics; //0

//         }
//     }


//     public void setPower(double power){
//         // leftMotor.set(power);
//         armMotor.set(power);
//     }

// }