package frc.robot.util.motorcontroller;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DriverStation;


public class BrushlessSparkFlexController extends MotorController {
	private static final double TICKS_PER_ROTATION = 42.0;
	private static final double FREE_SPEED_RPS = 5676.0 / 60.0;

	private final CANSparkFlex motor;
	private final SparkPIDController motorPID;


	public BrushlessSparkFlexController(int id) {
		this.motor = new CANSparkFlex(id, MotorType.kBrushless);
		this.motorPID = motor.getPIDController();

		useIntegratedEncoder();
	}

	

	

	@Override
	public void configFactoryDefault() {
		this.motor.restoreFactoryDefaults();
	}

	@Override
	public void setPIDF(double P, double I, double D, double F) {
		motorPID.setP(P);
		motorPID.setI(I);
		motorPID.setD(D);
		motorPID.setFF(F);
	}

	@Override
	public void set(double power) {
		motor.set(power);
	}
	
	@Override
	public void stop() {
		motor.stopMotor();
	}

	@Override
	public void setIntegratedEncoderPosition(double position) {
		motor.getEncoder().setPosition(position);
	}

	@Override
	public double getIntegratedEncoderPosition() {
		return motor.getEncoder().getPosition();
	}

	

	public void setFeedbackDevice(MotorFeedbackSensor sensor) {
		motorPID.setFeedbackDevice(sensor);
	}

	@Override
	public void useIntegratedEncoder() {
		setFeedbackDevice(motor.getEncoder());
	}

	@Override
	public void setInverted(boolean invert) {
		motor.setInverted(invert);
	}

	@Override
	public void setNominalVoltage(double voltage) {
		motor.enableVoltageCompensation(voltage);
	}

	@Override
	public void configureOptimization() {
		// motorPID.setPositionPIDWrappingEnabled(true);
		// motorPID.setPositionPIDWrappingMaxInput(DrivebaseSubsystem.STEER_REDUCTION);
		// motorPID.setPositionPIDWrappingMinInput(0);
	}

	@Override
	public void setAverageDepth(int depth) {
		motor.getEncoder().setAverageDepth(depth);
	}

	@Override
	public double getVelocity() {
		// rpm to rps
		return motor.getEncoder().getVelocity() / 60;
	}

	@Override
	public double getPercentOutput() {
		return motor.getAppliedOutput();
	}

	@Override
	public double getCurrentOutput() {
		return motor.getOutputCurrent();
	}

	@Override
	public void configCurrentLimit(int limit) {
		motor.setSmartCurrentLimit(limit);
	}

	@Override
	public double getFreeSpeedRPS() {
		return FREE_SPEED_RPS;
	}

	@Override
	public void setMeasurementPeriod(int periodMS) {
		motor.getEncoder().setMeasurementPeriod(periodMS);
	}

	@Override
	public void flashMotor() {
		motor.burnFlash();
	}

	
}
