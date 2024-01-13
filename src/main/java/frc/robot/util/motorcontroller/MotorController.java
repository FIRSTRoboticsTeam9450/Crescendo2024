package frc.robot.util.motorcontroller;


import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

public abstract class MotorController {
	



	public abstract void configFactoryDefault();

	public abstract void setPIDF(double P, double I, double D, double F);

	/** Units: rotations for position or velocity, voltage for voltage, percent for percent */
	public abstract void set(double setpoint);

	/** Units: rotations for position or velocity, voltage for voltage, percent for percent */

	public abstract void stop();

	/** Units: rotations */
	public abstract void setIntegratedEncoderPosition(double position);

	/** Units: rotations */
	public abstract double getIntegratedEncoderPosition();


	public abstract void useIntegratedEncoder();

	public abstract void setInverted(boolean inverted);

	public abstract void setNominalVoltage(double voltage);

	public abstract void configureOptimization();

	public abstract void setAverageDepth(int depth);

	/** Units: rotations per second */
	public abstract double getVelocity();

	public abstract double getPercentOutput();

	public abstract double getCurrentOutput();

	public abstract void configCurrentLimit(int limit);

	public abstract double getFreeSpeedRPS();

	public abstract void setMeasurementPeriod(int periodMS);

	public abstract void flashMotor();

}
