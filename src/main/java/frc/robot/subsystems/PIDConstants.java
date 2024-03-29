package frc.robot.subsystems;

public class PIDConstants{ 
        public double kP;
        public double maxVoltage;
        public PIDConstants(double kP, double maxVoltage) {
            this.kP = kP;
            this.maxVoltage = maxVoltage;
        }

    }
