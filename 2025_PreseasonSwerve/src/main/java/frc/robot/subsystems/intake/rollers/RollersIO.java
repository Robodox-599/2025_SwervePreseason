package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
    @AutoLog
    public static class RollersIOInputs {
        public double velocityRadsPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double velocitySetpoint = 0.0;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
    }
    

    public default void updateInputs(RollersIOInputs inputs){}
    
    public default void setVoltage(double voltage){}
        
    public default void setVelocity(double velocity){}
        
    public default void setBrake(boolean brake) {}

    public default void stop() {
        setVoltage(0.0);
    }

}

    
