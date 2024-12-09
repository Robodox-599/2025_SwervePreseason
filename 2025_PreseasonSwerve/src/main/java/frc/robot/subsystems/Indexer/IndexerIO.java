package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double velocityRadsPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double speedSetpoint = 0.0;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
    }
    public default void updateInputs(IndexerIOInputs inputs) {}
    
    public default void setVoltage(double voltage) {}

    public default void goToPose(double position) {}

    public default void setVelocity(double velocity) {}

    public default void setBrake(boolean brake) {}
}
