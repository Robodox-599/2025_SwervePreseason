package frc.robot.subsystems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
  
  @AutoLog
  public static class FlywheelsIOInputs {
    
    public double upperFlywheelPosition = 0.0;
    public double upperFlywheelVelocityRadPerSec = 0.0;
    public double upperFlywheelAppliedVoltage = 0.0;
    public double upperFlywheelCurrentAmps = 0.0;
    public double upperTempCelcius = 0.0;

    public double lowerFlywheelsPosition = 0.0;
    public double lowerFlywheelsVelocityRadPerSec = 0.0;
    public double lowerFlywheelsAppliedVoltage = 0.0;
    public double lowerFlywheelsCurrentAmps = 0.0;
    public double lowerTempVelcius;
  
}

  public default void updateInputs(FlywheelsIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(
      double topVelocityRadsPerSec,
      double topWheelFFVolts,
      double bottomVelocityRadsPerSec,
      double bottomWheelFFVolts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
