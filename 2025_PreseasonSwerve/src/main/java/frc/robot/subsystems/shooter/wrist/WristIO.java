package frc.robot.subsystems.shooter.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  
  @AutoLog
  public static class WristIOInputs {
    public double velocityRadsPerSecs = 0.0;
    public double appliedVoltage = 0.0;
    public double currentAmps = 0.0;
    public double tempCelcius = 0.0;
    public double angleRads = 0.0;
    public double setpointAngleRads = 0.0;

  }

 public default void updateInputs(WristIOInputs inputs) {}

 public default void setVoltage(double voltage) {}

 public default void setBrake(boolean brake){}

 public default void goToPose(double position){}

 public default double getAngle() {
     return 0.0;
 }
}
