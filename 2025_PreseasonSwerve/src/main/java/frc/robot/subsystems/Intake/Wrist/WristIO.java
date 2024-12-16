// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  /** Creates a new IntakeWristIO. */
  @AutoLog
  public static class IntakeWristIOInputs {
    public double angleRads = 0.0;
    public double angVelocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double setpointAngleRads = 0.0;
    public double currentAmps = 0.0;
    public double tempCelcius = 0.0;
  }

  public default void updateInputs(IntakeWristIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setspeed(double speed) {}

  public default void setBrake(double brake) {}

  public default double getAngle() {
    return 0.0;
  }

  public default void stop() {}

  public default void goToSetpoint(double setpoint) {}

  public default void holdSetpoint(double setpoint) {}

  public default void setBrake(boolean brake) {}

  public default boolean atSetpoint() {
    return false;
  }

  public default double getP() {
    return 0.0;
  }

  public default double getI() {
    return 0.0;
  }

  public default double getD() {
    return 0.0;
  }

  public default double getFF() {
    return 0.0;
  }

  public default double getkS() {
    return 0.0;
  }

  public default double getkG() {
    return 0.0;
  }

  public default double getkV() {
    return 0.0;
  }

  public default double getkA() {
    return 0.0;
  }

  public default void setI(double i) {}

  public default void setD(double d) {}

  public default void setFF(double ff) {}

  public default void setkS(double kS) {}

  public default void setkV(double kV) {}

  public default void setkG(double kG) {}

  public default void setkA(double kA) {}

  public default void setP(double p) {}
}