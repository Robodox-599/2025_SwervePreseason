package frc.robot.subsystems.shooter.wrist;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.wrist.WristIO.WristIOInputs;

public class Wrist extends SubsystemBase{
    
    private final WristIOInputs inputs = new WristIOInputs();
    private final WristIO io;
    private double setpoint = 0.0;

    public Wrist(WristIO io) {
      this.io = io;
    }
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", (LoggableInputs) inputs);
      }

  @AutoLogOutput(key = "Wrist/WristAppliedVoltage")
  public double getVoltage(voltage) {
      io.setVoltage(motorVolts);
  }

    public Command applyVoltage(double voltage) {
    return Commands.run(
        () -> {
          io.setVoltage(voltage);
        });
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setVoltage(0);
        });
  }

  public Command setPose(double pose) {
    return Commands.run(
        () -> {
          io.goToPose(pose);
        });}
    }
