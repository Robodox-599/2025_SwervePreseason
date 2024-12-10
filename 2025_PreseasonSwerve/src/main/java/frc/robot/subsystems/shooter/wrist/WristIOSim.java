package frc.robot.subsystems.shooter.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.robot.subsystems.shooter.wrist.WristConstants.*;

public class WristIOSim implements WristIO{
    private final DCMotorSim wristMotorSim =
    new DCMotorSim(DCMotor.getKrakenX60(1), gearRatio, wristMOI);
private PIDController controller = new PIDController(0, 0, 0);
private SimpleMotorFeedforward ff;
private double appliedVoltage;
private double desiredSpeed;

public WristIOSim() {
    controller.setPID(simP, simI, simD);
    ff = new SimpleMotorFeedforward(simS, simV);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.velocityRadsPerSecs = .getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.currentAmps = wristMotorSim.getCurrentDrawAmps();
    inputs.tempCelcius = 60;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = volts;
    wristMotorSim.setInputVoltage(volts);
  }

    // @Override
  // public void setBrake(boolean brake) {
  //   if(brake) {
  //     indexerMotorSim.setNeutralMode(NeutralModeValue.Brake);
  //   } else {
  //     indexerMotorSim.setNeutralMode(NeutralModeValue.Coast);
  //   }
  // }

  @Override
  public void goToPose(double pose){
    double volts = controller.calculate(wristMotorSim.getAngularPositionRad(), pose);
    wristMotorSim.setInputVoltage(volts);
  }
}
