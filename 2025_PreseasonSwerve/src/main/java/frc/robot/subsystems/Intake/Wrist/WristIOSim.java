// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
  /** Creates a new IntakeWristIOSim. */
  private final DCMotor wristGearbox = DCMotor.getKrakenX60(1);

  private ProfiledPIDController m_controller;
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 0);
  
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          wristGearbox,
          IntakeWristConstants.IntakeWristSimConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              IntakeWristConstants.IntakeWristSimConstants.kArmLength,
              IntakeWristConstants.IntakeWristSimConstants.kArmMass),
          IntakeWristConstants.IntakeWristSimConstants.kArmLength,
          IntakeWristConstants.IntakeWristSimConstants.kMinAngleRads,
          IntakeWristConstants.IntakeWristSimConstants.kMaxAngleRads,
          true, // change this to true later
          0.1);

  private EncoderSim m_encoderSim;

  public void IntakeWristIOSim() {
    m_encoderSim =
        new EncoderSim(
            new Encoder(
                IntakeWristConstants.IntakeWristSimConstants.kEncoderAChannel,
                IntakeWristConstants.IntakeWristSimConstants.kEncoderBChannel));

    m_encoderSim.setDistancePerPulse(
        IntakeWristConstants.IntakeWristSimConstants.kArmEncoderDistPerPulse);
    m_controller =
        new ProfiledPIDController(
            IntakeWristConstants.IntakeWristSimConstants.kPivotSimPID[0],
            IntakeWristConstants.IntakeWristSimConstants.kPivotSimPID[1],
            IntakeWristConstants.IntakeWristSimConstants.kPivotSimPID[2],
            new TrapezoidProfile.Constraints(2.45, 2.45));

    m_controller.setTolerance(0.1, 0.05);
  }

  @Override
  public void updateInputs(IntakeWristIOInputs inputs) {
    sim.update(0.02);
    inputs.angleRads = getAngle();
    inputs.angVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.setpointAngleRads = m_controller.getSetpoint().position;
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void goToSetpoint(double setpoint) {
    m_controller.setGoal(setpoint);
    double pidOutput = m_controller.calculate(getAngle());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);

    sim.setInputVoltage(pidOutput + feedforwardOutput);
  }

  @Override
  public double getAngle() {
    return sim.getAngleRads();
  }

  @Override
  public boolean atSetpoint() {
    return m_controller.atGoal();
  }

  @Override
  public void setP(double p) {
    m_controller.setP(p);
  }

  @Override
  public void setI(double i) {
    m_controller.setI(i);
  }

  @Override
  public void setD(double d) {
    m_controller.setD(d);
  }

  @Override
  public void setkS(double kS) {
    m_feedforward = new SimpleMotorFeedforward(kS, m_feedforward.getKv());
  }

  @Override
  public void setkV(double kV) {
    m_feedforward = new SimpleMotorFeedforward(m_feedforward.getKs(), kV);
  }

  @Override
  public double getP() {
    return m_controller.getP();
  }

  @Override
  public double getI() {
    return m_controller.getI();
  }

  @Override
  public double getD() {
    return m_controller.getD();
  }

  @Override
  public double getkS() {
    return m_feedforward.getKs();
  }

  @Override
  public double getkV() {
    return m_feedforward.getKv();
  }
}