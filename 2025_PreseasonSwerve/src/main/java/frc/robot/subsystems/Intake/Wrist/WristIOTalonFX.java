// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.Logger;

public class WristIOTalonFX implements WristIO {

  // private static final double GEAR_RATIO = 1.5; // change to specific gear ratio later
  private final TalonFX intakeWristMotor =
      new TalonFX(IntakeWristConstants.wristMotorID, IntakeWristConstants.wristMotorCANBus);

  private double setpoint = 0;
  private double motorEncoder;
  private int m_WristSlot = 0;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<AngularVelocity> angleVelocityRadsPerSec;
  private final StatusSignal<Temperature> tempCelcius;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Angle> angleRads;

  public WristIOTalonFX() {
    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 30.0; // maybe change laterr????
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kP = IntakeWristConstants.wristExtendKP;
    config.Slot0.kI = IntakeWristConstants.wristExtendKI;
    config.Slot0.kD = IntakeWristConstants.wristExtendKD;
    config.Slot0.kS = IntakeWristConstants.wristExtendKS;

    config.Slot1.kP = IntakeWristConstants.wristRetractKP;
    config.Slot1.kI = IntakeWristConstants.wristRetractKI;
    config.Slot1.kD = IntakeWristConstants.wristRetractKD;

    var mmConfig = config.MotionMagic;
    mmConfig.MotionMagicCruiseVelocity = IntakeWristConstants.maxWristVelocity;
    mmConfig.MotionMagicAcceleration = IntakeWristConstants.maxWristAccel;

    intakeWristMotor.getConfigurator().apply(config);
    intakeWristMotor.setPosition(0);
    setBrake(true);

    motorEncoder = intakeWristMotor.getPosition().getValueAsDouble();
    appliedVolts = intakeWristMotor.getSupplyVoltage();
    angleVelocityRadsPerSec = intakeWristMotor.getVelocity();
    tempCelcius = intakeWristMotor.getDeviceTemp();
    angleRads = intakeWristMotor.getPosition();
    currentAmps = intakeWristMotor.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, appliedVolts, tempCelcius, angleRads, currentAmps);
    intakeWristMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeWristIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVolts, tempCelcius, angleRads, currentAmps);
    inputs.setpointAngleRads = setpoint;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempCelcius = tempCelcius.getValueAsDouble();
    inputs.angleRads = angleRads.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();

    Logger.recordOutput("IntakeWrist/MotorEncoder", motorEncoder);
  }

  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("IntakeWrist/AppliedVolts", motorVolts);
    intakeWristMotor.setVoltage(motorVolts);
  }

  @Override
  public double getAngle() {
    return (Units.radiansToRotations(motorEncoder));
  }

  public void desiredWristSetPos(double passedInPosition) {
    m_WristSlot =
        passedInPosition == IntakeWristConstants.kWristExtendVal
            ? IntakeWristConstants.wristExtendSlot
            : IntakeWristConstants.wristRetractSlot;
    setpoint = passedInPosition;
    MotionMagicVoltage m_request =
        new MotionMagicVoltage(setpoint)
            .withSlot(m_WristSlot)
            .withFeedForward(IntakeWristConstants.kWristFeedForward);
    intakeWristMotor.setControl(m_request.withPosition(setpoint));
  }

  @Override
  public void goToSetpoint(double setpoint) {
    desiredWristSetPos(setpoint);
  }

  @Override
  public void holdSetpoint(double setpoint) {
    goToSetpoint(setpoint);
  }

  @Override
  public void setBrake(boolean brake) {
    if (brake) {
      intakeWristMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < IntakeWristConstants.intakeWristPositionTolerance;
  }
}
