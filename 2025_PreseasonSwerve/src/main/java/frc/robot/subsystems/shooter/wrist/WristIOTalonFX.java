package frc.robot.subsystems.shooter.wrist;
import static frc.robot.subsystems.shooter.wrist.WristConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;


public class WristIOTalonFX implements WristIO{
  private TalonFX wristMotor;
  TalonFXConfiguration wristConfig;

//   private final StatusSignal<Double> angleRads;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<AngularVelocity> velocityRadsPerSec;
  private final StatusSignal<Temperature> tempCelcius;
  private final StatusSignal<Current> currentAmps;

  private final PositionVoltage  wristPositionVoltage = new PositionVoltage(0).withSlot(0);

  public WristIOTalonFX(){
    wristMotor = new TalonFX(wristMotorID, wristMotorCANBus);
    wristConfig = new TalonFXConfiguration();
    wristConfig.Slot0.kP = realkP;
    wristConfig.Slot0.kP = realkI;
    wristConfig.Slot0.kP = realkD;
    wristConfig.Slot0.kP = realkS;
    wristConfig.Slot0.kP = realkV;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentDuration;

    appliedVolts = wristMotor.getSupplyVoltage();
    velocityRadsPerSec = wristMotor.getVelocity();
    tempCelcius = wristMotor.getDeviceTemp();
    currentAmps = wristMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, appliedVolts, velocityRadsPerSec, tempCelcius, currentAmps);

    wristMotor.optimizeBusUtilization();

    wristMotor.getConfigurator().apply(wristConfig);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVolts, velocityRadsPerSec, tempCelcius, currentAmps);

    inputs.velocityRadsPerSecs = velocityRadsPerSec.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempCelcius = tempCelcius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  @Override
  public void setBrake(boolean brake) {
    if(brake) {
      wristMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      wristMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void goToPose(double pose){
    wristMotor.setControl(wristPositionVoltage.withFeedForward(Units.radiansToRotations(pose)));
  }

//   @Override
//   public double getAngle() {
//     return (Units.rotationsToRadians(angleRads.getValueAsDouble()));
//   }
}
