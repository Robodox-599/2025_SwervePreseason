package frc.robot.subsystems.Indexer;

import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOTalonFX implements IndexerIO {
    private TalonFX indexerMotor;
    TalonFXConfiguration indexerConfig;

    private final StatusSignal<AngularVelocity> velocityRadsPerSec;
    private final StatusSignal<Voltage> appliedVoltage;
    private double desiredSpeedSetpoint;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> tempCelcius;
    private final PositionVoltage indexerPositionVoltage = new PositionVoltage(0). withSlot(0);
    private final VelocityVoltage indexerVelocityVoltage = new VelocityVoltage(0). withSlot(0);

    public IndexerIOTalonFX() {
        indexerMotor = new TalonFX(indexerMotorID, indexerMotorCANBus);
        indexerConfig = new TalonFXConfiguration();
        indexerConfig.Slot0.kP = realkP;
        indexerConfig.Slot0.kP = realkI;
        indexerConfig.Slot0.kP = realkD;
        indexerConfig.Slot0.kP = realkS;
        indexerConfig.Slot0.kP = realkV;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

        appliedVoltage = indexerMotor.getSupplyVoltage();
        velocityRadsPerSec = indexerMotor.getVelocity();
        tempCelcius = indexerMotor.getDeviceTemp();
        currentAmps = indexerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);

        indexerMotor.optimizeBusUtilization();
        indexerMotor.getConfigurator().apply(indexerConfig);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);

        inputs.speedSetpoint = desiredSpeedSetpoint;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.tempCelcius = tempCelcius.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.velocityRadsPerSec = velocityRadsPerSec.getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
    }

    @Override
    public void setBrake(boolean brake) {
        if (brake) {
             indexerMotor.setNeutralMode(NeutralModeValue.Brake);   
        } else {
            indexerMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    @Override
    public void goToPose(double pose) {
        indexerMotor.setControl(indexerPositionVoltage.withPosition(Units.radiansToRotations    (pose)));
    }
    public void setVelocity(double velocity) {
        indexerMotor.setControl(indexerVelocityVoltage.withVelocity(Units.radiansPerSecondToRotationsPerMinute(velocity)/60));
    }
}
