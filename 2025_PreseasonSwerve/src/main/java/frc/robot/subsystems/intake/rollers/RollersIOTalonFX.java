package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.subsystems.intake.rollers.RollersConstants.*;

public class RollersIOTalonFX implements RollersIO {
    private TalonFX rollerMotor;
    TalonFXConfiguration rollerConfig;

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<AngularVelocity> velocityRadsPerSec;
    private double desiredVelocitySetpoint;
    private final StatusSignal<Temperature> tempCelcius;
    private final StatusSignal<Current> currentAmps;
    
    private final VelocityVoltage rollerVelocityVoltage = new VelocityVoltage(0).withSlot(1);

    public RollersIOTalonFX(){
        rollerMotor = new TalonFX(rollerMotorID, rollerMotorCANBus);
        rollerConfig = new TalonFXConfiguration();
        
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = ContinuousCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

        appliedVoltage =  rollerMotor.getSupplyVoltage();
        velocityRadsPerSec = rollerMotor.getVelocity();
        tempCelcius = rollerMotor.getDeviceTemp();
        currentAmps = rollerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);

        rollerMotor.optimizeBusUtilization();
        rollerMotor.getConfigurator().apply(rollerConfig);

    }

    @Override
    public void updateInputs(RollersIOInputs inputs){
        BaseStatusSignal.refreshAll(appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);

        inputs.velocitySetpoint = desiredVelocitySetpoint;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.velocitySetpoint = desiredVelocitySetpoint;
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.tempCelcius = tempCelcius.getValueAsDouble();
    }
    @Override
    public void setVoltage(double voltage){
        rollerMotor.setVoltage(voltage);
    }
    public void setBrake(boolean brake) {
        if (brake) {
            rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        } else {
            rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

  

    @Override
    public void setVelocity(double velocity) {
        rollerMotor.setControl(rollerVelocityVoltage.withVelocity(Units.radiansPerSecondToRotationsPerMinute(velocity)/60));
    }

    

}
