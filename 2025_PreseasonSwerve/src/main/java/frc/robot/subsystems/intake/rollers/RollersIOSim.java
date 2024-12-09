package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.robot.subsystems.intake.rollers.RollersConstants.*;
public class RollersIOSim implements RollersIO {
   // private final DCMotorSim rollerMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), DCMotorSim(DCMotor.getKrakenX60(1), gearRatio, rollerMOI));
    private final DCMotorSim rollerMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), gearRatio, rollerMOI);
    
    private double appliedVoltage;
    private double desiredVelocity;

    public RollersIOSim() {
     
    }
    @Override
    public void updateInputs(RollersIOInputs inputs) {
        rollerMotorSim.update(0.02);
        inputs.velocityRadsPerSec = rollerMotorSim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmps = rollerMotorSim.getCurrentDrawAmps();
        inputs.tempCelcius = 60;
        inputs.velocitySetpoint = desiredVelocity;
    }
    @Override
    public void setVoltage(double volts){
        appliedVoltage = volts;
        rollerMotorSim.setInputVoltage(volts);
    }
   
        @Override
        public void setVelocity(double velocity){
            desiredVelocity = velocity;

            

        }
}