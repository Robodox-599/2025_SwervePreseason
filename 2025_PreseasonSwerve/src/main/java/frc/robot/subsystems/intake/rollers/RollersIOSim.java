package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import static frc.robot.subsystems.intake.rollers.RollersConstants.*;

public class RollersIOSim implements RollersIO{
    private final DCMotor rollerGearbox = DCMotor.getKrakenX60Foc(1);
    private PIDController controller = new PIDController(1, 0, 0);
    private SimpleMotorFeedforward ff;
   // private final DCMotorSim rollerMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), DCMotorSim(DCMotor.getKrakenX60(1), gearRatio, rollerMOI));
    private final DCMotorSim rollerMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(rollerGearbox, rollerMOI, gearRatio), rollerGearbox);
    
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
            double volts = controller.calculate(rollerMotorSim.getAngularVelocityRadPerSec(), velocity) + ff.calculate(simVelocityConstant); //fix
            rollerMotorSim.setInputVoltage(volts);

        }
}