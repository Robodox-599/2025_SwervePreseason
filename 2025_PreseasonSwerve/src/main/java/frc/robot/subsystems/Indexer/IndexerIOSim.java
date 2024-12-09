package frc.robot.subsystems.Indexer;

import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class IndexerIOSim implements IndexerIO {
    // private final driveSim =
    // new DCMotorSim(
    //     .createDCMotorSystem(
    //         DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
    //     DRIVE_GEARBOX);
    private static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private final DCMotorSim indexerMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(INDEXER_GEARBOX, indexerMOI, gearRatio),
        INDEXER_GEARBOX); 
    private PIDController controller = new PIDController(0, indexerMotorID, indexerMotorID);
    private SimpleMotorFeedforward ff;
    private double appliedVoltage;
    private double desiredSpeed;

    public IndexerIOSim() {
        controller.setPID(simkP, simkI, simkD);
        ff = new SimpleMotorFeedforward(simkS, simkV);
    } 

    @Override 
    public void updateInputs(IndexerIOInputs inputs) {
        indexerMotorSim.update(0.02);
        inputs.velocityRadsPerSec = indexerMotorSim.getAngularAccelerationRadPerSecSq();
        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmps = indexerMotorSim.getCurrentDrawAmps();
        inputs.tempCelcius = 60;
        inputs.speedSetpoint = desiredSpeed;
    }
    @Override 
    public void setVoltage(double volts) {
        appliedVoltage = volts;
        indexerMotorSim.setInputVoltage(volts);
    }
    @Override 
    public void goToPose(double pose) {
        double volts = controller.calculate(indexerMotorSim.getAngularPositionRad(), pose) + ff.calculate(simVelocityConstant);
        indexerMotorSim.setInputVoltage(volts);
    }
    @Override 
    public void setVelocity(double velocity) {
        double volts = controller.calculate(indexerMotorSim.getAngularVelocityRadPerSec(), velocity);
    }

}
