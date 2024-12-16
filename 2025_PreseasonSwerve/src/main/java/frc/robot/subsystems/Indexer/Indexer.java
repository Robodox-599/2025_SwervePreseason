package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
  IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Indexer", inputs);

    io.beamBreakPeriodic();
  }

  @AutoLogOutput(key = "Indexer/IndexerAppliedVoltage")
  public double getVoltage() {
    return inputs.appliedVoltage;
  }

  public Command applyVoltage(double voltage) {
    return Commands.run(
        () -> {
          io.setVoltage(voltage);
        });
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setVoltage(0);
        });
  }

  public Command setVelocity(double velocity) {
    return Commands.run(
        () -> {
          io.setVelocity(velocity);
        }
        );
  }
  public Command goToPose(double pose){
    return Commands.run(
        () -> {
          io.goToPose(pose);
        });
  }
  public void setBrake(boolean brake){
    io.setBrake(brake);
  }
  public Command indexerTillBeamBreak(){
	return Commands.sequence(
	new InstantCommand(()-> io.runVelocity(IndexerConstants.intakeVelocity)),
	new WaitUntilCommand(()-> io.noteInIndexer()),
	new InstantCommand(() -> io.runVelocity(0)));
}
}
