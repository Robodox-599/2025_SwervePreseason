package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Rollers extends SubsystemBase {
    private final RollersIO io;
    RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

    public Rollers(RollersIO io){
        this.io = io;
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Rollers", inputs);
        // idk sos
        

    }
    @AutoLogOutput(key = "Rollers/RollersAppliedVoltage")
    public double getVoltage(){
        return inputs.appliedVoltage;
    }
    public Command applyVoltage(double voltage){
        return Commands.run(
            ()-> {
                io.setVoltage(voltage);
            }
        );
    }
    public Command stop(){
        return Commands.run(
            ()-> {
                io.setVoltage(0);
            }
        );
    }
    public Command setVelocity(double velocity){
        return Commands.run(
            ()-> {
                io.setVelocity(velocity);
            }
        );
    }
    
    public Command setBrake(boolean brake){
         return Commands.run(
            ()-> {
                io.setBrake(brake);
            }
        );
        
    }   
}
