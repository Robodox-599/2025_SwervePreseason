package frc.robot.subsystems.intake.rollers;

public class RollersConstants {

    public static final int rollerMotorID = 0;
    public static final String rollerMotorCANBus = "rio";

    static final double gearRatio = 1.5;
    static final double rollerMOI = 0.04;


    public static final boolean EnableCurrentLimit = true;
    public static final int ContinuousCurrentLimit = 50;
    public static final int PeakCurrentLimit = 50;
    public static final double PeakCurrentDuration = 0.1;

    public static final double simVelocityConstant = 0.2;

}
