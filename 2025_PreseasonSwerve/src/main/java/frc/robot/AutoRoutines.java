package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines {
    private final Drive drive;

    public AutoRoutines(Drive drive) {
        this.drive = drive;
    }

    public Command twoNoteAuto(AutoFactory factory) {
        final AutoLoop routine = factory.newLoop("TwoNotePath Auto");
        final AutoTrajectory twoNotePath = factory.trajectory("TwoNotePath", routine);

        routine.enabled().onTrue(
            drive.runOnce(() ->
                twoNotePath.getInitialPose().ifPresentOrElse(
                    pose -> drive.setPose(pose),
                    routine::kill
                )
            )
            .andThen(twoNotePath.cmd())
        );
        return routine.cmd();
    }

    public Command simplePathAuto(AutoFactory factory) {
        final AutoLoop routine = factory.newLoop("SimplePath Auto");
        final AutoTrajectory simplePath = factory.trajectory("SimplePath", routine);

        routine.enabled().onTrue(
            drive.runOnce(() ->
                simplePath.getInitialPose().ifPresentOrElse(
                    pose -> drive.setPose(pose),
                    routine::kill
                )
            )
            .andThen(simplePath.cmd())
        );
        return routine.cmd();
    }
}
