package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

public class WaitForCoral extends Command {
    private Manipulator manipulator;

    public WaitForCoral(Manipulator manipulator) {
        this.manipulator = manipulator;
        addRequirements(manipulator);
    }

    @Override
    public boolean isFinished() {
        return manipulator.manipulatorLoaded();
    }
}
