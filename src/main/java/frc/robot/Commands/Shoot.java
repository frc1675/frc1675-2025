package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorState;

public class Shoot extends Command {
    private Manipulator manipulator;

    public Shoot(Manipulator manipulator) {
        this.manipulator = manipulator;
        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        manipulator.shoot();
    }

    @Override
    public boolean isFinished() {
        return manipulator.getState() == ManipulatorState.EMPTY;
    }
}
