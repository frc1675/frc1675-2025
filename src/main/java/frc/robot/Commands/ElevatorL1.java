package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevel;

public class ElevatorL1 extends Command {
    private Elevator elevator;

    public ElevatorL1(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setTarget(Elevator.ElevatorLevel.LEVEL_1);
    }

    @Override
    public boolean isFinished() {
        return elevator.getLevel() == ElevatorLevel.LEVEL_1;
    }
}
