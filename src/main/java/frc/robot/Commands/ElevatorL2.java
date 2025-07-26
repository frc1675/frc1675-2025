package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevel;

public class ElevatorL2 extends Command {
    private Elevator elevator;

    public ElevatorL2(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setTarget(Elevator.ElevatorLevel.LEVEL_2);
    }

    @Override
    public boolean isFinished() {
        return elevator.getLevel() == ElevatorLevel.LEVEL_2;
    }
}
