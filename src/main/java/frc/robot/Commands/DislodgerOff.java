package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dislodger;

public class DislodgerOff extends Command {
    private Dislodger dislodger;

    public DislodgerOff(Dislodger dislodger) {
        this.dislodger = dislodger;
        addRequirements(dislodger);
    }

    @Override
    public void execute() {
        dislodger.dislodgerOff();
    }

    @Override
    public boolean isFinished() {
        return dislodger.isOn == false;
    }
}
