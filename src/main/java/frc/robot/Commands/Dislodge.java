package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dislodger;

public class Dislodge extends Command {
    private Dislodger dislodger;

    public Dislodge(Dislodger dislodger) {
        this.dislodger = dislodger;
        addRequirements(dislodger);
    }

    @Override
    public void execute() {
        dislodger.dislodgerOn();
    }

    @Override
    public boolean isFinished() {
        return dislodger.isOn == true;
    }
}
