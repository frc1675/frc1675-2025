package frc.robot.operation;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class WillOperatorConfiguration extends AbstractCommandXboxOperationConfiguration {

    public WillOperatorConfiguration(CommandXboxController controller) {
        super(controller);
    }

    @Override
    public void registerRobotFunctions(RobotContainer rc) {}

    @Override
    public void registerTeleopFunctions(RobotContainer rc) {
        // none
    }
}
