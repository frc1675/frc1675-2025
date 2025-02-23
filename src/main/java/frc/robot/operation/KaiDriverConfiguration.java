package frc.robot.operation;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class KaiDriverConfiguration extends AbstractCommandXboxOperationConfiguration {

    public KaiDriverConfiguration(CommandXboxController controller) {
        super(controller);
    }

    @Override
    public void registerRobotFunctions(RobotContainer rc) {
        //  rc.registerSwissCheese(controller.urmom());
        // rc.registerWindexConsumption(controller.Programming());
        // rc.registerKingVon(controller.Switch());
    }

    @Override
    public void registerTeleopFunctions(RobotContainer rc) {}
}
