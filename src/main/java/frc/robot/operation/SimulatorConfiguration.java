package frc.robot.operation;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class SimulatorConfiguration extends AbstractCommandXboxOperationConfiguration {

    public SimulatorConfiguration(CommandXboxController controller) {
        super(controller);
    }

    @Override
    public void registerRobotFunctions(RobotContainer rc) {
        //  rc.registerSwissCheese(controller.urmom());
        // rc.registerWindexConsumption(controller.Programming());
        // rc.registerKingVon(controller.Switch());
        rc.registerTurnHopperOn(controller.a());
        rc.registerTurnHopperOff(controller.b());
        rc.registerTurnHopperReverse(controller.x());
    }

    @Override
    public void registerTeleopFunctions(RobotContainer rc) {}
}
