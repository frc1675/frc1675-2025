package frc.robot.operation;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class KaiDriverConfiguration extends AbstractCommandXboxOperationConfiguration {

    public KaiDriverConfiguration(CommandXboxController controller) {
        super(controller);
    }

    @Override
    public void registerRobotFunctions(RobotContainer rc) {
        rc.registerZeroGyro(controller.start());
        rc.registerTurnHopperAuto(controller.a());
        rc.registerTurnHopperReverse(controller.b());
        rc.registerTurnHopperOff(controller.x());
        rc.registerShootManipulator(controller.rightTrigger());
        rc.registerDeployWinch(controller.leftTrigger());
        rc.registerRetractWinch(controller.leftBumper());
        rc.registerTurnOffWinch(controller.rightBumper());
    }

    @Override
    public void registerTeleopFunctions(RobotContainer rc) {}
}
