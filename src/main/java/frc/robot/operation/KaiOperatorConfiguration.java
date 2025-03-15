package frc.robot.operation;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class KaiOperatorConfiguration extends AbstractCommandXboxOperationConfiguration {

    public KaiOperatorConfiguration(CommandXboxController controller) {
        super(controller);
    }

    @Override
    public void registerRobotFunctions(RobotContainer rc) {

        rc.registerElevatorLevelOne(controller.a());
        rc.registerElevatorLevelTwo(controller.b());
        rc.registerElevatorLevelThree(controller.y());
        rc.registerElevatorDown(controller.rightBumper());
        rc.registerElevatorUp(controller.leftBumper());
    }

    @Override
    public void registerTeleopFunctions(RobotContainer rc) {
        // rc.toggleHopperState(controller.a());
        // rc.toggleSpitCoral(controller.b());
        // rc.shoot(contoller.rightTrigger());
        // rc.climbe(controller.leftTrigger());
        // rc.align(controller.rightBumper());

    }
}
