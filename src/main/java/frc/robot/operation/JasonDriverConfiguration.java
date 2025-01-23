package frc.robot.operation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class JasonDriverConfiguration extends AbstractCommandXboxOperationConfiguration {

    public JasonDriverConfiguration(CommandXboxController controller) {
        super(controller);
    }

    @Override
    public void registerRobotFunctions(RobotContainer rc) {
        rc.registerZeroGyro(controller.start());
    }

    @Override
    public void registerTeleopFunctions(RobotContainer rc) {
        rc.registerDefaultDrive(
                () -> -1 * getJoystickInput(controller, Constants.Controller.LEFT_Y_AXIS),
                () -> -1 * getJoystickInput(controller, Constants.Controller.LEFT_X_AXIS),
                () -> getJoystickInput(controller, Constants.Controller.RIGHT_X_AXIS));
    }

    private double getJoystickInput(CommandGenericHID stick, int axe) {
        return MathUtil.applyDeadband(stick.getRawAxis(axe), Constants.Controller.DEADZONE_CONSTANT);
    }
}
