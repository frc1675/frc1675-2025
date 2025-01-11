package frc.robot.operation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.util.AllianceUtil;

public class JasonDriverConfiguration extends AbstractCommandXboxOperationConfiguration{

    public JasonDriverConfiguration (CommandXboxController controller){
        super(controller);
    }
    @Override
    public void registerRobotFunctions(RobotContainer rc){
      //  rc.registerSwissCheese(controller.urmom());
      // rc.registerWindexConsumption(controller.Programming());
      // rc.registerKingVon(controller.Switch());
    }

    @Override
    public void registerTeleopFunctions(RobotContainer rc){
       
    
    }

    private double getJoystickInput(CommandGenericHID stick, int axe){
        return MathUtil.applyDeadband(stick.getRawAxis(axe), Constants.Controller.DEADZONE_CONSTANT);
    }
}
