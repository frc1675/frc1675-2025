package frc.robot.operation;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class represents a operation configuration that uses an xbox controller
 */
public abstract class AbstractCommandXboxOperationConfiguration implements OperationConfiguration {

    protected CommandXboxController controller;

    public AbstractCommandXboxOperationConfiguration(CommandXboxController controller) {
        this.controller = controller;
    }
}
