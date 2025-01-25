// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.operation.OperationConfiguration;
import frc.robot.operation.SimulatorConfiguration;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import java.util.ArrayList;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // private final PathPlannerAutoGenerator autoGenerator;
    // private final RobotContext robotContext;

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final Hopper hopper;

    private ArrayList<OperationConfiguration> operationConfigs = new ArrayList<>();

    // The robot's subsystems and commands are defined here...

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);
        initOperationConfigs();
        registerRobotFunctions();
        hopper = new Hopper();
    }

    private void initOperationConfigs() {
        operationConfigs.add(new SimulatorConfiguration(driverController));
    }

    private void registerRobotFunctions() {
        for (OperationConfiguration opConfig : operationConfigs) {
            opConfig.registerRobotFunctions(this);
        }
    }

    public void teleopInit() {

        for (OperationConfiguration opConfig : operationConfigs) {
            opConfig.registerTeleopFunctions(this);
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }

    public void registerTurnHopperOn(Trigger t) {
        t.onTrue(new InstantCommand(() -> hopper.changeState(HopperState.ON)));
    }

    public void registerTurnHopperOff(Trigger t) {
        t.onTrue(new InstantCommand(() -> hopper.changeState(HopperState.OFF)));
    }

    public void registerTurnHopperReverse(Trigger t) {
        t.onTrue(new InstantCommand(() -> hopper.changeState(HopperState.REVERSE)));
    }
}
