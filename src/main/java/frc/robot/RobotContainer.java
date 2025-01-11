// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import frc.robot.operation.JasonDriverConfiguration;
import frc.robot.operation.OperationConfiguration;
//import frc.robot.operation.Operator;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;


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

    private ArrayList<OperationConfiguration> operationConfigs = new ArrayList<>();

    private boolean shotTesting = false;
    private ShuffleboardTab testOnlyTab;
    private GenericEntry testAngleEntry;
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        driverController = new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER);
        operatorController = new CommandXboxController(Constants.Controller.OPERATOR_CONTROLLER);
        initOperationConfigs();
        registerRobotFunctions();

        
    }
    

   

    private void initOperationConfigs(){
        operationConfigs.add(new JasonDriverConfiguration(driverController));
    }

    private void registerRobotFunctions(){
        for(OperationConfiguration opConfig : operationConfigs){
            opConfig.registerRobotFunctions(this);
        }
    }

    public void teleopInit(){


    //    for (OperationConfiguration opConfig : operationConfig){
    //     opConfig.registerTeleopFunctions(this);
    //    }

    //    @Override
    //    public void registerTeleopFunctions(trigger t){
       
    //    }
        
    }

    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(m_exampleSubsystem);
    }


}
