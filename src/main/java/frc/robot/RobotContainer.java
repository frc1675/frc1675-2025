// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drive.DefaultDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.operation.JasonDriverConfiguration;
import frc.robot.operation.OperationConfiguration;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

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
    // private final Hopper hopper;

    private ArrayList<OperationConfiguration> operationConfigs = new ArrayList<>();

    // The robot's subsystems and commands are defined here...
    private DriveSubsystem drive;

    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser();

       

        drive = new DriveSubsystem.DriveSubsystemBuilder()
                .withMaxVelocities(Constants.Drive.MAXIMUM_VELOCITY, Constants.Drive.MAXIMUM_ANGULAR_VELOCITY)
                .withPointingPID(
                        Constants.Drive.ROTATION_P,
                        Constants.Drive.ROTATION_I,
                        Constants.Drive.ROTATION_D,
                        Constants.Drive.ROTATION_TARGET_RANGE)
                .withVisionProperties(Constants.Drive.MAXIMUM_VISION_POSE_OVERRIDE_DISTANCE)
                .build();

        // Configure the trigger bindings
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);
        initOperationConfigs();
        registerRobotFunctions();
        // hopper = new Hopper();

        NamedCommands.registerCommand("thoseWhoKnow", getAutonomousCommand());


    }

    private void initOperationConfigs() {
        operationConfigs.add(new JasonDriverConfiguration(driverController));
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

    public void registerDefaultDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        drive.setDefaultCommand(new DefaultDrive(drive, x, y, rotation, () -> 1.0));
    }

    public void registerZeroGyro(Trigger t) {
        t.onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));
    }

  
 

    public void registerTurnHopperOn(Trigger t) {
        // t.onTrue(new InstantCommand(() -> hopper.changeState(HopperState.ON)));
    }

    public void registerTurnHopperOff(Trigger t) {
        // t.onTrue(new InstantCommand(() -> hopper.changeState(HopperState.OFF)));
    }

    public void registerTurnHopperReverse(Trigger t) {
        // t.onTrue(new InstantCommand(() -> hopper.changeState(HopperState.REVERSE)));
    }

  /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    //use after configuring autobuilder
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Test");
    }

     public Command getAutoPathFollowerCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

    
}
