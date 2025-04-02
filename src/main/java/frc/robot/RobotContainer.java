// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Dislodge;
import frc.robot.Commands.DislodgerOff;
import frc.robot.Commands.ElevatorL1;
import frc.robot.Commands.ElevatorL2;
import frc.robot.Commands.ElevatorL3;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.WaitForCoral;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.PathPlanner;
import frc.robot.operation.JasonDriverConfiguration;
import frc.robot.operation.KaiOperatorConfiguration;
import frc.robot.operation.OperationConfiguration;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Dislodger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Manipulator;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {

    // private final PathPlannerAutoGenerator autoGenerator;
    // private final RobotContext robotContext;

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    private ArrayList<OperationConfiguration> operationConfigs = new ArrayList<>();

    // The robot's subsystems and commands are defined here...
    private DriveSubsystem drive;
    private Hopper hopper;
    private Manipulator manipulator;
    private Shoot shoot;
    private WaitForCoral waitForCoral;
    private Climber climber;
    private Grabber grabber;
    private Elevator elevator;
    private ElevatorL1 elevatorL1;
    private ElevatorL2 elevatorL2;
    private ElevatorL3 elevatorL3;

    private Dislodger dislodger;
    private Dislodge dislodge;
    private DislodgerOff dislodgerOff;

    private PathPlanner auto;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        drive = new DriveSubsystem.DriveSubsystemBuilder()
                .withMaxVelocities(Constants.Drive.MAXIMUM_VELOCITY, Constants.Drive.MAXIMUM_ANGULAR_VELOCITY)
                .withPointingPID(
                        Constants.Drive.ROTATION_P,
                        Constants.Drive.ROTATION_I,
                        Constants.Drive.ROTATION_D,
                        Constants.Drive.ROTATION_TARGET_RANGE)
                .withVisionProperties(Constants.Drive.MAXIMUM_VISION_POSE_OVERRIDE_DISTANCE)
                .build();

        auto = new PathPlanner(drive);

        // Configure the trigger bindings
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);
        hopper = new Hopper();
        climber = new Climber();
        grabber = new Grabber();
        elevator = new Elevator();
        elevatorL1 = new ElevatorL1(elevator);
        elevatorL2 = new ElevatorL2(elevator);
        elevatorL3 = new ElevatorL3(elevator);
        manipulator = new Manipulator(elevator);
        shoot = new Shoot(manipulator);
        waitForCoral = new WaitForCoral(manipulator);
        dislodger = new Dislodger();
        dislodge = new Dislodge(dislodger);
        dislodgerOff = new DislodgerOff(dislodger);

        NamedCommands.registerCommand("Elevator L1", elevatorL1);
        NamedCommands.registerCommand("Elevator L2", elevatorL2);
        NamedCommands.registerCommand("Elevator L3", elevatorL3);
        NamedCommands.registerCommand("Shoot", shoot);
        NamedCommands.registerCommand("Pickup Coral", waitForCoral);
        NamedCommands.registerCommand("Dislodger On", dislodge);
        NamedCommands.registerCommand("Dislodger Off ", dislodgerOff);

        initOperationConfigs();
        registerRobotFunctions();
    }

    private void initOperationConfigs() {
        operationConfigs.add(new JasonDriverConfiguration(driverController));
        operationConfigs.add(new KaiOperatorConfiguration(operatorController));
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

        Trigger manipulatorTrigger = new Trigger(() -> manipulator.manipulatorLoaded());
        registerTurnHopperAuto(manipulatorTrigger);
    }

    public void registerSwerveAngularVelocityDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                        drive.getSwerveDrive(), x, y) // Axis which give the desired translational angle and speed.
                .withControllerRotationAxis(rotation) // Axis which give the desired angular velocity.
                .deadband(Constants.Controller.DEADZONE_CONSTANT) // Controller deadband
                .scaleTranslation(Constants.Controller.SCALE_TRANSLATION) // Scaled controller translation axis
                .allianceRelativeControl(
                        false); // Alliance relative controls. Done already in the driver configuration files.
        Command driveFieldOrientedAnglularVelocity = drive.driveFieldOriented(driveAngularVelocity);
        drive.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    public void registerZeroGyro(Trigger t) {
        t.onTrue(new InstantCommand(() -> drive.zeroGyroscope(), drive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return auto.getAuto();
    }

    public void registerTurnHopperOn(Trigger t) {
        t.onTrue(new InstantCommand(() -> hopper.changeState(Hopper.HopperState.ON)));
    }

    public void registerTurnHopperAuto(Trigger t) {
        t.onTrue(new InstantCommand(() -> hopper.changeState(Hopper.HopperState.OFF)));
        t.onFalse(new InstantCommand(() -> hopper.changeState(Hopper.HopperState.ON)));
    }

    public void registerTurnHopperOff(Trigger t) {
        t.onTrue(new InstantCommand(() -> hopper.changeState(Hopper.HopperState.OFF)));
    }

    public void registerTurnHopperReverse(Trigger t) {
        t.onTrue(new InstantCommand(() -> hopper.changeState(Hopper.HopperState.REVERSE)));
    }

    public void registerShootManipulator(Trigger t) {
        t.onTrue(new InstantCommand(() -> manipulator.shoot()));
    }

    public void registerToggleGrabber(Trigger t) {
        t.onTrue(new InstantCommand(() -> grabber.toggleGrabbing()));
    }

    public void registerTurnOffWinch(Trigger t) {
        t.onTrue(new InstantCommand(() -> climber.stopWinch()));
    }

    public void registerGoToStowed(Trigger t) {
        t.onTrue(new InstantCommand(() -> climber.setTarget(Constants.Climber.CLIMBER_STOWED_ANGLE)));
        t.onTrue(new InstantCommand(() -> climber.setState(Climber.GrabState.NOTHING)));
    }

    public void registerGoToMax(Trigger t) {
        t.onTrue(new InstantCommand(() -> climber.setTarget(Constants.Climber.CLIMBER_CLIMB_ANGLE)));
        t.onTrue(new InstantCommand(() -> climber.setState(Climber.GrabState.NOTHING)));
    }

    public void registerGoToGrab(Trigger t) {
        t.onTrue(new InstantCommand(() -> climber.setTarget(Constants.Climber.CLIMBER_GRAB_ANGLE)));
        t.onTrue(new InstantCommand(() -> climber.setState(Climber.GrabState.GRABBING)));
    }

    public void registerElevatorLevelOne(Trigger t) {
        t.onTrue(new InstantCommand(() -> elevator.setTarget(Elevator.ElevatorLevel.LEVEL_1)));
    }

    public void registerElevatorLevelTwo(Trigger t) {
        t.onTrue(new InstantCommand(() -> elevator.setTarget(Elevator.ElevatorLevel.LEVEL_2)));
    }

    public void registerElevatorLevelThree(Trigger t) {
        t.onTrue(new InstantCommand(() -> elevator.setTarget(Elevator.ElevatorLevel.LEVEL_3)));
    }

    public void registerShootThenHome(Trigger t) {
        t.onTrue(new InstantCommand(() -> manipulator.shoot())
                .andThen(() -> elevator.setTarget(Elevator.ElevatorLevel.LEVEL_1)));
    }

    public void registerToggleDislodger(Trigger t) {
        t.onTrue(new InstantCommand(() -> dislodger.toggleOnOff()));
    }

    public void registerDeployWinch(Trigger t) {
        t.onTrue(new InstantCommand(() -> climber.winchOut()));
        t.onFalse(new InstantCommand(() -> climber.stopWinch()));
    }

    public void registerRetractWinch(Trigger t) {
        t.onTrue(new InstantCommand(() -> climber.winchIn()));
        t.onFalse(new InstantCommand(() -> climber.stopWinch()));
    }
}
