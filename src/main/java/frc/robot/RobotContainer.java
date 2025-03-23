// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private Climber climber;
    private Grabber grabber;
    private Elevator elevator;
    private Dislodger dislodger;
    // private PathPlannerAuto auto;
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

        NamedCommands.registerCommand("shoot", Commands.run(manipulator::shoot, manipulator));
        climber = new Climber();
        grabber = new Grabber();
        elevator = new Elevator();
        manipulator = new Manipulator(elevator);
        dislodger = new Dislodger();

        initOperationConfigs();
        registerRobotFunctions();

        // register Named Commands
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

    // public Command loadAutos() {
    //     return new PathPlannerAuto("Example Auto");
    // }

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
                        true); // Alliance relative controls. Done already in the driver configuration files.
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
        // return new PathPlannerAuto("Strait Auto");
        return (new StartEndCommand(
                        //         () -> {
                        //             if (AllianceUtil.isRedAlliance()) {
                        //                 drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.PI)));
                        //             }
                        //             drive.drive(.25, 0, 0);
                        //         },
                        //         () -> {
                        //             drive.drive(0, 0, 0);

                        //         }))
                        // .withTimeout(1.0);

                        () -> {
                            drive.drive(.25, 0, 0);
                        },
                        () -> {
                            drive.drive(0, 0, 0);
                        }))
                .withTimeout(1.4)
                .andThen(new WaitCommand(2))
                .andThen(new InstantCommand(() -> manipulator.shoot()));

        // return new PathPlannerAuto("Strait Auto");
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
        t.onTrue(new InstantCommand(() -> manipulator.shoot()));
        t.onTrue(new InstantCommand(() -> elevator.setTarget(Elevator.ElevatorLevel.LEVEL_1)));
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
