// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drive.DriveSubsystem;
import frc.robot.operation.JasonDriverConfiguration;
import frc.robot.operation.OperationConfiguration;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

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
    private Hopper hopper;

    private ArrayList<OperationConfiguration> operationConfigs = new ArrayList<>();

    // The robot's subsystems and commands are defined here...
    private DriveSubsystem drive;

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

        // Configure the trigger bindings
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);
        initOperationConfigs();
        registerRobotFunctions();
        // hopper = new Hopper();
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

    public void registerSwervePointDrive(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier headingX, DoubleSupplier headingY) {
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                        drive.getSwerveDrive(), x, y) // Axis which give the desired translational angle and speed.
                .deadband(Constants.Controller.DEADZONE_CONSTANT) // Controller deadband
                .scaleTranslation(Constants.Controller.SCALE_TRANSLATION) // Scaled controller translation axis
                .allianceRelativeControl(
                        false); // Alliance relative controls. Done already in the driver configuration files.

        SwerveInputStream driveDirectAngle = driveAngularVelocity
                .copy() // Copy the stream so further changes do not affect driveAngularVelocity
                .withControllerHeadingAxis(
                        headingX, headingY) // Axis which give the desired heading angle using trigonometry.
                .headingWhile(true); // Enable heading based control.
        Command driveFieldOrientedDirectAngle = drive.driveFieldOriented(driveDirectAngle);
        drive.setDefaultCommand(driveFieldOrientedDirectAngle);
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
