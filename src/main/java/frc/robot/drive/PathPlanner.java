package frc.robot.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class PathPlanner {
    // private final SendableChooser<Command> autoChooser;
    private ShuffleboardTab dashboard;

    public PathPlanner(DriveSubsystem drive) {

        RobotConfig config = null;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
                drive::getPose,
                drive::resetOdometry,
                drive::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> drive.setRobotRelativeChassisSpeeds(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(Constants.Auto.TRANSLATION_P), new PIDConstants(Constants.Auto.ROTATION_P)),
                config,
                () -> true,
                drive);

        //    autoChooser = AutoBuilder.buildAutoChooser();

        //    initShuffleboard();
    }

    // public Command getAuto() {
    //     return autoChooser.getSelected();
    // }

    // private void initShuffleboard() {
    //     dashboard = Shuffleboard.getTab("Auto");

    //     dashboard.add("Auto Chooser", autoChooser);
    // }

    private void registerNamedCommand() {}
}
