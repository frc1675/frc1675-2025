package frc.robot.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.brownbox.util.AllianceUtil;

public class PathPlanner {
    private final SendableChooser<Command> autoChooser;

    public PathPlanner(DriveSubsystem drive) {

        RobotConfig config = null;

        autoChooser = AutoBuilder.buildAutoChooser();
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        ShuffleboardTab dashboard = Shuffleboard.getTab("Auto");
        dashboard.add("AutoChooser", autoChooser);

        AutoBuilder.configure(
                drive::getPose,
                drive::resetOdometry,
                drive::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> drive.setRobotRelativeChassisSpeeds(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(Constants.Auto.TRANSLATION_P), new PIDConstants(Constants.Auto.ROTATION_P)),
                config,
                AllianceUtil::isRedAlliance,
                drive);
    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }
}
