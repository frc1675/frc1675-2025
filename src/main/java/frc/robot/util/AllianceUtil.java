package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;

/**
 * Convenience methods based on the current alliance.
 */
public class AllianceUtil {

    /**
     * Determines whether or not the robot is on the red alliance.
     * In simulation the robot is always Blue alliance by default.
     * If no alliance could be found from the field, the robot reverts to blue alliance.
     * @return Whether or not the robot is on the red alliance.
     */
    public static boolean isRedAlliance() {
        if (Robot.isSimulation()) {
            return false;
        }

        if (DriverStation.getAlliance().isEmpty()) {
            // TODO add an alert to the dashboard that this happened.
            return false;
        }

        return DriverStation.getAlliance().get().equals(Alliance.Red);
    }

    /**
     * Maintains field-centric control based on driver station.
     * "Away" for blue (towards red) is opposite "away" for red (towards blue).
     * "Left/Right" for blue (towards or away from scoring table) is opposite "left/right" for red.
     * Multiply by this value whenever you calculate a translation component in teleop.
     * @return Scalar (1 or -1) to multiply robot translation inputs by based on what alliance you are on.
     */
    public static int getTranslationDirection() {
        return isRedAlliance() ? 1 : -1;
    }
}
