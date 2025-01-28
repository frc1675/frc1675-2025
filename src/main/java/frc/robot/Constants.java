// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public final static boolean LIVE_PID = true;

    public static class Drive {
        public static final double MAXIMUM_VELOCITY = 5.5; // meters per second
        public static final double MAXIMUM_ANGULAR_VELOCITY = 8; // radians per second

        public static final double AUTONOMOUS_VELOCITY = MAXIMUM_VELOCITY; // meters per second
        public static final double AUTONOMOUS_ACCELERATION = 10.0; // meters per second squared

        public static final double MAXIMUM_VISION_POSE_OVERRIDE_DISTANCE = 1.0; // meters

        public static final double DRIVE_GEAR_RATIO = 6.12;
        public static final double STEER_GEAR_RATIO = 12.8;
        public static final double PULSE_PER_ROTATION = 1;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

        public static final double SLOW_DRIVE_SCALE = 0.5;

        public static final double ROTATION_P = 0.05;
        public static final double ROTATION_I = 0.001;
        public static final double ROTATION_D = 0;
        public static final double ROTATION_TARGET_RANGE = 1.5;
    }

    public static class Controller {
        public static final double DEADZONE_CONSTANT = 0.1675;
        public static final int LEFT_X_AXIS = 0;
        public static final int LEFT_Y_AXIS = 1;
        public static final int RIGHT_X_AXIS = 4;
    }
}
