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
public class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

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

    public static class Auto {
        // 0 is a placeholder until best values are found
        public static final double TRANSLATION_P = 6;
        public static final double ROTATION_P = 6;
    }

    public static class Controller {
        public static final double DEADZONE_CONSTANT = 0.1675;
        public static final int LEFT_X_AXIS = 0;
        public static final int LEFT_Y_AXIS = 1;
        public static final int RIGHT_X_AXIS = 4;

        public static final int SCALE_TRANSLATION = 1;
    }

    public static class Climber {
        public static final int ENCODER_CHANNEL = 0;

        public static final int CLIMB_MOTOR = 12;
        // This motor has a current limit of 30A set in the REV firmware

        public static final double OUT_WINCH_SPEED = 0.85;
        public static final double IN_WINCH_SPEED = -0.85;

        public static final double CLIMBER_STOWED_ANGLE = 136;
        public static final double CLIMBER_CLIMB_ANGLE = 159; // Make sure this is Max angle, not test
        public static final double CLIMBER_GRAB_ANGLE = 290; //

        public static final double CLIMBER_NEO_STOWED_ANGLE = 9;

        public static final double CLIMBER_NEO_CLIMB_ANGLE = 159; // Make sure this is Max angle, not test

        public static final double CLIMBER_NEO_GRAB_ANGLE = -219;
    }

    public static class Grabber {
        public static final int LEFT_PULLER_MOTOR = 13;
        public static final int RIGHT_PULLER_MOTOR = 14;
        public static final int CAGE_SENSOR = 32;

        public static final double LEFT_PULLER_MOTOR_SPEED = 0.5;
        public static final double RIGHT_PULLER_MOTOR_SPEED = -0.5;
    }

    public static class Hopper {
        public static final int HOPPER_MOTOR = 9;

        public static final double HOPPER_INTAKE_SPEED = 1;
        public static final double HOPPER_REVERSE_SPEED = -1;
    }

    public static class Manipulator {
        public static final int MANIPULATOR_MOTOR_1 = 10; // bottom
        public static final int MANIPULATOR_MOTOR_2 = 11; // top
        public static final int CORAL_SENSOR = 31;

        public static final double DELAY = 1.5;
        public static final double CHECK_DELAY = 0.5;

        public static final double TOP_SHOOTING_SPEED = 0.1;
        public static final double BOTTOM_SHOOTING_SPEED = 0.2;

        public static final double HOME_TOP_SHOOTING_SPEED = 0.05;
        public static final double HOME_BOTTOM_SHOOTING_SPEED = 0.1;

        public static final double TOP_INTAKE_SPEED = 0.10;
        public static final double BOTTOM_INTAKE_SPEED = 0.10;
        public static final double MAX_VOLTAGE = 12.0;

        public static final double DETECTION_RANGE = 85; // mm
    }

    public static class Elevator {
        public static final int ELEVATOR_MOTOR = 15;
        public static final int ENCODER_A = 3;
        public static final int ENCODER_B = 4;
        public static final int ELEVATOR_HOMESWITCH = 2;

        // elevator power/voltage to go up and down
        public static final double ELEVATOR_MANUAL_UP = 0.1; // positive is up
        public static final double ELEVATOR_MANUAL_DOWN = -0.1;

        // max voltage that can be provided to the robot
        public static final double MAX_VOLTAGE = 12.0;

        public static final double ELEVATOR_MAX_ACCELERATION = 4800; // Highest amount of degrees per second square
        public static final double ELEVATOR_MAX_VELOCITY = 3500; // Degrees per second

        public static final double ELEVATOR_PID_P_COEFFICIENT = 0.002;
        public static final double ELEVATOR_PID_I_COEFFICIENT = 0;
        public static final double ELEVATOR_PID_D_COEFFICIENT = 0;

        public static final double LEVEL_ONE_ANGLE = 30;
        public static final double LEVEL_TWO_ANGLE = 462;
        public static final double LEVEL_THREE_ANGLE = 1484; // 414
        public static final double MAX_LIMIT = 1790.2; // 5.530 inches per revolution
    }

    public static class Dislodger {
        public static final int DISLODGER_MOTOR = 16;
        public static final double DISLODGER_MOTOR_SPEED = 1;
    }
}
