// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private SparkMax winchMotor;
    private SparkMax cagePullerMotorLeft;
    private SparkMax cagePullerMotorRight;

    // private LaserCan lasercan
    // private RelativeEncoder climbEncoder;

    private int CLIMB_MOTOR;
    private int LEFT_PULLER_MOTOR;
    private int RIGHT_PULLER_MOTOR;
    private double WINCH_MOTOR_SPEED;
    private double PULLER_MOTOR_SPEED;

    public boolean cageDetected;
    private ClimberState climberState;

    /** Creates a new Climber. */
    public Climber() {
        climberState = ClimberState.WAITING;
        winchMotor = new SparkMax(CLIMB_MOTOR, MotorType.kBrushless);
        cagePullerMotorLeft = new SparkMax(LEFT_PULLER_MOTOR, MotorType.kBrushless);
        cagePullerMotorRight = new SparkMax(RIGHT_PULLER_MOTOR, MotorType.kBrushless);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        /*
         * Starts in a Waiting state
         * Driver drives into cage
         * Cage goes into Activated state
         *
         */
        if (climberState == ClimberState.WAITING) {}

        if (climberState == ClimberState.GRABBING) {
            cagePullerMotorLeft.setVoltage(12 * PULLER_MOTOR_SPEED);
            cagePullerMotorLeft.setVoltage(12 * PULLER_MOTOR_SPEED);
        }

        if (climberState == ClimberState.WINCHING) {
            winchMotor.setVoltage(12 * WINCH_MOTOR_SPEED);
        }
    }

    enum ClimberState {
        WAITING,
        GRABBING,
        WINCHING,
        DONE
    }

    public double getCurrentAngle() {
        return 1;
    }

    public double getTarget() {
        return 1;
    }

    public void setTarget(double angle) {}

    public boolean isCageDetected() {

        return false;
    }

    public ClimberState getState() {
        return climberState;
    }
}
