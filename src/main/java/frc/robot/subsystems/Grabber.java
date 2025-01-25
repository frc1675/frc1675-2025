// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
    private SparkMax cagePullerMotorLeft;
    private SparkMax cagePullerMotorRight;

    // private LaserCan lasercan

    private int LEFT_PULLER_MOTOR;
    private int RIGHT_PULLER_MOTOR;

    private double PULLER_MOTOR_SPEED;

    private GrabberState grabberState;

    /** Creates a new Grabber. */
    public Grabber() {
        grabberState = GrabberState.WAITING;
        cagePullerMotorLeft = new SparkMax(LEFT_PULLER_MOTOR, MotorType.kBrushless);
        cagePullerMotorRight = new SparkMax(RIGHT_PULLER_MOTOR, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (cageDetected() && grabberState == GrabberState.GRABBING) {
            grabberState = GrabberState.DONE;
        }
        if (grabberState == GrabberState.GRABBING) {
            cagePullerMotorLeft.setVoltage(PULLER_MOTOR_SPEED * 12);
            cagePullerMotorRight.setVoltage(PULLER_MOTOR_SPEED * 12);
        }

        if (grabberState == GrabberState.DONE) {
            cagePullerMotorLeft.setVoltage(0);
            cagePullerMotorRight.setVoltage(0);
        }
    }

    enum GrabberState {
        WAITING,
        GRABBING,
        DONE
    }

    public boolean cageDetected() {
        // detect lasercan value, if certain value, the cage is detected
        return false;
    }

    public void toggleGrabbing() {
        if (grabberState == GrabberState.GRABBING) {
            grabberState = GrabberState.WAITING;
        } else {
            grabberState = GrabberState.GRABBING;
        }
    }
}
