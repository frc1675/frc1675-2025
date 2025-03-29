// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Climber extends SubsystemBase {

    @NotLogged
    private double targetAngle;

    @NotLogged
    private SparkMax winchMotor;

    @NotLogged
    private DutyCycleEncoder climberEncoder;

    @NotLogged
    private ProfiledPIDController retractWinchPID;

    @NotLogged
    private TrapezoidProfile.Constraints retractConstraints;

    @NotLogged
    private ProfiledPIDController deployWinchPID;

    @NotLogged
    private TrapezoidProfile.Constraints deployConstraints;

    public GrabState currentGrabState;

    /** Creates a new Climber. */
    public Climber() {
        winchMotor = new SparkMax(Constants.Climber.CLIMB_MOTOR, MotorType.kBrushless);
        climberEncoder = new DutyCycleEncoder(Constants.Climber.ENCODER_CHANNEL);
        setTarget(Constants.Climber.CLIMBER_STOWED_ANGLE);
        currentGrabState = GrabState.NOTHING;
    }

    public enum GrabState {
        NOTHING,
        GRABBING,
        GRABBED
    }

    @Override
    public void periodic() {
        goToSetTarget();
    }

    @Logged
    public double getNeoAngle() {
        return -1 * winchMotor.getEncoder().getPosition();
    }

    @Logged
    public double getCurrentAngle() {
        return climberEncoder.get() * 360;
    }

    @Logged
    public double getTarget() {
        return targetAngle;
    }

    public void setTarget(double angle) {
        targetAngle = angle;
    }

    public GrabState getState() {
        return currentGrabState;
    }

    public void setState(Climber.GrabState grabState) {
        currentGrabState = grabState;
    }

    public void goToSetTarget() {
        if (getState() != GrabState.GRABBED) {
            if (getCurrentAngle() < getTarget() - 5) {
                winchOut();
            } else if (getCurrentAngle() > getTarget() + 5) {
                winchIn();

            } else {
                if (getState() == Climber.GrabState.GRABBING) {
                    setState(GrabState.GRABBED);
                }
                winchMotor.setVoltage(0);
            }
        } else {
            winchMotor.setVoltage(0);
        }
    }

    public void winchOut() {
        winchMotor.setVoltage(Constants.Climber.OUT_WINCH_SPEED * 12);
    }

    public void winchIn() {
        winchMotor.setVoltage(Constants.Climber.IN_WINCH_SPEED * 12);
    }

    public void stopWinch() {
        winchMotor.setVoltage(0);
    }

    @Logged
    public double getWinchMotorDraw() {
        return winchMotor.getOutputCurrent();
    }
}
