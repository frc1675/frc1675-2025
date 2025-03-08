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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Climber extends SubsystemBase {

    @NotLogged
    private double targetAngle;

    private ShuffleboardTab dashboard;

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

    /** Creates a new Climber. */
    public Climber() {
        winchMotor = new SparkMax(Constants.Climber.CLIMB_MOTOR, MotorType.kBrushless);
        climberEncoder = new DutyCycleEncoder(Constants.Climber.ENCODER_CHANNEL);
        initDashboard();
    }

    private void initDashboard() {
        dashboard = Shuffleboard.getTab("Climber");
        dashboard.addDouble("Get angle", () -> getCurrentAngle());
    }

    @Override
    public void periodic() {

        // will change comparison when we know how encoder works
        if (getCurrentAngle() > Constants.Climber.CLIMBER_MAX_ANGLE) {
            winchMotor.setVoltage(0);
        }
    }

    @Logged
    public double getCurrentAngle() {
        return climberEncoder.get() * 360;
    }

    public double getTarget() {
        return targetAngle;
    }

    public void setTarget(double angle) {
        targetAngle = angle;
    }

    public void deployWinch() {
        winchMotor.setVoltage(Constants.Climber.DEPLOY_WINCH_SPEED * 12);
    }

    public void retractWinch() {
        winchMotor.setVoltage(Constants.Climber.RETRACT_WINCH_SPEED * 12);
    }

    public void stopWinch() {
        winchMotor.setVoltage(0);
    }
}
