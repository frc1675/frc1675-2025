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

    /** Creates a new Climber. */
    public Climber() {
        winchMotor = new SparkMax(Constants.Climber.CLIMB_MOTOR, MotorType.kBrushless);
        climberEncoder = new DutyCycleEncoder(Constants.Climber.ENCODER_CHANNEL);

        deployConstraints = new TrapezoidProfile.Constraints(
                Constants.Climber.DEPLOY_WINCH_MAX_VELOCITY, Constants.Climber.DEPLOY_WINCH_MAX_ACCELERATION);
        retractConstraints = new TrapezoidProfile.Constraints(
                Constants.Climber.RETRACT_WINCH_MAX_VELOCITY, Constants.Climber.RETRACT_WINCH_MAX_ACCELERATION);

        retractWinchPID = new ProfiledPIDController(
                Constants.Climber.DEPLOY_PID_P_COEFFICIENT,
                Constants.Climber.DEPLOY_PID_I_COEFFICIENT,
                Constants.Climber.DEPLOY_PID_D_COEFFICIENT,
                deployConstraints);
        deployWinchPID = new ProfiledPIDController(
                Constants.Climber.RETRACT_PID_P_COEFFICIENT,
                Constants.Climber.RETRACT_PID_I_COEFFICIENT,
                Constants.Climber.RETRACT_PID_D_COEFFICIENT,
                retractConstraints);
    }

    @Override
    public void periodic() {
        double motorPower;
        // will change comparison when we know how encoder works
        if (getCurrentAngle() > getTarget()) {
            motorPower = deployWinchPID.calculate(getCurrentAngle(), getTarget());
        } else {
            motorPower = retractWinchPID.calculate(getCurrentAngle(), getTarget());
        }

        winchMotor.setVoltage(motorPower);
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
}
