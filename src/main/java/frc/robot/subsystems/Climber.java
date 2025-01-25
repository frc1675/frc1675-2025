// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private double targetAngle;

    private SparkMax winchMotor;

    private DutyCycleEncoder climberEncoder;
    private static final int ENCODER_CHANNEL = 1;

    private int CLIMB_MOTOR;
    private double WINCH_MOTOR_SPEED = 1;
    private double DEPLOY_WINCH_MAX_ACCELERATION = 1;
    private double DEPLOY_WINCH_MAX_VELOCITY = 1;

    private double RETRACT_WINCH_MAX_ACCELERATION = 1;
    private double RETRACT_WINCH_MAX_VELOCITY = 1;

    private ProfiledPIDController retractWinchPID;
    public static final double RETRACT_PID_P_COEFFICIENT = 0;
    public static final double RETRACT_PID_I_COEFFICIENT = 0;
    public static final double RETRACT_PID_D_COEFFICIENT = 0;
    private TrapezoidProfile.Constraints retractConstraints;

    private ProfiledPIDController deployWinchPID;
    public static final double DEPLOY_PID_P_COEFFICIENT = 0;
    public static final double DEPLOY_PID_I_COEFFICIENT = 0;
    public static final double DEPLOY_PID_D_COEFFICIENT = 0;
    private TrapezoidProfile.Constraints deployConstraints;

    /** Creates a new Climber. */
    public Climber() {

        winchMotor = new SparkMax(CLIMB_MOTOR, MotorType.kBrushless);

        climberEncoder = new DutyCycleEncoder(ENCODER_CHANNEL);

        deployConstraints = new TrapezoidProfile.Constraints(DEPLOY_WINCH_MAX_VELOCITY, DEPLOY_WINCH_MAX_ACCELERATION);
        retractConstraints =
                new TrapezoidProfile.Constraints(RETRACT_WINCH_MAX_VELOCITY, RETRACT_WINCH_MAX_ACCELERATION);

        retractWinchPID = new ProfiledPIDController(
                DEPLOY_PID_P_COEFFICIENT, DEPLOY_PID_I_COEFFICIENT, DEPLOY_PID_D_COEFFICIENT, deployConstraints);
        deployWinchPID = new ProfiledPIDController(
                RETRACT_PID_P_COEFFICIENT, RETRACT_PID_I_COEFFICIENT, RETRACT_PID_D_COEFFICIENT, retractConstraints);
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
