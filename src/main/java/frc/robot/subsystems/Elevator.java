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

public class Elevator extends SubsystemBase {
    private int ELEVATOR_MOTOR;

    private int targetLevel;

    private double motorPower;

    private DutyCycleEncoder elevatorEncoder;
    private static final int ENCODER_CHANNEL = 1;

    private double DEPLOY_MAX_ACCELERATION = 1;
    private double DEPLOY_MAX_VELOCITY = 1;

    private ProfiledPIDController deployElevatorPID;
    public static final double DEPLOY_PID_P_COEFFICIENT = 0;
    public static final double DEPLOY_PID_I_COEFFICIENT = 0;
    public static final double DEPLOY_PID_D_COEFFICIENT = 0;
    private TrapezoidProfile.Constraints deployConstraints;

    private double deployPTuning = 0;
    private double deployITuning = 0;
    private double deployDTuning = 0;

    private SparkMax elevatorMotor;

    public Elevator() {
        elevatorMotor = new SparkMax(ELEVATOR_MOTOR, MotorType.kBrushless);

        elevatorEncoder = new DutyCycleEncoder(ENCODER_CHANNEL);

        deployConstraints = new TrapezoidProfile.Constraints(DEPLOY_MAX_VELOCITY, DEPLOY_MAX_ACCELERATION);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        motorPower = deployElevatorPID.calculate(getCurrentLevel(), getTargetLevel());

        elevatorMotor.setVoltage(motorPower * 12);
    }

    public double getCurrentLevel() {
        return elevatorEncoder.get();
    }

    public int getTargetLevel() {
        return targetLevel;
    }

    public void setTargetLevel(int level) {
        targetLevel = level;
    }
}
