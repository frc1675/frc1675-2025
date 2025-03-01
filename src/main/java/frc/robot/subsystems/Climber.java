// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private double targetAngle;

    private ShuffleboardTab dashboard;

    private SparkMax winchMotor;

    private DutyCycleEncoder climberEncoder;

    /** Creates a new Climber. */
    public Climber() {
        winchMotor = new SparkMax(Constants.Climber.CLIMB_MOTOR, MotorType.kBrushless);
        climberEncoder = new DutyCycleEncoder(Constants.Climber.ENCODER_CHANNEL);
    }

    private void initDashboard() {
        dashboard = Shuffleboard.getTab("Manipulator");
        dashboard.add("Get angle", getCurrentAngle());
    }

    @Override
    public void periodic() {

        // will change comparison when we know how encoder works
        if (getCurrentAngle() > getTarget()) {}
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
