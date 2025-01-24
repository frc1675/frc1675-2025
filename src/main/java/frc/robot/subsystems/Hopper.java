// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    private SparkMax hopperMotor;
    private HopperState hopperCurrentState;

    /** Creates a new Hopper. */
    public Hopper() {
        hopperMotor = new SparkMax(0, MotorType.kBrushless); // Replace CAN ID with constant
        hopperCurrentState = HopperState.OFF;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (getState() == HopperState.OFF) {
            hopperMotor.setVoltage(0);
        }
        if (getState() == HopperState.ON) {
            hopperMotor.setVoltage(12);
            // Should adjust to a speed constant later
        }
        if (getState() == HopperState.REVERSE) {
            hopperMotor.setVoltage(-12);
            // Should adjust to a speed constant later
        }
    }

    enum HopperState {
        ON,
        OFF,
        REVERSE
    }

    public void changeState(Hopper.HopperState hopperState) {
        hopperCurrentState = hopperState;
    }

    public HopperState getState() {
        return hopperCurrentState;
    }
}
