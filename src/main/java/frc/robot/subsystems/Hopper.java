// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    private SparkMax hopperMotor;
    private HopperState hopperCurrentState;

    /** Creates a new Hopper. */
    public Hopper() {
        hopperMotor = new SparkMax(Constants.Hopper.HOPPER_MOTOR, MotorType.kBrushless); // Replace CAN ID with constant
        hopperCurrentState = HopperState.OFF;
        ShuffleboardTab tab = Shuffleboard.getTab("Hopper Display");
        tab.add("Hopper Shuffleboard", 3);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (getState() == HopperState.OFF) {
            hopperMotor.setVoltage(0);
        }
        if (getState() == HopperState.ON) {
            hopperMotor.setVoltage(Constants.Hopper.HOPPER_INTAKE_SPEED * 12.0);
        }
        if (getState() == HopperState.REVERSE) {
            hopperMotor.setVoltage(Constants.Hopper.HOPPER_REVERSE_SPEED * 12.0);
        }
    }

    public enum HopperState {
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
