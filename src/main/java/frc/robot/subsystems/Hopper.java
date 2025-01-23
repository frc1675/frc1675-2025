// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    private SparkMax HOPPER_MOTOR;
    private HopperState HopperCurrentState;

    /** Creates a new Hopper. */
    public Hopper() {
        HOPPER_MOTOR = new SparkMax(0, MotorType.kBrushless); // Replace CAN ID with constant
        HopperCurrentState = HopperState.OFF;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    enum HopperState {
        ON,
        OFF,
        REVERSE
    }

    public void changeState(Hopper.HopperState hopperState) {
        HopperCurrentState = hopperState;
        if (getState() == HopperState.OFF) {
            HOPPER_MOTOR.setVoltage(0);
        }
        if (getState() == HopperState.ON) {
            HOPPER_MOTOR.setVoltage(12);
            // Should adjust to a speed constant later
        }
        if (getState() == HopperState.REVERSE) {
            HOPPER_MOTOR.setVoltage(-12);
            // Should adjust to a speed constant later
        }
    }

    public HopperState getState() {
        return HopperCurrentState;
    }
    // make something that makes sum motor spin jus like last year intake robot but just one not two

}
