// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Hopper extends SubsystemBase {

    @NotLogged
    private SparkMax hopperMotor;

    @Logged
    private HopperState hopperCurrentState;

    @NotLogged
    private Timer hopperTimer;
    // intake speed, and speed when intake would go reverse

    /** Creates a new Hopper.*/
    public Hopper() {
        hopperMotor = new SparkMax(Constants.Hopper.HOPPER_MOTOR, MotorType.kBrushless); // Replace CAN ID with constant
        hopperTimer = new Timer();
        hopperTimer.stop();
        hopperTimer.reset();

        hopperCurrentState = HopperState.ON;
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

        if (hopperCurrentState == Hopper.HopperState.REVERSE) {
            if (!hopperTimer.isRunning()) {
                hopperTimer.restart();
            } else {
                if (hopperTimer.hasElapsed(3)) {
                    hopperCurrentState = Hopper.HopperState.ON;
                    hopperTimer.stop();
                }
            }
        }
    }

    // can probably replace the "on" state and is written this way to not interfere with "reverse" and "off" states

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
