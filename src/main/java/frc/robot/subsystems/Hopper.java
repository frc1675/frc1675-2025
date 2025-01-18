// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    /** Creates a new Hopper. */
    public Hopper() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    enum HopperState {
        ON,
        OFF,
        REVERSE
    }

    HopperState state = HopperState.OFF;

    public void changeState(Hopper.HopperState hopperState) {}

    public HopperState getState() {
        return state;
    }
}
