// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private boolean hasCoral;

    /** Creates a new Manipulator. */
    public Manipulator() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    enum ManipulatorState {
        LOADED,
        SHOOTING,
        WAITING
    }

    ManipulatorState state = ManipulatorState.WAITING;

    public ManipulatorState getState() {
        return state;
    }

    public void changeState(Manipulator.ManipulatorState changeManipulatorState) {}

    public void shoot() {}
}
