// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    public Climber() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double getCurrentAngle() {
        return 1;
    }

    public void getTarget(double target) {}

    public void setTarget(double angle) {}

    public boolean isCageDetected() {
        return false;
    }
}
