// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private boolean hasCoral;

    private SparkMax shooter;

    //  private LaserCan laserCAN;

    /** Creates a new Manipulator. */
    public Manipulator() {
        shooter = new SparkMax(1, MotorType.kBrushless);
        hasCoral = true;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // checks sensor if it's loaded or not --> will trasnition to loaded when coral is in

        if (state == ManipulatorState.WAITING && hasCoral) {

            state = ManipulatorState.LOADED;
        }

        if (state == ManipulatorState.SHOOTING && !hasCoral) {
            state = ManipulatorState.WAITING;

            shooter.setVoltage(0);
        }
    }

    enum ManipulatorState {
        LOADED,
        // the state when there is 1 coral present in the manipulator
        // Will be "waiting" if not ready to shoot --> maybe refer "waiting" to "hold" in the loaded state
        SHOOTING,
        // the state when 1 coral is shot to score L1 at the reef
        // triggered when driver
        WAITING
        // the state when there is no coral present in the manipulator
    }

    ManipulatorState state = ManipulatorState.WAITING;

    public ManipulatorState getState() {
        return state;
    }

    // Shooting changes state of manipulator to shooting, then waiting since no coral is detected by sensor
    public void shoot() {
        if (state == ManipulatorState.LOADED) {
            state = ManipulatorState.SHOOTING;

            shooter.setVoltage(1);
        }
    }
}
