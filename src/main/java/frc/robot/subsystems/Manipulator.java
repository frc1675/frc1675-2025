// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private boolean hasCoral;

    private SparkMax shooter;

    ManipulatorState state = ManipulatorState.EMPTY;

    private final double DELAY = 1.5;
    private final double SHOOTING_SPEED = 1.0;
    private final double INTAKE_SPEED = 0.2;
    private final double MAX_VOLTAGE = 12.0;
    private Timer stopwatch;

    //  private LaserCan laserCAN;

    /** Creates a new Manipulator. */
    public Manipulator() {
        shooter = new SparkMax(1, MotorType.kBrushless);
        hasCoral = false;

        stopwatch = new Timer();
        // Not sure if timer starts automaticallly but wants to be off
        stopwatch.stop();
        stopwatch.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // checks sensor if it's loaded or not --> will trasnition to loaded when coral is in

        // stage 1 - Transitioning between states
        if (state == ManipulatorState.EMPTY && hasCoral) {

            state = ManipulatorState.LOADED;
        }

        if (state == ManipulatorState.SHOOTING) {

            if (!hasCoral) {

                if (!stopwatch.isRunning()) {
                    stopwatch.restart();
                } else {
                    if (stopwatch.hasElapsed(DELAY)) {
                        state = ManipulatorState.EMPTY;
                        stopwatch.stop();
                    }
                }
            }
        } // End of stage 1

        // stage 2 - Setting the motor speed
        if (state == ManipulatorState.SHOOTING) {
            shooter.setVoltage(SHOOTING_SPEED * MAX_VOLTAGE);
        }

        if (state == ManipulatorState.EMPTY) {
            shooter.setVoltage(INTAKE_SPEED * MAX_VOLTAGE);
        }

        if (state == ManipulatorState.LOADED) {
            shooter.setVoltage(0);
        }
    }

    enum ManipulatorState {

        // the state when there is 1 coral present in the manipulator
        // Will be "empty" if not ready to shoot
        LOADED,

        // the state when 1 coral is shot to score L1 at the reef
        // triggered when driver
        SHOOTING,

        // the state when there is no coral present in the manipulator
        EMPTY
    }

    public ManipulatorState getState() {
        return state;
    }

    // Shooting changes state of manipulator to shooting, then waiting since no coral is detected by sensor
    public void shoot() {
        if (state == ManipulatorState.LOADED) {
            state = ManipulatorState.SHOOTING;
        }
    }
}
