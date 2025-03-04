// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
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
public class Manipulator extends SubsystemBase {

    private boolean hasCoral;

    @NotLogged
    private ShuffleboardTab dashboard;

    @NotLogged
    private SparkMax shooter;

    @NotLogged
    private SparkMax shootTwo;

    ManipulatorState state = ManipulatorState.EMPTY;

    @NotLogged
    private Timer shooterTimer;

    @NotLogged
    private Timer manipulatorTimer;

    @NotLogged
    public LaserCan laserCAN;
    // 0.155-0.160m without coral
    // 0.075-0.080m with coral

    /** Creates a new Manipulator. */
    public Manipulator() {
        shooter = new SparkMax(Constants.Manipulator.MANIPULATOR_MOTOR_1, MotorType.kBrushless); // bottom
        shootTwo = new SparkMax(Constants.Manipulator.MANIPULATOR_MOTOR_2, MotorType.kBrushless); // top
        shootTwo.setInverted(true);
        hasCoral = false;

        shooterTimer = new Timer();
        // Not sure if timer starts automaticallly but wants to be off
        shooterTimer.stop();
        shooterTimer.reset();

        // manipulatorTimer = new Timer();
        // manipulatorTimer.stop();
        // manipulatorTimer.reset();

        laserCAN = new LaserCan(Constants.Manipulator.CORAL_SENSOR);

        try {
            laserCAN.setRangingMode(RangingMode.SHORT);
            laserCAN.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();

            initDashboard();
        }
    }

    private void initDashboard() {
        dashboard = Shuffleboard.getTab("Manipulator");
        dashboard.addBoolean("Has Coral", () -> hasCoral);
        dashboard.add("Current State", state);
    }

    @Override
    public void periodic() {

        if (getMeasurement() < Constants.Manipulator.DETECTION_RANGE) {
            hasCoral = true;
        } else {
            hasCoral = false;
        }

        // This method will be called once per scheduler run
        // checks sensor if it's loaded or not --> will trasnition to loaded when coral is in

        // stage 1 - Transitioning between states
        if (state == ManipulatorState.EMPTY && hasCoral) {

            state = ManipulatorState.LOADED;
        }

        if (state == ManipulatorState.SHOOTING) {

            if (!hasCoral) {

                if (!shooterTimer.isRunning()) {
                    shooterTimer.restart();
                } else {
                    if (shooterTimer.hasElapsed(Constants.Manipulator.DELAY)) {
                        state = ManipulatorState.EMPTY;
                        shooterTimer.stop();
                    }
                }
            }
        } // End of stage 1

        // stage 2 - Setting the motor speed
        if (state == ManipulatorState.SHOOTING) {
            shooter.setVoltage(Constants.Manipulator.BOTTOM_SHOOTING_SPEED * Constants.Manipulator.MAX_VOLTAGE);
            shootTwo.setVoltage(Constants.Manipulator.TOP_SHOOTING_SPEED * Constants.Manipulator.MAX_VOLTAGE);
        }

        if (state == ManipulatorState.EMPTY) {
            shooter.setVoltage(Constants.Manipulator.BOTTOM_INTAKE_SPEED * Constants.Manipulator.MAX_VOLTAGE);
            shootTwo.setVoltage(Constants.Manipulator.TOP_INTAKE_SPEED * Constants.Manipulator.MAX_VOLTAGE);
        }

        if (state == ManipulatorState.LOADED) {

            shooter.setVoltage(0);
            shootTwo.setVoltage(0);

            // if (!manipulatorTimer.isRunning()) {
            //     manipulatorTimer.restart();
            // } else {
            //     if (manipulatorTimer.hasElapsed(Constants.Manipulator.CHECK_DELAY)) {
            //         if(!hasCoral){
            //             state = ManipulatorState.EMPTY;
            //         }
            //         manipulatorTimer.stop();
            //     }
            // }
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

    public double getMeasurement() {
        return laserCAN.getMeasurement() == null ? 0 : laserCAN.getMeasurement().distance_mm;
    }

    public boolean manipulatorLoaded() {
        return hasCoral;
    }
}
