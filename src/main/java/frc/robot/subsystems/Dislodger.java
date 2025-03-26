// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Dislodger extends SubsystemBase {
    public SparkMax dislodgerMotor;
    public double motorPower = 0;
    public boolean isOn = false;
    /** Creates a new Dislodger. */
    public Dislodger() {
        dislodgerMotor = new SparkMax(Constants.Dislodger.DISLODGER_MOTOR, MotorType.kBrushless);
    }

    @Override
    public void periodic() {

        if (isOn == true) {
            dislodgerMotor.setVoltage(Constants.Dislodger.DISLODGER_MOTOR_SPEED * 12);
        } else {
            dislodgerMotor.setVoltage(0);
        }

        // This method will be called once per scheduler run
    }

    public void dislodgerOn() {
        isOn = true;
    }

    public void dislodgerOff() {
        isOn = false;
    }

    public void toggleOnOff() {
        if (isOn == false) {
            isOn = true;
        } else if (isOn == true) {
            isOn = false;
        }
    }
}
