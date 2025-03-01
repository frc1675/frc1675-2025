package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Elevator {
    private static final ElevatorLevel ElevatorLevel = null;
    public SparkMax elevatorMotor;
    public ElevatorLevel elevatorCurrentLevel;
    private double elevatorEncoder;
    private DigitalInput homeSwitch;

    public Elevator() {
        elevatorMotor = new SparkMax(Constants.Elevator.ELEVATOR, MotorType.kBrushless);
        //    elevatorEncoder = new
        homeSwitch = new DigitalInput(Constants.Elevator.ELEVATOR_HOMESWITCH);

        elevatorCurrentLevel = ElevatorLevel.OFF;
    }

    public enum ElevatorLevel {
        OFF,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
    }

    public void periodic() {
        if (getLevel() == ElevatorLevel.LEVEL_1) {
            elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * .1);
        }
        if (getLevel() == ElevatorLevel.LEVEL_2) {
            elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * .1);
        }
        if (getLevel() == ElevatorLevel.LEVEL_3) {
            elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * .1);
        }
    }

    public void changeState(Elevator.ElevatorLevel elevatorLevel) {
        elevatorCurrentLevel = ElevatorLevel;
    }

    public ElevatorLevel getLevel() {
        return elevatorCurrentLevel;
    }

    public boolean isAtHome() {
        return getHomeSwitch();
    }

    public boolean getHomeSwitch() {
        return !homeSwitch.get();
    }

    public void elevatorUp() {
        elevatorMotor.setVoltage(Constants.Elevator.ELEVATOR_UP * 12);
    }

    public void elevatorDown() {
        elevatorMotor.setVoltage(Constants.Elevator.ELEVATOR_DOWN * 12);
    }
}
