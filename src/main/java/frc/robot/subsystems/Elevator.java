package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Elevator {
    private static final ElevatorLevel ElevatorLevel = null;
    public SparkMax elevatorMotor;
    public ElevatorLevel elevatorCurrentLevel;
    private double elevatorEncoder;
    private DigitalInput homeSwitch;

    private ProfiledPIDController pid;
    private TrapezoidProfile.Constraints profileConstraints;

    public Elevator() {
        elevatorMotor = new SparkMax(Constants.Elevator.ELEVATOR_MOTOR, MotorType.kBrushless);
        //    elevatorEncoder = new
        homeSwitch = new DigitalInput(Constants.Elevator.ELEVATOR_HOMESWITCH);

        profileConstraints = new TrapezoidProfile.Constraints(
                Constants.Elevator.ELEVATOR_MAX_VELOCITY, Constants.Elevator.ELEVATOR_MAX_ACCELERATION);
        pid = new ProfiledPIDController(
                Constants.Elevator.ELEVATOR_PID_P_COEFFICIENT,
                Constants.Elevator.ELEVATOR_PID_I_COEFFICIENT,
                Constants.Elevator.ELEVATOR_PID_D_COEFFICIENT,
                profileConstraints);

        elevatorCurrentLevel = ElevatorLevel.OFF;
    }

    public enum ElevatorLevel {
        OFF,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
    }

    public void periodic() {
        if (setTarget() == ElevatorLevel.LEVEL_1) {
            elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * .1);
        }
        if (setTarget() == ElevatorLevel.LEVEL_2) {
            elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * .1);
        }
        if (setTarget() == ElevatorLevel.LEVEL_3) {
            elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * .1);
        }
    }

    public void setTarget(Elevator.ElevatorLevel elevatorLevel) {
        elevatorCurrentLevel = ElevatorLevel;
    }

    public ElevatorLevel setTarget() {
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
    // multiplied voltage from original set (1, -1) in constants for elevator movement
    public void elevatorDown() {
        elevatorMotor.setVoltage(Constants.Elevator.ELEVATOR_DOWN * 12);
    }
}
