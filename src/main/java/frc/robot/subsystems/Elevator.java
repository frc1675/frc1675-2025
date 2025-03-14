package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class Elevator {
    private static final ElevatorLevel ElevatorLevel = null;
    public SparkMax elevatorMotor;
    public double motorPower = 0;
    public ElevatorLevel elevatorCurrentLevel;
    private DutyCycleEncoder elevatorEncoder;
    private DigitalInput homeSwitch;
    private double targetAngle;

    private ProfiledPIDController pid;
    private TrapezoidProfile.Constraints profileConstraints;

    public Elevator() {
        elevatorMotor = new SparkMax(Constants.Elevator.ELEVATOR_MOTOR, MotorType.kBrushless);

        homeSwitch = new DigitalInput(Constants.Elevator.ELEVATOR_HOMESWITCH);

        profileConstraints = new TrapezoidProfile.Constraints(
                Constants.Elevator.ELEVATOR_MAX_VELOCITY, Constants.Elevator.ELEVATOR_MAX_ACCELERATION);

        pid = new ProfiledPIDController(
                Constants.Elevator.ELEVATOR_PID_P_COEFFICIENT,
                Constants.Elevator.ELEVATOR_PID_I_COEFFICIENT,
                Constants.Elevator.ELEVATOR_PID_D_COEFFICIENT,
                profileConstraints);

        elevatorCurrentLevel = ElevatorLevel.LEVEL_1;
    }

    public enum ElevatorLevel {
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
    }

    public void periodic() {
        if (getLevel() == ElevatorLevel.LEVEL_1) {
            setAngle(Constants.Elevator.LEVEL_ONE_ANGLE);
        }

        if (getLevel() == ElevatorLevel.LEVEL_2) {
            setAngle(Constants.Elevator.LEVEL_TWO_ANGLE);
        }

        if (getLevel() == ElevatorLevel.LEVEL_3) {
            setAngle(Constants.Elevator.LEVEL_THREE_ANGLE);
        }

        motorPower = -1.0 * pid.calculate(getAngle(), targetAngle);

        if (motorPower > 0) { // if trying to go down
            if (isHome()) {
                elevatorMotor.setVoltage(0);
            } else {
                elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * motorPower);
            }
        }

        if (motorPower < 0) { // if trying to go up
            if (getAngle() > Constants.Elevator.MAX_LIMIT) {
                elevatorMotor.setVoltage(0);
            } else {
                elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * motorPower);
            }
        }
    }

    public void setTarget(Elevator.ElevatorLevel elevatorLevel) {
        elevatorCurrentLevel = ElevatorLevel;
    }

    public ElevatorLevel getLevel() {
        return elevatorCurrentLevel;
    }

    public boolean isHome() {
        return !homeSwitch.get();
    }

    public void elevatorUp() {
        elevatorMotor.setVoltage(Constants.Elevator.ELEVATOR_UP * 12);
    }
    // multiplied voltage from original set (1, -1) in constants for elevator movement
    public void elevatorDown() {
        elevatorMotor.setVoltage(Constants.Elevator.ELEVATOR_DOWN * 12);
    }

    public double getAngle() {
        return elevatorEncoder.get() * 360;
    }

    public void setAngle(double angle) {
        targetAngle = angle;
    }
}
