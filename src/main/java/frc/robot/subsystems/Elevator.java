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
        elevatorMotor = new SparkMax(Constants.Elevator.ELEVATOR, MotorType.kBrushless);
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
        if (getLevel() == ElevatorLevel.LEVEL_1) {
            setAngle(Constants.Elevator.LEVEL_ONE_ANGLE);
            motorPower = -1.0 * pid.calculate(getAngle(), targetAngle);
            elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * motorPower);
        }
        if (getLevel() == ElevatorLevel.LEVEL_2) {
            setAngle(Constants.Elevator.LEVEL_TWO_ANGLE);
            motorPower = -1.0 * pid.calculate(getAngle(), targetAngle);
            elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * .1);
        }
        if (getLevel() == ElevatorLevel.LEVEL_3) {
            setAngle(Constants.Elevator.LEVEL_THREE_ANGLE);
            motorPower = -1.0 * pid.calculate(getAngle(), targetAngle);
            elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * .1);
        }
    }

    public void setTarget(Elevator.ElevatorLevel elevatorLevel) {
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

    public double getAngle() {
        return elevatorEncoder.get() * 360;
    }

    public void setAngle(double angle) {
        targetAngle = angle;
    }
}
