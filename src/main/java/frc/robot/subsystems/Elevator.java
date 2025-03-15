package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Elevator extends SubsystemBase {
    private static final ElevatorLevel ElevatorLevel = null;
    public SparkMax elevatorMotor;
    public double motorPower = 0;
    public ElevatorLevel elevatorCurrentLevel;
    private Encoder elevatorEncoder;
    private DigitalInput homeSwitch;
    private double targetAngle;
    private boolean toldGoUp = false;
    private double testMotorPower = 0;

    private ProfiledPIDController pid;
    private TrapezoidProfile.Constraints profileConstraints;

    public Elevator() {

        elevatorEncoder = new Encoder(Constants.Elevator.ENCODER_A, Constants.Elevator.ENCODER_B);
        elevatorMotor = new SparkMax(Constants.Elevator.ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorEncoder.setReverseDirection(true);
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

    @Override
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

        motorPower = pid.calculate(getAngle(), targetAngle);
        if (motorPower < 0) { // if trying to go down
            if (isHome() || getAngle() < 0) {
                elevatorMotor.setVoltage(0);
            } else {
                elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * motorPower);
            }
        }

        if (motorPower > 0) { // if trying to go up
            if (getAngle() > Constants.Elevator.MAX_LIMIT) {
                elevatorMotor.setVoltage(0);
            } else {
                elevatorMotor.setVoltage(Constants.Elevator.MAX_VOLTAGE * motorPower);
            }
        }

        if (isHome() == true) {
            elevatorEncoder.reset();
        }
    }

    public void setTarget(Elevator.ElevatorLevel elevatorLevel) {
        elevatorCurrentLevel = elevatorLevel;
    }

    public ElevatorLevel getLevel() {
        return elevatorCurrentLevel;
    }

    public boolean isHome() {
        return !homeSwitch.get();
    }

    public void elevatorUp() {
        motorPower = Constants.Elevator.ELEVATOR_UP;
    }
    // multiplied voltage from original set (1, -1) in constants for elevator movement
    public void elevatorDown() {
        motorPower = Constants.Elevator.ELEVATOR_DOWN;
    }

    public double getAngle() {
        return elevatorEncoder.get() / 2048.0 * 360.0;
    }

    public void setAngle(double angle) {
        targetAngle = angle;
    }

    public double getSetAngle() {
        return targetAngle;
    }
}
