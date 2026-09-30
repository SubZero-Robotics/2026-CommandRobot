package frc.robot.mechanisms;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.units.measure.Angle;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

import com.revrobotics.RelativeEncoder;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem implements Mechanism {
    SparkMax m_climbMotor = new SparkMax(CANConstants.kCanPort, ClimberConstants.kMotorCanId, MotorType.kBrushless);

    RelativeEncoder m_relativeEncoder = m_climbMotor.getEncoder();

    private final SparkLimitSwitch m_minLimitSwitch = m_climbMotor.getReverseLimitSwitch();
    private final SparkLimitSwitch m_maxLimitSwitch = m_climbMotor.getForwardLimitSwitch();

    public ClimberSubsystem() {
        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    public double GetPosition() {
        return m_relativeEncoder.getPosition().get();
    }

    public void climbUp() {

        // if (!atMax()) {
        if (true) {
            m_climbMotor.setThrottle(ClimberConstants.kUpVelocity);
        } else {
            Stop();
        }
    }

    public void climbDown() {

        // if (!atMin()) { // TODO: Put this back
        if (true) {
            m_climbMotor.setThrottle(ClimberConstants.kDownVelocity);
        } else {
            Stop();
        }
    }

    public boolean atMax() {
        return GetPosition() >= ClimberConstants.kMaxExtension;
    }

    public boolean atMin() {
        return GetPosition() <= ClimberConstants.kMinExtension;
    }

    public void Stop() {
        m_climbMotor.setThrottle(0.0);
    }

    public Command ZeroCommand() {
        return Command.noRequirements(coroutine -> {
            m_relativeEncoder.setPosition(0);
        }).named("Zero Climber");
    }

    public void periodic() {
        if (m_minLimitSwitch.isPressed().get()) {
            m_relativeEncoder.setPosition(ClimberConstants.kLimitMinExtension);
        } else if (m_maxLimitSwitch.isPressed().get()) {
            m_relativeEncoder.setPosition(ClimberConstants.kLimitMinExtension);
        }

        // SmartDashboard.putNumber("Climber Position", GetPosition());
        // SmartDashboard.putData("Zero", ZeroCommand());
    }
}