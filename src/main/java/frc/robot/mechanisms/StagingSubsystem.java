package frc.robot.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import org.wpilib.command3.Mechanism;
import frc.robot.constants.CANConstants;
import frc.robot.constants.StagingConstants;

public class StagingSubsystem implements Mechanism {
    SparkMax m_feedIntoHoodMotor = new SparkMax(CANConstants.kCanPort, StagingConstants.kFeedIntoHoodMotor, MotorType.kBrushless);
    SparkMax m_agitationMotor = new SparkMax(CANConstants.kCanPort, StagingConstants.kAgitationMotorId, MotorType.kBrushless);
    SparkMax m_rollerMotor = new SparkMax(CANConstants.kCanPort, StagingConstants.kRollerMotorId, MotorType.kBrushless);

    public StagingSubsystem() {
    }

    public void Agitate() {
        m_agitationMotor.setThrottle(StagingConstants.kAgitationSpeed);
    }

    // NOT ACTUALLY FEEDING FUEL TO OTHER SIDE OF FIELD, this feed refers to feedin
    // fuel into the hood
    public void Feed() {
        m_feedIntoHoodMotor.setThrottle(StagingConstants.kFeedIntoHoodSpeed);
    }

    // Refers to the roller that rolls balls into the feeder
    public void Roll() {
        m_rollerMotor.setThrottle(StagingConstants.kRollerSpeed);
    }

    public void StopAgitate() {
        m_agitationMotor.stopMotor();
    }

    public void StopFeed() {
        m_feedIntoHoodMotor.stopMotor();
    }

    public void StopRoll() {
        m_rollerMotor.stopMotor();
    }

    public void reverseAgitater() {
        m_agitationMotor.setThrottle(StagingConstants.kReverseAgitationSpeed);
    }

    public void reverseRoller() {
        m_rollerMotor.setThrottle(StagingConstants.kReverseRollingSpeed);
    }

    public void reverseFeeder() {
        m_feedIntoHoodMotor.setThrottle(StagingConstants.kReverseFeedSpeed);
    }
}
