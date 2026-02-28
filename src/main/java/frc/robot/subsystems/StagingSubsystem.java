package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StagingConstants;

public class StagingSubsystem extends SubsystemBase {
    SparkMax m_feedIntoHoodMotor = new SparkMax(StagingConstants.kFeedIntoHoodMotor, MotorType.kBrushless);
    SparkMax m_agitationMotor = new SparkMax(StagingConstants.kAgitationMotorId, MotorType.kBrushless);
    SparkMax m_rollerMotor = new SparkMax(StagingConstants.kRollerMotorId, MotorType.kBrushless);

    public void Agitate() {
        m_agitationMotor.set(StagingConstants.kAgitationSpeed);
    }

    // NOT ACTUALLY FEEDING FUEL TO OTHER SIDE OF FIELD, this feed refers to feedin
    // fuel into the hood
    public void Feed() {
        m_feedIntoHoodMotor.set(StagingConstants.kFeedIntoHoodSpeed);
    }

    // Refers to the roller that rolls balls into the feeder
    public void Roll() {
        m_rollerMotor.set(StagingConstants.kRollerSpeed);
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
}
