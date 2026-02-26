package frc.robot.subsystems;

import edu.wpi.first.units.Units.*;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    SparkMax m_climbMotor = new SparkMax(ClimberConstants.kMotorId, MotorType.kBrushless);

    RelativeEncoder m_relativeEncoder = m_climbMotor.getEncoder();

    private final SparkLimitSwitch m_minLimitSwitch = m_climbMotor.getReverseLimitSwitch();
    private final SparkLimitSwitch m_maxLimitSwitch = m_climbMotor.getForwardLimitSwitch();
    
    public void climbUp() {
        Angle position = Radians.of(m_relativeEncoder.getPosition());

        if (ClimberConstants.kMaxExtension.lt(position))
        {
            m_climbMotor.set(ClimberConstants.kUp);
        } else 
        {
            m_climbMotor.set(0);
        }
    }

    public void climbDown() {
        Angle position = Radians.of(m_relativeEncoder.getPosition());

        if (ClimberConstants.kMaxExtension.lt(position))
        {
            m_climbMotor.set(ClimberConstants.kDown);
        } else 
        {
            m_climbMotor.set(0);
        }
    }

    public void stop() {
        m_climbMotor.set(0);
    }


    @Override
    public void periodic() {
        if (m_minLimitSwitch.isPressed()) {
            m_relativeEncoder.setPosition(ClimberConstants.kMinExtension.in(Rotations));
        } else if (m_maxLimitSwitch.isPressed()) {
            m_relativeEncoder.setPosition(ClimberConstants.kMaxExtension.in(Rotations));
        }
    }
}
