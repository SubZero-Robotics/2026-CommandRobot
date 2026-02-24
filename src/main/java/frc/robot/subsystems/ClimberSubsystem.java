package frc.robot.subsystems;

import edu.wpi.first.units.Units.*;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    SparkMax m_climbMotor = new SparkMax(ClimberConstants.kMotorId, MotorType.kBrushless);

    RelativeEncoder m_relativeEncoder = m_climbMotor.getEncoder();

    private final SparkClosedLoopController m_climberClosedLoopController = m_climbMotor.getClosedLoopController();

    private final SparkMaxConfig m_climbConfig = new SparkMaxConfig();

    private final SparkLimitSwitch m_minLimitSwitch = m_climbMotor.getReverseLimitSwitch();
    private final SparkLimitSwitch m_maxLimitSwitch = m_climbMotor.getForwardLimitSwitch();



    public ClimberSubsystem() {

        m_climbConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        m_climbMotor.configure(m_climbConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void climb(AngularVelocity climbVelocity) {
                   
        m_climberClosedLoopController.setSetpoint(climbVelocity.in(RPM), ControlType.kVelocity);
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
