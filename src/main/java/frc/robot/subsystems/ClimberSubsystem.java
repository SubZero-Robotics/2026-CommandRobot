package frc.robot.subsystems;

import edu.wpi.first.units.Units.*;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem {
    SparkMax m_climbMotor = new SparkMax(ClimberConstants.kMotorId, MotorType.kBrushless);

    AbsoluteEncoder m_absoluteEncoder = m_climbMotor.getAbsoluteEncoder();

    private final SparkClosedLoopController m_climberClosedLoopController = m_climbMotor.getClosedLoopController();

    private final SparkMaxConfig m_climbConfig = new SparkMaxConfig();


    public ClimberSubsystem() {

        m_climbConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        m_climbMotor.configure(m_climbConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void climb(AngularVelocity climbVelocity) {
        m_climberClosedLoopController.setSetpoint(climbVelocity.in(RPM), ControlType.kVelocity);
    }
}
