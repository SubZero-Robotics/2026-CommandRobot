package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkLimitSwitch;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);
    private final SparkMax m_deployMotor = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);

    private final SparkClosedLoopController m_intakeClosedLoopController = m_intakeMotor.getClosedLoopController();
    private final SparkClosedLoopController m_deployClosedLoopController = m_deployMotor.getClosedLoopController();

    private final RelativeEncoder m_deployRelativeEncoder = m_deployMotor.getEncoder();

    private final SparkLimitSwitch m_minLimitSwitch = m_intakeMotor.getReverseLimitSwitch();
    private final SparkLimitSwitch m_maxLimitSwitch = m_intakeMotor.getForwardLimitSwitch();

    private SparkMaxConfig m_deployConfig = new SparkMaxConfig();

    public IntakeSubsystem() {

        m_deployConfig.closedLoop.p(IntakeConstants.kP).i(IntakeConstants.kI)
                .d(IntakeConstants.kD);
        // Apparently there is no absolute encoder :(
        // m_deployConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_deployConfig.idleMode(IdleMode.kBrake);
        m_deployConfig.smartCurrentLimit(IntakeConstants.kDeployMotorCurrentLimit);
        m_deployMotor.configure(m_deployConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void spinIntake(AngularVelocity velocity) {
        m_intakeClosedLoopController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }

    public void stopIntake() {
        spinIntake(RPM.of(0));
    }

    public void deployIntake() {
        m_deployClosedLoopController.setSetpoint(
                IntakeConstants.kDeployRotations.in(Rotations),
                ControlType.kPosition);
    }

    public void retractIntake() {
        m_deployClosedLoopController.setSetpoint(
                IntakeConstants.kRetractRotations.in(Rotations),
                ControlType.kPosition);
    }

    @Override
    public void periodic() {
        if (m_minLimitSwitch.isPressed()) {
            m_deployRelativeEncoder.setPosition(IntakeConstants.kMinExtension.in(Rotations));
        } else if (m_maxLimitSwitch.isPressed()) {
            m_deployRelativeEncoder.setPosition(IntakeConstants.kMaxExtension.in(Rotations));
        }
    }
}
