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
    private final SparkMax m_deployMotor1 = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);
    private final SparkMax m_deployMotor2 = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);

    private final SparkClosedLoopController m_intakeClosedLoopController = m_intakeMotor.getClosedLoopController();
    private final SparkClosedLoopController m_deploy1ClosedLoopController = m_deployMotor1.getClosedLoopController();
    private final SparkClosedLoopController m_deploy2ClosedLoopController = m_deployMotor2.getClosedLoopController();

    private final RelativeEncoder m_deploy1RelativeEncoder = m_deployMotor1.getEncoder();
    private final RelativeEncoder m_deploy2RelativeEncoder = m_deployMotor2.getEncoder();

    private final SparkLimitSwitch m_minLimitSwitch = m_intakeMotor.getReverseLimitSwitch();
    private final SparkLimitSwitch m_maxLimitSwitch = m_intakeMotor.getForwardLimitSwitch();

    private SparkMaxConfig m_deploy1Config = new SparkMaxConfig();
    private SparkMaxConfig m_deploy2Config = new SparkMaxConfig();

    public IntakeSubsystem() {

        m_deploy1Config.closedLoop.p(IntakeConstants.kP).i(IntakeConstants.kI)
                .d(IntakeConstants.kD);
        m_deploy2Config.closedLoop.p(IntakeConstants.kP).i(IntakeConstants.kI)
                .d(IntakeConstants.kD);
        // Apparently there is no absolute encoder :(
        // m_deployConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_deploy1Config.idleMode(IdleMode.kBrake);
        m_deploy2Config.idleMode(IdleMode.kBrake);
       
        m_deploy1Config.smartCurrentLimit(IntakeConstants.kDeployMotorCurrentLimit);
        m_deploy2Config.smartCurrentLimit(IntakeConstants.kDeployMotorCurrentLimit);
        
        m_deployMotor1.configure(m_deploy1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_deployMotor2.configure(m_deploy1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void spinIntake(AngularVelocity velocity) {
        m_intakeClosedLoopController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }

    public void stopIntake() {
        spinIntake(RPM.of(0));
    }

    public void deployIntake() {
        m_deploy1ClosedLoopController.setSetpoint(
                IntakeConstants.kDeployRotations.in(Rotations),
                ControlType.kPosition);
         m_deploy2ClosedLoopController.setSetpoint(
                IntakeConstants.kDeployRotations.in(Rotations),
                ControlType.kPosition);
    }

    public void retractIntake() {
        m_deploy1ClosedLoopController.setSetpoint(
                IntakeConstants.kRetractRotations.in(Rotations),
                ControlType.kPosition);
        m_deploy1ClosedLoopController.setSetpoint(
                IntakeConstants.kRetractRotations.in(Rotations),
                ControlType.kPosition);
    }

    @Override
    public void periodic() {
        if (m_minLimitSwitch.isPressed()) {
            m_deploy1RelativeEncoder.setPosition(IntakeConstants.kMinExtension.in(Rotations));
            m_deploy2RelativeEncoder.setPosition(IntakeConstants.kMinExtension.in(Rotations));
        } else if (m_maxLimitSwitch.isPressed()) {
            m_deploy1RelativeEncoder.setPosition(IntakeConstants.kMaxExtension.in(Rotations));
            m_deploy2RelativeEncoder.setPosition(IntakeConstants.kMaxExtension.in(Rotations));
        }
    }
}
