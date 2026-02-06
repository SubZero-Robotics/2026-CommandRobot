package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem {
    private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);
    private final SparkMax m_deployMotor = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);

    private final SparkClosedLoopController m_intakeClosedLoopController = m_intakeMotor.getClosedLoopController();
    private final SparkClosedLoopController m_deployClosedLoopController = m_intakeMotor.getClosedLoopController();

    private SparkMaxConfig m_PidConfig = new SparkMaxConfig();

    public IntakeSubsystem() {

        m_PidConfig.closedLoop.p(IntakeConstants.kP).i(IntakeConstants.kI)
                .d(IntakeConstants.kD);
        m_PidConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_deployMotor.configure(m_PidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void spinIntake(AngularVelocity velocity) {
        m_intakeClosedLoopController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }

    public void stopIntake() {
        spinIntake(RPM.of(0));
    }

    public void toDeployPosition() {
        m_deployClosedLoopController.setSetpoint(
                IntakeConstants.kDeployRotations.in(Rotations),
                ControlType.kPosition);
    }

    public void retractIntake() {
        m_deployClosedLoopController.setSetpoint(
                IntakeConstants.kRetractRotations.in(Rotations),
                ControlType.kPosition);
    }
}
