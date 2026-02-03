package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem {
    private final SparkMax m_IntakeMotor = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);
    private final SparkMax m_DeployMotor = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);

    private final AbsoluteEncoder m_absoluteEncoder = m_DeployMotor.getAbsoluteEncoder();

    private final SparkClosedLoopController m_IntakeClosedLoopController = m_IntakeMotor.getClosedLoopController();
    private final SparkClosedLoopController m_DeployClosedLoopController = m_IntakeMotor.getClosedLoopController();

    private SparkMaxConfig m_PidConfig = new SparkMaxConfig();

    public IntakeSubsystem() {

        m_PidConfig.closedLoop.p(IntakeConstants.kP).i(IntakeConstants.kI)
                .d(IntakeConstants.kD);
        m_PidConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_DeployMotor.configure(m_PidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void SpinIntake(AngularVelocity velocity) {
        m_IntakeClosedLoopController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }

     public void StopIntake() {
        SpinIntake(RPM.of(0));
    }

    public void DeployPosition() {
    }
}


