package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.utils.ShuffleboardPid;

public class ShooterSubsystem extends SubsystemBase {

    SparkMax m_shooterMotor = new SparkMax(ShooterConstants.kShooterMotorId, MotorType.kBrushless);
    SparkMax m_hoodMotor = new SparkMax(ShooterConstants.kHoodMotorId, MotorType.kBrushless);

    AbsoluteEncoder m_absoluteEncoder = m_hoodMotor.getAbsoluteEncoder();

    private final SparkClosedLoopController m_shooterClosedLoopController = m_shooterMotor.getClosedLoopController();
    private final SparkClosedLoopController m_hoodClosedLoopController = m_hoodMotor.getClosedLoopController();

    private final SparkMaxConfig m_shooterConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_hoodConfig = new SparkMaxConfig();

    public ShooterSubsystem() {

        m_shooterConfig.closedLoop
                .p(ShooterConstants.kShooterP)
                .i(ShooterConstants.kShooterI)
                .d(ShooterConstants.kShooterD);

        m_hoodConfig.closedLoop
                .p(ShooterConstants.kHoodP)
                .i(ShooterConstants.kHoodI)
                .d(ShooterConstants.kHoodD);
        m_hoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        m_shooterMotor.configure(m_shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_hoodMotor.configure(m_hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void Aim(Angle angle) {
        m_hoodClosedLoopController.setSetpoint(angle.in(Rotations) / ShooterConstants.kHoodGearRatio,
                ControlType.kPosition);
    }

    public void Spin(AngularVelocity shootSpeedVelocity) {
        m_shooterClosedLoopController.setSetpoint(shootSpeedVelocity.in(RPM), ControlType.kVelocity);
    }

    public void StopShooting() {
        Spin(RPM.of(0));
    }

    @Override
    public void periodic() {

    }
}
