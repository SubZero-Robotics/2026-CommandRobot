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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.UtilityFunctions;

public class ShooterSubsystem extends SubsystemBase {

    SparkMax m_shooterMotor = new SparkMax(ShooterConstants.kShooterMotorId, MotorType.kBrushless);
    SparkMax m_hoodMotor = new SparkMax(ShooterConstants.kHoodMotorId, MotorType.kBrushless);

    AbsoluteEncoder m_absoluteEncoder = m_hoodMotor.getAbsoluteEncoder();

    private final SparkClosedLoopController m_shooterClosedLoopController = m_shooterMotor.getClosedLoopController();
    private final SparkClosedLoopController m_hoodClosedLoopController = m_hoodMotor.getClosedLoopController();

    private final SparkMaxConfig m_shooterConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_hoodConfig = new SparkMaxConfig();

    Angle m_targetAngle = Degrees.of(0.0);

    public ShooterSubsystem() {

        m_shooterConfig.closedLoop
                .p(ShooterConstants.kShooterP)
                .i(ShooterConstants.kShooterI)
                .d(ShooterConstants.kShooterD)
                .feedForward.kV(ShooterConstants.kShooterFF);

        m_hoodConfig.closedLoop
                .p(ShooterConstants.kHoodP)
                .i(ShooterConstants.kHoodI)
                .d(ShooterConstants.kHoodD);
        m_hoodConfig.smartCurrentLimit(ShooterConstants.kHoodSmartCurrentLimit);
        m_hoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_hoodConfig.absoluteEncoder.inverted(true);
        m_hoodConfig.inverted(true);

        // .6 rotations = 30 degrees
        // 1 rotation = 50 degrees
        m_hoodConfig.absoluteEncoder.positionConversionFactor(1);
        m_hoodConfig.encoder.positionConversionFactor(1);

        m_shooterMotor.configure(m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_hoodMotor.configure(m_hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Position between 0 and .55
    public void MoveHoodToPosition(Angle angle) {
        System.out.println("Move hood to position: " + angle);

        // get target absolute encoder position. 0 starts in hood min, hood max is .55
        // (30 degrees of movement)
        var targetPosition = angle.in(Degrees) * (ShooterConstants.kHoodDegreeConversionFactor);
        if (targetPosition < 0 || targetPosition > ShooterConstants.kHoodMaxAbsolutePosition) {
            System.out.println("Hood target position out of bounds. Target: " + targetPosition);
            return;
        }
        var curentPosition = m_absoluteEncoder.getPosition();
        System.out.println("current hood: " + curentPosition);
        if (curentPosition > ShooterConstants.kHoodMaxAbsolutePosition) {
            System.out.println("Hood position icorrect for safe movement. Pos: " + curentPosition);
            return;
        }

        m_hoodClosedLoopController.setSetpoint(targetPosition, ControlType.kPosition);
    }

    public void Spin(AngularVelocity shootSpeedVelocity) {
        m_shooterClosedLoopController.setSetpoint(shootSpeedVelocity.in(RPM), ControlType.kVelocity);
    }

    public void Stop() {
        Spin(RPM.of(0));
    }

    public Angle GetHoodAngle() {
        return Degrees.of(m_absoluteEncoder.getPosition() / ShooterConstants.kHoodDegreeConversionFactor);
    }

    public boolean AtTarget() {
        return UtilityFunctions.angleDiff(GetHoodAngle(), m_targetAngle).abs(Degrees) < ShooterConstants.kHoodTolerence
                .in(Degrees);
    }

    @Override
    public void periodic() {

    }
}
