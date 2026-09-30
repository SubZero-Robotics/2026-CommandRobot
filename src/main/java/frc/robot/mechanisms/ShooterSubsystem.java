package frc.robot.mechanisms;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;

import com.revrobotics.spark.config.SparkMaxConfig;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import frc.robot.constants.CANConstants;
import frc.robot.constants.NumericalConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem implements Mechanism {

    SparkMax m_shooterMotor = new SparkMax(CANConstants.kCanPort, ShooterConstants.kShooterMotorId, MotorType.kBrushless);
    // SparkMax m_hoodMotor = new SparkMax(ShooterConstants.kHoodMotorId,
    // MotorType.kBrushless);

    // AbsoluteEncoder m_absoluteEncoder = m_hoodMotor.getAbsoluteEncoder();
    RelativeEncoder m_shooterRelativeEncoder = m_shooterMotor.getEncoder();

    private final SparkClosedLoopController m_shooterClosedLoopController = m_shooterMotor.getClosedLoopController();
    // private final SparkClosedLoopController m_hoodClosedLoopController =
    // m_hoodMotor.getClosedLoopController();

    private final SparkMaxConfig m_shooterConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_hoodConfig = new SparkMaxConfig();

    Angle m_targetAngle = Degrees.of(0.0);
    AngularVelocity m_targetVelocity = RPM.of(0);

    public ShooterSubsystem() {

        m_shooterConfig.closedLoop
                .p(ShooterConstants.kShooterP)
                .i(ShooterConstants.kShooterI)
                .d(ShooterConstants.kShooterD).feedForward.kV(ShooterConstants.kShooterFF);

        m_shooterConfig.idleMode(IdleMode.kCoast);
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
        // REVLib 2027 removed conversion factors. The hood used a factor of 1, which is the
        // native unit, so no conversion is needed.

        m_shooterMotor.configure(m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // m_hoodMotor.configure(m_hoodConfig, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);

        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    // Position between 0 and .55. Disabled hood motor
    public void MoveHoodToPosition(Angle angle) {
        // System.out.println("Move hood to position: " + angle);

        // get target absolute encoder position. 0 starts in hood min, hood max is .55
        // (30 degrees of movement)
        var targetPosition = angle.in(Degrees) * (ShooterConstants.kHoodDegreeConversionFactor);
        if (targetPosition < 0 || targetPosition > ShooterConstants.kHoodMaxAbsolutePosition) {
            System.out.println("Hood target position out of bounds. Target: " + targetPosition);
            return;
        }

        // var currentPosition = m_absoluteEncoder.getPosition();

        // if (currentPosition > ShooterConstants.kHoodMaxAbsolutePosition) {
        // System.out.println("Hood position incorrect for safe movement. Pos: " +
        // currentPosition);
        // return;
        // }

        // m_hoodClosedLoopController.setSetpoint(targetPosition,
        // ControlType.kPosition);
    }

    public void Spin(AngularVelocity shootSpeedVelocity) {
        m_targetVelocity = shootSpeedVelocity;
        m_shooterClosedLoopController.setSetpoint(shootSpeedVelocity.in(RPM), ControlType.kVelocity);
    }

    public void Stop() {
        m_shooterClosedLoopController.setSetpoint(0, ControlType.kVelocity);
    }

    public Angle GetHoodAngle() {
        // return Degrees.of(m_absoluteEncoder.getPosition() /
        // ShooterConstants.kHoodDegreeConversionFactor);
        return NumericalConstants.kNoRotation;
    }

    public boolean AtHoodTarget() {
        // return m_hoodClosedLoopController.isAtSetpoint();
        return true;
    }

    public boolean AtWheelVelocityTarget() {
        return RPM.of(m_shooterClosedLoopController.getSetpoint().get())
                .minus(RPM.of(m_shooterRelativeEncoder.getVelocity().get()))
                .abs(RPM) < ShooterConstants.kShooterVelocityTolerance.in(RPM);

        // return true;
    }

    public void periodic() {
        DogLog.log("Motor velocity setpoint", m_shooterClosedLoopController.getSetpoint().get());
    }
}
