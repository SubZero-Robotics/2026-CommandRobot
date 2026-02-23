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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NumericalConstants;
import frc.robot.Constants.ShooterConstants;

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

    // Think of this as controlling the turret on a cross-section going from the
    // turret at the angle it is at and the target. Forward is the distance away
    // from the turret while maxZ is the maximum Z height you want the ball to
    // have. endZ is the z position you want the ball to hit once forward is
    // achieved
    public LinearVelocity shootToPose(Distance forward, Distance maxZ, Distance endZ) {
        double forwardMeters = forward.in(Meters);
        double maxZMeters = maxZ.in(Meters);
        double endZMeters = endZ.in(Meters);
        double gravityMpsps = NumericalConstants.kGravity.in(MetersPerSecondPerSecond);

        LinearVelocity vy = MetersPerSecond.of(Math
                .sqrt((2.0 * Math.pow(gravityMpsps, 2.0) * maxZMeters)
                        / (2 * gravityMpsps - 1.0)));

        double vyMps = vy.in(MetersPerSecond);

        LinearVelocity vx = MetersPerSecond
                .of((-2.0 * vyMps * forwardMeters + Math.sqrt(Math.pow(2.0 * vyMps * forwardMeters, 2.0) +
                        8.0 * endZMeters * gravityMpsps * Math.pow(forwardMeters, 2.0))) / (-4.0 * endZMeters));

        double vxMps = vx.in(MetersPerSecond);

        LinearVelocity shooterMuzzleVelocity = MetersPerSecond.of(Math.pow(vxMps, 2.0) + Math.pow(vyMps, 2.0));
        Angle shooterAngle = Radians.of(Math.acos(vxMps / shooterMuzzleVelocity.in(MetersPerSecond)));

        if (shooterMuzzleVelocity.gt(ShooterConstants.kMaxMuzzleVelocity)) {
            System.out.println("Necessary velocity to hit target " + shooterMuzzleVelocity
                    + " beyond max velocity of " + ShooterConstants.kMaxMuzzleVelocity + ".");
            return null;
        }

        MoveHoodToPosition(shooterAngle);

        return shooterMuzzleVelocity;
    }

    // Position between 0 and .55
    public void MoveHoodToPosition(Angle angle) {
        System.out.println("Move hood to position: " + angle);
       
        // get target absolute encoder position. 0 starts in hood min, hood max is .55 (30 degrees of movement)
        var targetPosition = angle.in(Degrees) * (0.55 / 30);
        if(targetPosition < 0 || targetPosition > 0.55){
            System.out.println("Hood target position out of bounds. Target: " + targetPosition);
            return;
        }
        var curentPosition = m_absoluteEncoder.getPosition();
        System.out.println("current hood: " + curentPosition);
        if( curentPosition > 0.55){
            System.out.println("Hood position icorrect for safe movement. Pos: " + curentPosition);
            return;
        }

        m_hoodClosedLoopController.setSetpoint(targetPosition, ControlType.kPosition);
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
