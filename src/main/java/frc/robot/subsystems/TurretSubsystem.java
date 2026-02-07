package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.utils.PositionBuffer;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax m_turretMotor = new SparkMax(TurretConstants.kMotorId, MotorType.kBrushless);
    private final SparkMaxSim m_simTurretMotor = new SparkMaxSim(m_turretMotor, DCMotor.getNEO(1));

    private final AbsoluteEncoder m_absoluteEncoder = m_turretMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController m_turretClosedLoopController = m_turretMotor.getClosedLoopController();

    private final PositionBuffer m_positionBuffer = new PositionBuffer(TurretConstants.kPositionBufferLength);

    private Supplier<Pose2d> m_robotPoseSupplier;

    private SparkMaxConfig m_config = new SparkMaxConfig();

    public TurretSubsystem(Supplier<Pose2d> robotPoseSupplier) {
        m_robotPoseSupplier = robotPoseSupplier;

        m_config.closedLoop.p(TurretConstants.kP).i(TurretConstants.kI).d(TurretConstants.kD);
        m_config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_config.smartCurrentLimit(TurretConstants.kSmartCurrentLimit);
        m_config.idleMode(IdleMode.kBrake);
        m_turretMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveToAngle(Angle angle) {

        angle = wrapAngle(angle);

        if (angle.gt(TurretConstants.kMaxAngle)) {
            System.out
                    .println("Angle " + angle + "is bigger than maximum angle " +
                            TurretConstants.kMaxAngle + ".");
            return;
        } else if (angle.lt(TurretConstants.kMinAngle)) {
            System.out.println(
                    "Angle " + angle + "is to smaller than minimum angle " +
                            TurretConstants.kMinAngle + ".");
            return;
        }

        // System.out.println("Target angle is " + angle);
        m_turretClosedLoopController.setSetpoint(angle.in(Rotations), ControlType.kPosition);
    }

    public void pointToHeading(Angle heading) {
        // moveToAngle(
        // heading.minus(Radians.of(m_robotPoseSupplier.get().getRotation().getRadians()
        // - heading.magnitude())));
        moveToAngle(Radians.of(m_robotPoseSupplier.get().getRotation().getRadians() - heading.in(Radians)));
    }

    public Angle getRotation() {
        return Rotations.of(m_absoluteEncoder.getPosition());
    }

    public Angle getRotationAtTime(double timestamp) {
        return m_positionBuffer.getAngleAtTime(timestamp);
    }

    @Override
    public void periodic() {
    }

    // Connected to another periodic loop that runs quicker than 0.02 seconds
    public void pushCurrentEncoderReading() {
        m_positionBuffer.pushElement(Rotations.of(m_absoluteEncoder.getPosition()),
                TurretConstants.kEncoderReadingDelay.in(Seconds));
    }

    @Override
    public void simulationPeriodic() {
        m_simTurretMotor.setVelocity(1.0);
        // System.out.println(m_simAbsoluteEncoder.getPosition());
    }

    private Angle wrapAngle(Angle angle) {
        Angle wrap = Radians.of((angle.in(Radians) % TurretConstants.kFullRotation.in(Radians)));
        if (wrap.lt(TurretConstants.kNoRotation)) {
            wrap = wrap.plus(TurretConstants.kFullRotation);
        }

        return wrap;
    }
}