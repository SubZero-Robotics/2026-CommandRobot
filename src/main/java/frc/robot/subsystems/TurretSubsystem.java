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
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.utils.PositionBuffer;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax m_turretMotor = new SparkMax(TurretConstants.kMotorId, MotorType.kBrushless);
    private final SparkMaxSim m_simTurretMotor = new SparkMaxSim(m_turretMotor, DCMotor.getNEO(1));

    private final AbsoluteEncoder m_absoluteEncoder = m_turretMotor.getAbsoluteEncoder();
    private final SparkAbsoluteEncoderSim m_simAbsoluteEncoder = m_simTurretMotor.getAbsoluteEncoderSim();

    private final SparkClosedLoopController m_turretClosedLoopController = m_turretMotor.getClosedLoopController();

    private final PositionBuffer m_positionBuffer = new PositionBuffer(TurretConstants.kPositionBufferLength);

    private Supplier<Pose2d> m_robotPoseSupplier;

    private SparkMaxConfig m_pidConfig = new SparkMaxConfig();

    public TurretSubsystem(Supplier<Pose2d> robotPoseSupplier) {
        m_robotPoseSupplier = robotPoseSupplier;

        m_pidConfig.closedLoop.p(TurretConstants.kP).i(TurretConstants.kI).d(TurretConstants.kD);
        m_pidConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_turretMotor.configure(m_pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveToAngle(Angle angle) {
            if (angle.gt(TurretConstants.kMaxAngle)) {
                System.out
                        .println("Angle " + angle + "is bigger than maximum angle " + TurretConstants.kMaxAngle + ".");
                return;
            } else if (angle.lt(TurretConstants.kMinAngle)) {
                System.out.println(
                        "Angle " + angle + "is to smaller than minimum angle " + TurretConstants.kMinAngle + ".");
                return;
            }

            m_turretClosedLoopController.setSetpoint(angle.in(Rotations), ControlType.kPosition);
    }

    public void pointToHeading(Angle heading) {
        Angle robotHeading = Radians.of(m_robotPoseSupplier.get().getRotation().getRadians());
        Angle targetAngle = heading.minus(robotHeading);

        moveToAngle(targetAngle);
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
        System.out.println(m_simAbsoluteEncoder.getPosition());
    }
}