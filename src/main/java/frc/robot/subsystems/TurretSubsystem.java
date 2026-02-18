package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NumericalConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utils.PositionBuffer;
import frc.robot.utils.TurretPosition;
import frc.robot.utils.UtilityFunctions;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax m_turretMotor = new SparkMax(TurretConstants.kMotorId, MotorType.kBrushless);
    private final SparkMaxSim m_simTurretMotor = new SparkMaxSim(m_turretMotor, DCMotor.getNEO(1));

    private final AbsoluteEncoder m_absoluteEncoder = m_turretMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController m_turretClosedLoopController = m_turretMotor.getClosedLoopController();

    private final PositionBuffer m_positionBuffer = new PositionBuffer(TurretConstants.kPositionBufferLength);

    private SparkMaxConfig m_config = new SparkMaxConfig();

    Mechanism2d m_simMech = new Mechanism2d(4.0, 4.0);
    MechanismRoot2d m_mechRoot = m_simMech.getRoot("Turret Root", 2.0, 2.0);
    MechanismLigament2d m_simLigament = new MechanismLigament2d("Turret", 2, 0);

    Angle m_targetAngle = Degrees.of(0.0);
    Angle m_simAngle = Degrees.of(0.0);

    public TurretSubsystem() {
        m_config.closedLoop.p(TurretConstants.kP).i(TurretConstants.kI).d(TurretConstants.kD);
        m_config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_config.smartCurrentLimit(TurretConstants.kSmartCurrentLimit);
        m_config.idleMode(IdleMode.kBrake);
        m_config.absoluteEncoder.inverted(true);
        m_config.inverted(true);
        m_turretMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_positionBuffer.pushElement(UtilityFunctions.WrapAngle(Rotations.of(m_absoluteEncoder.getPosition())),
                RPM.of(m_absoluteEncoder.getVelocity()),
                TurretConstants.kEncoderReadingDelay.in(Seconds));

        m_simLigament = m_mechRoot.append(m_simLigament);
    }

    public void moveToAngle(Angle angle) {

        angle = UtilityFunctions.WrapAngle(angle);

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
        m_targetAngle = angle;
        m_turretClosedLoopController.setSetpoint(angle.in(Rotations), ControlType.kPosition);
    }

    public Angle getRotation() {
        return UtilityFunctions.WrapAngle(Rotations.of(m_absoluteEncoder.getPosition()));
    }

    public void addDriveHeading(Angle angle) {
        m_simAngle = m_targetAngle.plus(angle);
    }

    public TurretPosition getRotationAtTime(double timestamp) {
        return m_positionBuffer.getAngleAtTime(timestamp);
        // return new TurretPosition(getRotation(), RotationsPerSecond.of(0.0),
        // timestamp);
    }

    @Override
    public void periodic() {
        m_positionBuffer.pushElement(UtilityFunctions.WrapAngle(Rotations.of(m_absoluteEncoder.getPosition())),
                RPM.of(m_absoluteEncoder.getVelocity()),
                TurretConstants.kEncoderReadingDelay.in(Seconds));
    }

    // Connected to another periodic loop that runs quicker than 0.02 seconds
    public void pushCurrentEncoderReading() {
        // m_positionBuffer.pushElement(UtilityFunctions.WrapAngle(Rotations.of(m_absoluteEncoder.getPosition())),
        // RPM.of(m_absoluteEncoder.getVelocity()),
        // TurretConstants.kEncoderReadingDelay.in(Seconds));
    }

    @Override
    public void simulationPeriodic() {
        m_simLigament.setAngle(m_simAngle.in(Degrees));
        SmartDashboard.putData("Turret Rotation", m_simMech);
    }
}