package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.utils.PositionBuffer;
import frc.robot.utils.TurretPosition;
import frc.robot.utils.UtilityFunctions;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax m_turretMotor = new SparkMax(TurretConstants.kMotorId, MotorType.kBrushless);

    private final AbsoluteEncoder m_absoluteEncoder = m_turretMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController m_turretClosedLoopController = m_turretMotor.getClosedLoopController();

    private final PositionBuffer m_positionBuffer = new PositionBuffer(TurretConstants.kPositionBufferLength);

    private SparkMaxConfig m_config = new SparkMaxConfig();

    private Mechanism2d m_simMech = new Mechanism2d(4.0, 4.0);
    private MechanismRoot2d m_mechRoot = m_simMech.getRoot("Turret Root", 2.0, 2.0);

    private MechanismLigament2d m_simLigament = new MechanismLigament2d("Turret", 2, 0);

    private MechanismLigament2d m_min1 = new MechanismLigament2d("min1", 2, 0);
    private MechanismLigament2d m_max1 = new MechanismLigament2d("max1", 2, 0);
    private MechanismLigament2d m_min2 = new MechanismLigament2d("min2", 2, 0);
    private MechanismLigament2d m_max2 = new MechanismLigament2d("max2", 2, 0);

    private Angle robotRotation;

    private Angle m_targetAngle = Degrees.of(0.0);
    private Angle m_simAngle = Degrees.of(0.0);

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

        m_min1 = m_mechRoot.append(m_min1);
        m_min1.setColor(new Color8Bit("#FF00FF"));

        m_max1 = m_mechRoot.append(m_max1);
        m_max1.setColor(new Color8Bit("#FF00FF"));

        m_min2 = m_mechRoot.append(m_min2);
        m_min2.setColor(new Color8Bit("#FF0000"));

        m_max2 = m_mechRoot.append(m_max2);
        m_max2.setColor(new Color8Bit("#FF0000"));

        robotRotation = Degrees.of(0);
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
        robotRotation = angle;
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

    public LinearVelocity getMuzzleVelocityAtHoodAngle() {
        return MetersPerSecond.of(5);
    }

    @Override
    public void simulationPeriodic() {
        m_simLigament.setAngle(m_targetAngle.plus(robotRotation).in(Degrees));
        m_min1.setAngle(TurretConstants.kHubMinAngle1.plus(robotRotation).in(Degrees));
        m_max1.setAngle(TurretConstants.kHubMaxAngle1.plus(robotRotation).in(Degrees));
        m_min2.setAngle(TurretConstants.kHubMinAngle2.plus(robotRotation).in(Degrees));
        m_max2.setAngle(TurretConstants.kHubMaxAngle2.plus(robotRotation).in(Degrees));
        SmartDashboard.putData("Turret Rotation", m_simMech);
    }

    public boolean atTarget() {
        return UtilityFunctions.angleDiff(m_targetAngle, getRotation())
                .abs(Degrees) < TurretConstants.kTurretAngleTolerance.in(Degrees);
    }
}