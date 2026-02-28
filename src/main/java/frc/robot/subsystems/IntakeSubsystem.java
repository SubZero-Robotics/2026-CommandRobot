package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkLimitSwitch;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
    private final SparkMax m_deployMotor1 = new SparkMax(IntakeConstants.kDeployMotor1Id, MotorType.kBrushless);
    private final SparkMax m_deployMotor2 = new SparkMax(IntakeConstants.kDeployMotor2Id, MotorType.kBrushless);

    private final SparkClosedLoopController m_intakeClosedLoopController = m_intakeMotor.getClosedLoopController();
    private final SparkClosedLoopController m_deploy1ClosedLoopController = m_deployMotor1.getClosedLoopController();
    private final SparkClosedLoopController m_deploy2ClosedLoopController = m_deployMotor2.getClosedLoopController();

    private final RelativeEncoder m_deploy1RelativeEncoder = m_deployMotor1.getEncoder();
    private final RelativeEncoder m_deploy2RelativeEncoder = m_deployMotor2.getEncoder();

    private final SparkLimitSwitch m_minLimitSwitch1 = m_deployMotor1.getReverseLimitSwitch();
    private final SparkLimitSwitch m_minLimitSwitch2 = m_deployMotor2.getReverseLimitSwitch();

    private final SparkLimitSwitch m_maxLimitSwitch1 = m_deployMotor1.getForwardLimitSwitch();
    private final SparkLimitSwitch m_maxLimitSwitch2 = m_deployMotor2.getForwardLimitSwitch();

    NetworkTableEntry m_entry = NetworkTableInstance.getDefault().getTable("Intake").getEntry("Intake");

    private SparkMaxConfig m_deploy1Config = new SparkMaxConfig();
    private SparkMaxConfig m_deploy2Config = new SparkMaxConfig();
    private SparkMaxConfig m_intakeConfig = new SparkMaxConfig();

    public IntakeSubsystem() {
        m_deploy1Config.closedLoop.p(IntakeConstants.kP1).i(IntakeConstants.kI1)
                .d(IntakeConstants.kD1);
        m_deploy2Config.closedLoop.p(IntakeConstants.kP2).i(IntakeConstants.kI2)
                .d(IntakeConstants.kD2);
        m_intakeConfig.closedLoop.p(IntakeConstants.kPIn).i(IntakeConstants.kIIn).d(IntakeConstants.kDIn).feedForward
                .kV(IntakeConstants.kFFIn);
        // Apparently there is no absolute encoder :(
        // m_deployConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_deploy1Config.idleMode(IdleMode.kCoast);
        m_deploy2Config.idleMode(IdleMode.kCoast); // TODO: change to brake
        m_intakeConfig.idleMode(IdleMode.kBrake);

        m_deploy1Config.smartCurrentLimit(IntakeConstants.kDeployMotorCurrentLimit);
        m_deploy2Config.smartCurrentLimit(IntakeConstants.kDeployMotorCurrentLimit);
        m_intakeConfig.smartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

        m_deploy1Config.inverted(true);
        m_deployMotor1.configure(m_deploy1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_deployMotor2.configure(m_deploy2Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_intakeMotor.configure(m_intakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void spinIntake(AngularVelocity velocity) {
        m_intakeClosedLoopController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }

    public void stopIntake() {
        spinIntake(RPM.of(0));
    }

    public void deployIntake() {
        Angle deploy1Position = Rotations.of(m_deploy1RelativeEncoder.getPosition());
        Angle deploy2Position = Rotations.of(m_deploy2RelativeEncoder.getPosition());

        if (deploy1Position.lt(IntakeConstants.kMaxExtension)) {
            // m_deploy1ClosedLoopController.setSetpoint(IntakeConstants.kDeployRotations.in(Rotations),
            // ControlType.kPosition);

            m_deploy1ClosedLoopController.setSetpoint(IntakeConstants.kDeployRotations.in(Rotations),
                    ControlType.kPosition);
        } else {
            System.out.println("Attempting to extend when beyond limit hitting limit on deploy motor 1.");
        }

        if (deploy2Position.lt(IntakeConstants.kMaxExtension)) {
            // m_deploy2ClosedLoopController.setSetpoint(IntakeConstants.kDeployRotations.in(Rotations),
            // ControlType.kPosition);
            m_deploy2ClosedLoopController.setSetpoint(IntakeConstants.kDeployRotations.in(Rotations),
                    ControlType.kPosition);
        } else {
            System.out.println("Attempting to extend beyond limit when hitting limit on deploy motor 2.");
        }
    }

    public void retractIntake() {
        Angle deploy1Position = Rotations.of(m_deploy1RelativeEncoder.getPosition());
        Angle deploy2Position = Rotations.of(m_deploy2RelativeEncoder.getPosition());

        if (deploy1Position.gt(IntakeConstants.kMinExtension)) {
            m_deploy1ClosedLoopController.setSetpoint(IntakeConstants.kRetractRotations.in(Rotations),
                    ControlType.kPosition);
        } else {
            System.out.println("Attempting to retract beyond limit when hitting limit on deploy motor 1.");
        }

        if (deploy2Position.gt(IntakeConstants.kMinExtension)) {
            m_deploy2ClosedLoopController.setSetpoint(IntakeConstants.kRetractRotations.in(Rotations),
                    ControlType.kPosition);
        } else {
            System.out.println("Attempting to retract beyond limit when hitting limit on deploy motor 2.");
        }
    }

    @Override
    public void periodic() {
        if (m_minLimitSwitch1.isPressed()) {
            m_deploy1RelativeEncoder.setPosition(IntakeConstants.kMinExtension.in(Rotations));
        }

        if (m_minLimitSwitch2.isPressed()) {
            m_deploy2RelativeEncoder.setPosition(IntakeConstants.kMinExtension.in(Rotations));
        }

        if (m_maxLimitSwitch1.isPressed()) {
            m_deploy1RelativeEncoder.setPosition(IntakeConstants.kMaxExtension.in(Rotations));
        }

        if (m_maxLimitSwitch2.isPressed()) {
            m_deploy2RelativeEncoder.setPosition(IntakeConstants.kMaxExtension.in(Rotations));
        }

        // if (m_minLimitSwitch1.isPressed())
        // System.out.println("Min One");

        // if (m_minLimitSwitch2.isPressed())
        // System.out.println("Min Two");

        // if (m_maxLimitSwitch1.isPressed())
        // System.out.println("Max One");

        // if (m_maxLimitSwitch2.isPressed())
        // System.out.println("Max Two");

        DogLog.log("Deploy motor 1: canID 13 position",
                m_deploy1RelativeEncoder.getPosition());
        DogLog.log("Deploy motor 2: canID 8 position",
                m_deploy2RelativeEncoder.getPosition());

        DogLog.log("Deploy motor can ID 8 setpoint", m_deploy2ClosedLoopController.getSetpoint());
        DogLog.log("Deploy motor can ID 13 setpoint", m_deploy1ClosedLoopController.getSetpoint());

        // DogLog.log("Min limit one", m_minLimitSwitch1.isPressed());
        // DogLog.log("Min limit two", m_minLimitSwitch2.isPressed());

        // DogLog.log("Max limit one", m_maxLimitSwitch1.isPressed());
        // DogLog.log("Max limit two", m_maxLimitSwitch2.isPressed());
    }
}
