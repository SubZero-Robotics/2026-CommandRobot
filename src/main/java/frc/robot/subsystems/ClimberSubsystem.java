package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    SparkMax m_climbMotor = new SparkMax(ClimberConstants.kMotorCanId, MotorType.kBrushless);

    RelativeEncoder m_relativeEncoder = m_climbMotor.getEncoder();

    private final SparkLimitSwitch m_minLimitSwitch = m_climbMotor.getReverseLimitSwitch();
    private final SparkLimitSwitch m_maxLimitSwitch = m_climbMotor.getForwardLimitSwitch();
    
    public double GetPosition() {
        return m_relativeEncoder.getPosition();
    }

    public void climbUp() {

        if (!atMax())
        {
            m_climbMotor.set(ClimberConstants.kUpVelocity);
        } 
        else 
        {
            Stop();
        }
    }

    public void climbDown() {

        if (!atMin())
        {
            m_climbMotor.set(ClimberConstants.kDownVelocity);
        } 
        else 
        {
            Stop();
        }
    }

    public boolean atMax() {
        return GetPosition() >= ClimberConstants.kMaxExtension;
    }

    public boolean atMin() {
        return GetPosition() <= ClimberConstants.kMinExtension;
    }

    public void Stop() {
        m_climbMotor.set(0.0);
    }

    public Command ZeroCommand() {
        return new InstantCommand(() -> {
            m_relativeEncoder.setPosition(0);
        });
    }

    @Override
    public void periodic() {
        if (m_minLimitSwitch.isPressed()) {
            m_relativeEncoder.setPosition(ClimberConstants.kLimitMinExtension);
        } else if (m_maxLimitSwitch.isPressed()) {
            m_relativeEncoder.setPosition(ClimberConstants.kLimitMinExtension);
        }

        SmartDashboard.putNumber("Climber Position", GetPosition());
        SmartDashboard.putData("Zero", ZeroCommand());
    }
}
