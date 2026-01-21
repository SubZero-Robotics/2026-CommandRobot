package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    
    SparkMax m_shooterMotor = new SparkMax(ShooterConstants.kMotorId, MotorType.kBrushless);

    AbsoluteEncoder m_absoluteEncoder = m_shooterMotor.getAbsoluteEncoder();

    private final SparkClosedLoopController m_shooterClosedLoopController = m_shooterMotor.getClosedLoopController();

    public ShooterSubsystem() {

    }

    public void Spin(AngularVelocity shootSpeedVelocity) {
        //activates shooter to shoot balls at specific velocity

        m_shooterClosedLoopController.setSetpoint(shootSpeedVelocity.in(RPM), ControlType.kVelocity);
    }

    public void Stop() {
        Spin(RPM.of(0));
    }

    @Override
    public void periodic() {
        
    }
}
