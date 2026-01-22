package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;


public class TurretSubsystem extends SubsystemBase {
    
    private final SparkMax m_turretMotor = new SparkMax(TurretConstants.kMotorId, MotorType.kBrushless);
    private final SparkMaxSim m_simTurretMotor = new SparkMaxSim(m_turretMotor, DCMotor.getNEO(1));

    private final AbsoluteEncoder m_absoluteEncoder = m_turretMotor.getAbsoluteEncoder();
    private final SparkAbsoluteEncoderSim m_simAbsoluteEncoder = m_simTurretMotor.getAbsoluteEncoderSim();

    private final SparkClosedLoopController m_turretClosedLoopController = m_turretMotor.getClosedLoopController();

    private Supplier<Angle> m_robotAngleSupplier;
    
    public TurretSubsystem(Supplier<Angle> robotAngleSupplier) {
        m_robotAngleSupplier = robotAngleSupplier;
    }

    public Command moveToAngle(Angle angle) {
        return new InstantCommand(() -> {
            if (angle.gt(TurretConstants.kMaxAngle)) {
                System.out.println("Angle " + angle + "is bigger than maximum angle " + TurretConstants.kMaxAngle + ".");
                return; 
                
            } else if (angle.lt(TurretConstants.kMinAngle)) {
                System.out.println("Angle " + angle + "is to smaller than minimum angle " + TurretConstants.kMinAngle + ".");
                return; 
            }

            m_turretClosedLoopController.setSetpoint(angle.in(Rotations), ControlType.kPosition);
        });
    }

    public Angle getRotation() {
        return Rotations.of(m_absoluteEncoder.getPosition());
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {
        m_simTurretMotor.setVelocity(1.0);
        System.out.println(m_simAbsoluteEncoder.getPosition());
    }
}