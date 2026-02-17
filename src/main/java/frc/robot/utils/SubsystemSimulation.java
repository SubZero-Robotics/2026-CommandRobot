package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class SubsystemSimulation {
    private Mechanism2d m_mech;
    private MechanismRoot2d m_root;

    // Simulation for elevator
    public SubsystemSimulation(String name, Distance width, Distance height, Distance startDistance) {
        m_mech = new Mechanism2d(width.in(Meters), height.in(Meters));
        m_root = m_mech.getRoot(name, width.in(Meters) / 2, 0.0);
        m_root.append(new MechanismLigament2d(name, startDistance.in(Meters), Math.PI));
    }

    // Simulation for arm
    public SubsystemSimulation(String name, Distance width, Distance height, Distance length, Angle startAngle) {
        m_mech = new Mechanism2d(width.in(Meters), height.in(Meters));
        m_root = m_mech.getRoot(name, width.in(Meters) / 2, 0.0);
        m_root.append(new MechanismLigament2d(name, length.in(Meters), startAngle.in(Radians)));
    }
}