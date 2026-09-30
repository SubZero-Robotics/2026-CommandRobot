package frc.robot.utils;

import org.wpilib.math.controller.PIDController;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

// Shuffleboard was removed in WPILib 2027, so the P, I, and D values are published as
// tunables under the given name instead of in a Shuffleboard tab.
public class ShuffleboardPid extends PIDController {

    private TunableDouble m_pEntry;
    private TunableDouble m_iEntry;
    private TunableDouble m_dEntry;

    private double m_curP;
    private double m_curI;
    private double m_curD;

    public ShuffleboardPid(double initialP, double initialI, double initialD, String name) {
        super(initialP, initialI, initialD);

        m_curP = initialP;
        m_curI = initialI;
        m_curD = initialD;

        m_pEntry = Tunables.addDouble(name + "/P", initialP);
        m_iEntry = Tunables.addDouble(name + "/I", initialI);
        m_dEntry = Tunables.addDouble(name + "/D", initialD);
    }

    // Must be called every periodic loop
    public void periodic() {
        double entryP = m_pEntry.get();
        double entryI = m_iEntry.get();
        double entryD = m_dEntry.get();

        if (entryP != m_curP) {
            super.setP(entryP);
            m_curP = entryP;
        }

        if (entryI != m_curI) {
            super.setI(entryI);
            m_curI = entryI;
        }

        if (entryD != m_curD) {
            super.setD(entryD);
            m_curD = entryD;
        }
    }

    @Override
    public String toString() {
        return m_pEntry.get() + ", " + m_iEntry.get() + ", " + m_dEntry.get();
    }
}
