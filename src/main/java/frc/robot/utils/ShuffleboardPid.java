package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleboardPid extends PIDController {

    private ShuffleboardTab m_tab;
    private GenericEntry m_pEntry;
    private GenericEntry m_iEntry;
    private GenericEntry m_dEntry;

    private double m_curP;
    private double m_curI;
    private double m_curD;

    public ShuffleboardPid(double initialP, double initialI, double initialD, String name) {
        super(initialP, initialI, initialD);

        m_curP = initialP;
        m_curI = initialI;
        m_curD = initialD;

        m_tab = Shuffleboard.getTab(name);

        m_pEntry = m_tab.add("P", initialP).getEntry();
        m_iEntry = m_tab.add("I", initialI).getEntry();
        m_dEntry = m_tab.add("D", initialD).getEntry();
    }

    // Must be called every periodic loop
    public void periodic() {
        double entryP = m_pEntry.getDouble(Double.NaN);
        double entryI = m_iEntry.getDouble(Double.NaN);
        double entryD = m_dEntry.getDouble(Double.NaN);

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
        return m_pEntry.getDouble(0.0) + ", " + m_iEntry.getDouble(0.0) + ", " + m_dEntry.getDouble(0.0);
    }
}
