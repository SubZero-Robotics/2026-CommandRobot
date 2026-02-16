package frc.robot.utils;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

public class PositionBuffer {

    TurretPosition m_position;

    private final RingBuffer<TurretPosition> m_positions;

    public PositionBuffer(int length) {
        m_positions = new RingBuffer<>(length);
    }

    public void pushElement(Angle angle, AngularVelocity velocity, double delay) {
        try {
            m_positions.push(new TurretPosition(angle, velocity, Timer.getFPGATimestamp() - delay));
        } catch (Exception e) {
            System.out.println("Ring Buffer Exception: " + e.getMessage());
        }
    }

    private double interpolate(double x1, double x2, double y1, double y2, double x) {
        double dsx = x2 - x1;
        double dy = y2 - y1;
        double dx = x - x1;

        return (dx / dsx) * dy + y1;
    }

    public TurretPosition getAngleAtTime(double requestedTime) {
        // A fun little binary search algo
        int high = m_positions.getLength();
        int low = 0;

        // Avoiding overflow (though isn't really necessary here)
        int midpoint = low + (high - low) / 2;

        TurretPosition midpointTurretPosition = null;
        double timeAtMidpoint = 0.0;

        while (low < high) {
            try {
                midpointTurretPosition = m_positions.get(midpoint);
                timeAtMidpoint = midpointTurretPosition.timestamp();
            } catch (Exception e) {
                System.out.println("Ring Buffer Exception: " + e.getMessage());
                return null;
            }

            if (requestedTime > timeAtMidpoint) {
                low = midpoint + 1;
            } else if (requestedTime < timeAtMidpoint) {
                high = midpoint;
            }

            midpoint = low + (high - low) / 2;
        }

        // Linearly interpolate velocity if we aren't on the first/last timestamp
        if (midpoint != 0 && midpoint != m_positions.getLength() - 1) {
            double dt;

            try {
                dt = timeAtMidpoint - requestedTime;
            } catch (Exception e) {
                return null;
            }

            // Stampted Time 1 should always be less than stampted time 2
            TurretPosition firstTurretPosition;
            TurretPosition secondTurretPosition;

            if (dt > 0) {
                try {
                    secondTurretPosition = m_positions.get(midpoint + 1);
                    firstTurretPosition = midpointTurretPosition;
                } catch (Exception e) {
                    System.out.println("Ring Buffer Exception: " + e.getMessage());
                    return null;
                }
            } else {
                try {
                    firstTurretPosition = m_positions.get(midpoint - 1);
                    secondTurretPosition = midpointTurretPosition;
                } catch (Exception e) {
                    System.out.println("Ring Buffer Exception: " + e.getMessage());
                    return null;
                }
            }

            Angle angle = Radians.of(interpolate(firstTurretPosition.timestamp(), secondTurretPosition.timestamp(),
                    firstTurretPosition.angle().in(Radians), secondTurretPosition.angle().in(Radians), requestedTime));

            AngularVelocity velocity = RPM
                    .of(interpolate(firstTurretPosition.timestamp(), secondTurretPosition.timestamp(),
                            firstTurretPosition.velocity().in(RPM), secondTurretPosition.velocity().in(RPM),
                            requestedTime));

            return new TurretPosition(angle, velocity, requestedTime);
        }

        try {
            return m_positions.get(midpoint);
        } catch (Exception e) {
            System.out.println("Ring Buffer Exception: " + e.getMessage());
            return null;
        }
    }
}
