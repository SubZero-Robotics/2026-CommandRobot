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

        while (low < high) {
            double timeAtMidpoint;
            try {
                timeAtMidpoint = m_positions.get(midpoint).timestamp();
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
            TurretPosition closestTimestamp;

            try {
                closestTimestamp = m_positions.get(midpoint);
            } catch (Exception e) {
                return null;
            }

            double dt;

            try {
                dt = m_positions.get(midpoint).timestamp() - requestedTime;
            } catch (Exception e) {
                return null;
            }

            TurretPosition nextTimeStamp;

            // Stampted Time 1 should always be less than stampted time 2
            TurretPosition stamptedTime1;
            TurretPosition stamptedTime2;

            if (dt > 0) {
                try {
                    stamptedTime2 = m_positions.get(midpoint + 1);
                    stamptedTime1 = closestTimestamp;
                } catch (Exception e) {
                    System.out.println("Ring Buffer Exception: " + e.getMessage());
                    return null;
                }
            } else {
                try {
                    stamptedTime1 = m_positions.get(midpoint - 1);
                    stamptedTime2 = closestTimestamp;
                } catch (Exception e) {
                    System.out.println("Ring Buffer Exception: " + e.getMessage());
                    return null;
                }
            }

            Angle angle = Radians.of(interpolate(stamptedTime1.timestamp(), stamptedTime2.timestamp(),
                    stamptedTime1.angle().in(Radians), stamptedTime2.angle().in(Radians), requestedTime));

            AngularVelocity velocity = RPM
                    .of(interpolate(stamptedTime1.timestamp(), stamptedTime2.timestamp(),
                            stamptedTime1.velocity().in(RPM), stamptedTime2.velocity().in(RPM), requestedTime));

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
