package frc.robot.utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;

public class PositionBuffer {

    private record TimePosition(Angle angle, double timestamp) {
    }

    private final RingBuffer<TimePosition> m_positions;

    public PositionBuffer(int length) {
        m_positions = new RingBuffer<>(length);
    }

    public void pushElement(Angle angle, double delay) {
        try {
            m_positions.push(new TimePosition(angle, Timer.getFPGATimestamp() - delay));
        } catch (Exception e) {
            System.out.println("Ring Buffer Exception: " + e.getMessage());
        }
    }

    public Angle getAngleAtTime(double requestedTime) {
        // A fun little binary search algo
        int high = m_positions.getLength();
        int low = 0;

        // Avoiding overflow (though isn't really necessary here)
        int midpoint = low + (high - low) / 2;

        while (low < high) {
            double timeAtMidpoint;
            try {
                timeAtMidpoint = m_positions.get(midpoint).timestamp;
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
            TimePosition closestTimestamp;

            try {
                closestTimestamp = m_positions.get(midpoint);
            } catch (Exception e) {
                return null;
            }

            double dt;

            try {
                dt = m_positions.get(midpoint).timestamp - requestedTime;
            } catch (Exception e) {
                return null;
            }

            TimePosition nextTimeStamp;

            // Stampted Time 1 should always be less than stampted time 2
            TimePosition stamptedTime1;
            TimePosition stamptedTime2;

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

            double deltaTimeStampted = stamptedTime2.timestamp - stamptedTime1.timestamp;
            double timeSinceStamp1 = requestedTime - stamptedTime1.timestamp;

            Angle deltaTheta = stamptedTime2.angle.minus(stamptedTime2.angle);

            return deltaTheta.times(timeSinceStamp1 / deltaTimeStampted).plus(stamptedTime1.angle);
        }

        try {
            return m_positions.get(midpoint).angle;
        } catch (Exception e) {
            System.out.println("Ring Buffer Exception: " + e.getMessage());
            return null;
        }
    }
}
