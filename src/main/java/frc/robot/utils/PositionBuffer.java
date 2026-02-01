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
        m_positions.push(new TimePosition(angle, Timer.getFPGATimestamp() - delay));
    }

    public Angle getAngleAtTime(double time) {
        // A fun little binary search algo
        int high = m_positions.getLength();
        int low = 0;

        // Avoiding overflow (though isn't really necessary here)
        int midpoint = low + (high - low) / 2;

        while (low < high) {
            double timeAtMidpoint = m_positions.get(midpoint).timestamp;

            if (time > timeAtMidpoint) {
                low = midpoint + 1;
            } else if (time < timeAtMidpoint) {
                high = midpoint;
            }

            midpoint = low + (high - low) / 2;
        }

        Angle angleAtTimeStamp;

        // Linearly interpolate velocity if we aren't on the first/last timestamp
        if (midpoint != 0 && midpoint != m_positions.getLength() - 1) {
            TimePosition closestTimestamp = m_positions.get(midpoint);

            double dt = m_positions.get(midpoint).timestamp - time;
            TimePosition nextTimeStamp = dt > 0 ? m_positions.get(midpoint + 1) : m_positions.get(midpoint - 1);

            Angle daStamped = nextTimeStamp.angle.minus(closestTimestamp.angle);

            // Absolute since in order to preserve the sign, only one part of the fraction
            // can keep its original sign. In this case we always keep the denominator
            // positive
            double dtStamped = Math.abs(nextTimeStamp.timestamp - closestTimestamp.timestamp);

            double percentageTimeTraveled = dt / dtStamped;
            Angle da = daStamped.times(percentageTimeTraveled);

            angleAtTimeStamp = closestTimestamp.angle.plus(da);
        } else {
            angleAtTimeStamp = m_positions.get(midpoint).angle;
        }

        return angleAtTimeStamp;

    }
}
