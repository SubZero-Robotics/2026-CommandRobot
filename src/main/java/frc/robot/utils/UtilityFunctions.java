package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.NumericalConstants;

public final class UtilityFunctions {
    public static Angle WrapAngle(Angle angle) {
        return Radians.of(((angle.in(Radians) % NumericalConstants.kFullRotation.in(Radians))
                + NumericalConstants.kFullRotation.in(Radians)) % NumericalConstants.kFullRotation
                        .in(Radians));
    }

    public static Angle WrapTo180(Angle angle) {
        Angle wrapped = WrapAngle(angle);

        if (wrapped.gt(Degrees.of(180))) {
            wrapped = wrapped.minus(NumericalConstants.kFullRotation);
        }

        return wrapped;
    }

    public static double interpolate(double x1, double x2, double y1, double y2, double x) {
        double dsx = x2 - x1;
        double dy = y2 - y1;
        double dx = x - x1;

        return (dx / Math.abs(dsx) > 0.0 ? dsx : NumericalConstants.kEpsilon) * dy + y1;
    }

    public static Angle angleDiff(Angle a1, Angle a2) {
        Angle diff = a1.minus(a2);
        return WrapTo180(diff);
    }
}
