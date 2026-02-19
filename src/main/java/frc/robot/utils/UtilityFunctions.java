package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.NumericalConstants;

public final class UtilityFunctions {
    public static Angle WrapAngle(Angle angle) {
        return Radians.of(((angle.in(Radians) % NumericalConstants.kFullRotation.in(Radians)) + NumericalConstants.kFullRotation.in(Radians)) % NumericalConstants.kFullRotation
                .in(Radians));
    }

    public static Angle WrapTo180(Angle angle) {
        Angle wrapped = WrapAngle(angle);

        if (wrapped.gt(Degrees.of(180))) {
            wrapped = wrapped.minus(NumericalConstants.kFullRotation);
        }

        return wrapped;
    }
}
