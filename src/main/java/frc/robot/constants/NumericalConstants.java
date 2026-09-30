package frc.robot.constants;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Radians;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;

public final class NumericalConstants {
        public static final double kEpsilon = 1e-6;
        public static final Angle kFullRotation = Radians.of(2.0 * Math.PI);
        public static final Angle kNoRotation = Radians.of(0.0);
        public static final LinearAcceleration kGravity = MetersPerSecondPerSecond.of(9.807);
        public static final Angle kHalfRotation = Degrees.of(180);
        public static final AngularVelocity kNoRotations = RPM.of(0.0);
}
