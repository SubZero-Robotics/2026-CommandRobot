package frc.robot.utils;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public record TurretPosition(Angle angle, AngularVelocity velocity,
                double timestamp) implements Interpolatable<TurretPosition> {

        @Override
        public TurretPosition interpolate(TurretPosition end, double fraction) {
                return new TurretPosition(UtilityFunctions.WrapAngle(new Rotation2d(angle)
                                .interpolate(new Rotation2d(end.angle), fraction).getMeasure()),
                                RadiansPerSecond.of(MathUtil.interpolate(velocity.in(RadiansPerSecond),
                                                end.velocity.in(RadiansPerSecond), fraction)),
                                MathUtil.interpolate(timestamp, end.timestamp, fraction));
        }

        /** Net field-relative angular speed of the camera, including chassis rotation. */
        public boolean isWithinVisionSpeed(AngularVelocity chassisVelocity, AngularVelocity limit) {
                return velocity.plus(chassisVelocity).abs(RadiansPerSecond) <= limit.in(RadiansPerSecond);
        }
}
