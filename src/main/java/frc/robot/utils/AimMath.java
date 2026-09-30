package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.ToDoubleFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;

/** Single-pass moving-shot solver. Inputs use the pose estimator's field frame. */
public final class AimMath {
    private final ExtrapolatingTable flightTime;
    private final ExtrapolatingTable wheelSpeed;
    private final ExtrapolatingTable hoodAngle;
    private final double stationarySpeed;

    public AimMath(ShootingEntry[] entries, LinearVelocity stationaryThreshold) {
        if (entries == null || entries.length < 2) {
            throw new IllegalArgumentException("Shooting table requires at least two entries");
        }
        double previous = Double.NEGATIVE_INFINITY;
        for (ShootingEntry entry : entries) {
            if (entry == null || entry.distance() == null || entry.timeOfFlight() == null
                    || entry.wheelVelocity() == null || entry.shooterAngle() == null) {
                throw new IllegalArgumentException("Shooting table has a missing required value");
            }
            double distance = entry.distance().in(Meters);
            if (!Double.isFinite(distance) || distance <= previous
                    || !Double.isFinite(entry.timeOfFlight().in(Seconds))
                    || !Double.isFinite(entry.wheelVelocity().in(RadiansPerSecond))
                    || !Double.isFinite(entry.shooterAngle().in(Radians))) {
                throw new IllegalArgumentException("Shooting table must be finite and strictly ordered by distance");
            }
            previous = distance;
        }
        stationarySpeed = stationaryThreshold.in(MetersPerSecond);
        if (!Double.isFinite(stationarySpeed) || stationarySpeed < 0.0) {
            throw new IllegalArgumentException("Stationary threshold must be finite and nonnegative");
        }
        flightTime = new ExtrapolatingTable(entries, e -> e.timeOfFlight().in(Seconds));
        wheelSpeed = new ExtrapolatingTable(entries, e -> e.wheelVelocity().in(RadiansPerSecond));
        hoodAngle = new ExtrapolatingTable(entries, e -> e.shooterAngle().in(Radians));
    }

    public TargetSolution solve(Pose2d robotPose, Translation2d turretOffset, Translation2d fieldTarget,
            ChassisSpeeds fieldRelativeSpeeds) {
        Translation2d toTarget = fieldTarget.minus(RobotGeometry.turretPosition(robotPose, turretOffset));
        double distance = toTarget.getNorm();
        Rotation2d hubHeading = RobotGeometry.bearing(toTarget);
        Rotation2d phi = Rotation2d.kZero;

        // These vector components have units of meters/second, not position.
        Translation2d fieldVelocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond,
                fieldRelativeSpeeds.vyMetersPerSecond);
        if (fieldVelocity.getNorm() > stationarySpeed) {
            double seconds = flightTime.get(distance);
            Translation2d hubFrameVelocity = fieldVelocity.rotateBy(hubHeading.unaryMinus());
            Translation2d virtualTarget = new Translation2d(distance, 0.0)
                    .minus(hubFrameVelocity.times(seconds));
            distance = virtualTarget.getNorm();
            // Existing commands subtract phi from the uncompensated hub bearing.
            phi = RobotGeometry.bearing(virtualTarget).unaryMinus();
        }

        return new TargetSolution(Radians.of(hoodAngle.get(distance)),
                RadiansPerSecond.of(wheelSpeed.get(distance)), phi.getMeasure(), Meters.of(distance),
                hubHeading.getMeasure());
    }

    /** WPIMath interpolates inside the table; this adapter preserves season extrapolation. */
    private static final class ExtrapolatingTable {
        private final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
        private final double[] distances;
        private final double[] values;

        ExtrapolatingTable(ShootingEntry[] entries, ToDoubleFunction<ShootingEntry> value) {
            distances = new double[entries.length];
            values = new double[entries.length];
            for (int i = 0; i < entries.length; i++) {
                distances[i] = entries[i].distance().in(Meters);
                values[i] = value.applyAsDouble(entries[i]);
                table.put(distances[i], values[i]);
            }
        }

        double get(double distance) {
            int last = distances.length - 1;
            if (distance >= distances[0] && distance <= distances[last]) {
                return table.get(distance);
            }
            int first = distance < distances[0] ? 0 : last - 1;
            // MathUtil.interpolate clamps its fraction, so cannot perform this operation.
            double fraction = (distance - distances[first]) / (distances[first + 1] - distances[first]);
            return values[first] + fraction * (values[first + 1] - values[first]);
        }
    }
}
