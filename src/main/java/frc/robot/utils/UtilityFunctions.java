package frc.robot.utils;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/** Unit-aware adapters for WPIMath's angular operations. */
public final class UtilityFunctions {
    private UtilityFunctions() {}

    /** Normalize to [0, 2pi), including zero for an exact full turn. */
    public static Angle WrapAngle(Angle angle) {
        double wrapped = MathUtil.inputModulus(angle.in(Radians), 0.0, 2.0 * Math.PI);
        return Radians.of(wrapped == 2.0 * Math.PI ? 0.0 : wrapped);
    }

    /** Signed difference in (-pi, pi]; the half-turn tie is positive. */
    public static Angle angleDiff(Angle a1, Angle a2) {
        return Radians.of(MathUtil.angleModulus(a1.minus(a2).in(Radians)));
    }

    /** Compose orientations, rather than accumulating mechanical travel. */
    public static Angle addRotation(Angle a, Angle b) {
        return new Rotation2d(a).plus(new Rotation2d(b)).getMeasure();
    }

    public static Angle subtractRotation(Angle a, Angle b) {
        return new Rotation2d(a).minus(new Rotation2d(b)).getMeasure();
    }

    public static double angularDistance(Angle a, Angle b) {
        return angleDiff(a, b).abs(Radians);
    }

    /** Strict membership in a bounded, non-wrapping mechanism window. */
    public static boolean withinWindow(Angle min, Angle max, Angle angle) {
        Angle candidate = WrapAngle(angle);
        return candidate.gt(WrapAngle(min)) && candidate.lt(WrapAngle(max));
    }

    /** Strict membership in a counterclockwise arc, which may cross zero. */
    public static boolean withinArc(Angle min, Angle max, Angle angle) {
        double span = WrapAngle(max.minus(min)).in(Radians);
        double offset = WrapAngle(angle.minus(min)).in(Radians);
        return offset > 0.0 && offset < span;
    }

    /** Nearest orientation; callers choose which candidate wins an exact tie. */
    public static Angle closestAngle(Angle reference, boolean lastWinsTie, Angle... candidates) {
        if (candidates.length == 0) {
            return null;
        }
        Angle closest = WrapAngle(candidates[0]);
        double distance = angularDistance(reference, closest);
        for (int i = 1; i < candidates.length; i++) {
            double candidateDistance = angularDistance(reference, candidates[i]);
            if (candidateDistance < distance || (lastWinsTie && candidateDistance == distance)) {
                closest = WrapAngle(candidates[i]);
                distance = candidateDistance;
            }
        }
        return closest;
    }
}
