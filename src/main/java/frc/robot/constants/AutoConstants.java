package frc.robot.constants;

import org.wpilib.math.trajectory.TrapezoidProfile;
import static org.wpilib.units.Units.Seconds;
import org.wpilib.units.measure.Time;

public final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        // Whether pathplanner should actually reset odometry for pathplanner autos
        public static final boolean kIgnoreResetOdometry = false;

        // Auto Names
        public static final String kExampleAutoName = "Example Auto";
        public static final Time kShootTime = Seconds.of(3.5);
}
