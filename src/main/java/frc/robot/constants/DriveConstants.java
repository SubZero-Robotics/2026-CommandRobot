package frc.robot.constants;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.util.Units;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Seconds;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;
import frc.robot.Robot;

public final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds

        public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(4.0);
        public static final LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(10.0);

        public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(2 * Math.PI);
        public static final AngularAcceleration kMaxAngularAcceleration = RadiansPerSecondPerSecond
                        .of(4 * Math.PI);

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(23.149606);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(23.149606);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs Drive Motors
        public static final int kFrontLeftDrivingCanId = 10;
        public static final int kRearLeftDrivingCanId = 14;
        public static final int kFrontRightDrivingCanId = 1;
        public static final int kRearRightDrivingCanId = 2;

        // SPARK MAX CAN IDs Turning Motors
        public static final int kFrontLeftTurningCanId = 11;
        public static final int kRearLeftTurningCanId = 15;
        public static final int kFrontRightTurningCanId = 62;
        public static final int kRearRightTurningCanId = 3;

        // Auxiliary Device Can IDs
        public static final int kPidgeyCanId = 13;

        public static final boolean kGyroReversed = false;

        public static final Time kPeriodicInterval = Seconds.of(0.02);

        public static final double kAutoRotationP = Robot.isReal() ? 3.6 : 3.0;
        public static final double kAutoRotationI = 0.0;
        public static final double kAutoRotationD = 0.0;

        public static enum RangeType {
                Within,
                CloseMin,
                CloseMax
        }

        public static final Angle kTurnToAngleTolerance = Degrees.of(2);
}
