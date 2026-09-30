package frc.robot.constants;

import org.wpilib.math.geometry.Translation2d;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

public final class TurretConstants {
        public static final int kMotorId = 18; // Was 20
        public static final Angle kMinAngle = Rotations.of(0.1);
        public static final Angle kMaxAngle = Rotations.of(0.854);

        public static final int kPositionBufferLength = 300;
        public static final Time kEncoderReadingDelay = Seconds.of(0.005);

        public static final Time kEncoderReadInterval = Seconds.of(0.05);

        public static final double kP = 1.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final int kSmartCurrentLimit = 40;

        public static final Angle kHubMinAngle1 = Degrees.of(311);
        public static final Angle kHubMaxAngle1 = Degrees.of(351);

        public static final Angle kHubMinAngle2 = Degrees.of(180);
        public static final Angle kHubMaxAngle2 = Degrees.of(224);

        public static final Angle kFeedMinAngle = Degrees.of(180);
        public static final Angle kFeedMaxAngle = Degrees.of(224);

        public static final Angle kTurretCameraIdleViewMinAngle = Rotations.of(0.375);
        public static final Angle kTurretCameraIdleViewMaxAngle = Rotations.of(0.582);
        public static final Angle kTurretCameraMidPoint = kTurretCameraIdleViewMinAngle
                        .plus(kTurretCameraIdleViewMaxAngle).div(2.0);

        public static final Angle[] kRestrictedAngles = new Angle[] {
                        kFeedMinAngle, kFeedMaxAngle
        };

        public static final Angle[] kUnrestrictedAngles = new Angle[] {
                        kHubMinAngle1, kHubMaxAngle1, kHubMinAngle2, kHubMaxAngle2
        };

        public static final Angle kOvershootAmount = Degrees.of(10.0);

        public static final Translation2d kTurretOffset = new Translation2d(Inches.of(-6.25),
                        Inches.of(6.151));

        public static final Angle kTurretAngularOffset = Radians
                        .of(Math.atan2(kTurretOffset.getY(), kTurretOffset.getX()));

        public static final Distance kTurretCenterDistanceFromRobotCenter = Meters
                        .of(Math.sqrt(Math.pow(kTurretOffset.getX(), 2.0)
                                        + Math.pow(kTurretOffset.getY(), 2.0)));

        public static final Angle kTurretAngleTolerance = Degrees.of(2.0);

        public static Angle kNonAimTurretAngle = Degrees.of(0.0);
        public static int kTurretMotorAmpLimit = 10;
        public static final Angle kTurretTorwardsFront = Degrees.of(180);

        public static final Angle kAngularDistanceToFrontOfRobot = Rotations.of(0.629);
}
