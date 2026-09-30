package frc.robot.constants;

import org.wpilib.units.AngleUnit;
import org.wpilib.units.Measure;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;
import frc.robot.utils.ShootingEntry;

public final class ShooterConstants {
        public static final int kShooterMotorId = 5;
        public static final int kHoodMotorId = 4; // Was 31

        public static final double kHoodP = 5.0; // Only use this high P when converion factor is 1.
        public static final double kHoodI = 0.0;
        public static final double kHoodD = 0.0;

        public static final double kShooterP = 0.0001;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;
        public static final double kShooterFF = 0.0019;

        public static final AngularVelocity kShooterVelocityTolerance = RPM.of(50);

        // Teeth on encoder gear to teeth on shaft, teeth on shaft to teeth on hood part
        // NOTE: Need to use 14D so the result is a double, otherwise you end up with
        // zero.
        // 2.48 * 0.0642201834862385 = 0.1592660550458716
        // 16.5 motor rotations to one absolute encoder rotation (roughly)
        // NOTE: gear ration commented out for now is it isn't used
        // public static final double kHoodGearRatio = (62D / 25) * (14D / 218);
        public static final int kHoodSmartCurrentLimit = 20;
        public static final Angle kFeedAngle = Degrees.of(25.0);

        public static final AngularVelocity kPlaceholderWheelVelocity = RPM.of(2000);
        public static final LinearVelocity kMuzzleVelocity = MetersPerSecond.of(10);

        public static final LinearVelocity kMaxMuzzleVelocity = MetersPerSecond.of(10.0);

        public static final Distance kHubRobotTurretOffset = Inches.of(47);

        public static final ShootingEntry[] kShootingEntries = {
                        new ShootingEntry(Inches.of(30).plus(kHubRobotTurretOffset), RPM.of(3519), null,
                                        Inches.of(101.9),
                                        Seconds.of(0.812),
                                        Degrees.of(0)),
                        new ShootingEntry(Inches.of(59).plus(kHubRobotTurretOffset), RPM.of(3565), null,
                                        Inches.of(102.335),
                                        Seconds.of(0.822),
                                        Degrees.of(0)),
                        new ShootingEntry(Inches.of(89).plus(kHubRobotTurretOffset), RPM.of(3975), null,
                                        Inches.of(120.62),
                                        Seconds.of(1.0),
                                        Degrees.of(0)),
                        new ShootingEntry(Inches.of(122).plus(kHubRobotTurretOffset), RPM.of(4375), null,
                                        Inches.of(134.055333),
                                        Seconds.of(1.217),
                                        Degrees.of(0)),
                        new ShootingEntry(Inches.of(148).plus(kHubRobotTurretOffset), RPM.of(4600), null,
                                        Inches.of(147.348333),
                                        Seconds.of(1.322),
                                        kFeedAngle)
        };

        public static final Angle kHoodTolerence = Degrees.of(2.0);

        public static final LinearVelocity kMaxStationaryVelocity = MetersPerSecond.of(1e-1);

        public static final double kHoodMinAbsolutePosition = 0.0;
        public static final double kHoodMaxAbsolutePosition = 0.55;

        public static final double kHoodDegreeConversionFactor = kHoodMaxAbsolutePosition / 30;

        // TODO: Change to real numbers
        public static final AngularVelocity kNonAimShooterVelocity = RPM.of(2000);
        public static final Angle kNonAimHoodAngle = Degrees.of(15.0);
        public static final AngularVelocity kFeedingWheelVelocity = RPM.of(4000);
        public static final Angle kHoodFeedingPosition = Degrees.of(25.0);
        public static final Measure<AngleUnit> kTurretAngleRestrictiveShooterAngle = Degrees.of(10);

        public static final Angle kHoodStartingAngle = Degrees.of(3.0);
        public static final AngularVelocity kShooterStartVelocity = RPM.of(0.0);
        public static final Angle kDefaultHoodPosition = Degrees.of(3.0);

        public static final Time kRampTime = Seconds.of(0.4);

        // Absolute encoder wraps backwards, so it doesn't read -0.001, it reads 0.999.
        // This is the min rotational amount where we can reasonably assume that the
        // hood has just gone backwards a little too far, beyond the zero of the encoder
        public static final Angle kWrapBackMin = Rotations.of(0.9);
        public static final Time kAutoShootTime = Seconds.of(4.0);
        public static final AngularVelocity kDefaultShooterVelocity = RPM.of(4000);
}
