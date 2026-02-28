// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.lang.reflect.Field;
import java.util.HashMap;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.utils.ShootingEntry;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static final class DriveConstants {
                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds

                public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(2.4);
                public static final LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(10.0);

                public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(2 * Math.PI);
                public static final AngularAcceleration kMaxAngularAcceleration = RadiansPerSecondPerSecond
                                .of(4 * Math.PI);

                // Chassis configuration
                public static final double kTrackWidth = Units.inchesToMeters(26.5);
                // Distance between centers of right and left wheels on robot
                public static final double kWheelBase = Units.inchesToMeters(26.5);
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

        public static final class ModuleConstants {
                // The MAXSwerve module can be configured with one of three pinion gears: 12T,
                // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
                // more teeth will result in a robot that drives faster).

                public static final int kDrivingMotorPinionTeeth = 14;

                // Calculations required for driving motor conversion factors and feed forward
                public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = 0.0762;
                public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
                // teeth on the bevel pinion

                public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                                * kWheelCircumferenceMeters)
                                / kDrivingMotorReduction;
        }

        public static final class OIConstants {

                public static final int kDriverControllerPort = 0;
                public static final double kDriveDeadband = 0.05;
        }

        public static final class AutoConstants {

                public static final double kMaxSpeedMetersPerSecond = 3;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

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
        }

        public static final class NeoMotorConstants {

                public static final double kFreeSpeedRpm = 5676;
        }

        public static final class VisionConstants {

                public static final String kCameraName1 = "Photonvision";
                public static final String kCameraName2 = "Photonvision2";

                // Distance to fill pose3d z value assuming robot is on the ground
                public static Distance kEncoderZOffset = Inches.of(5.0);

                // Confidence of encoder readings for vision; should be tuned
                public static final double kEncoderConfidence = 0.15;

                public static final Transform3d kRobotToCamOne = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                                new Rotation3d(0, 0, 0));

                // These are not final numbers
                public static final Transform3d kRobotToCamTwo = new Transform3d(
                                new Translation3d(Inches.of(8.375), Inches.of(-2.16), Inches.of(-20.668)),
                                new Rotation3d(0, 0, 0));

                public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                                .loadField(AprilTagFields.kDefaultField);

                // Placeholder numbers
                public static final Distance kTurretCameraDistanceToCenter = Meters.of(0.13);
                public static final Distance kCameraTwoZ = Inches.of(18.0);

                public static final Translation3d kTurretCenterOfRotation = new Translation3d(Meters.of(0.0),
                                Meters.of(0.0),
                                Inches.of(18));

                public static final Angle kCameraTwoPitch = Degrees.of(15.0);
                public static final Angle kCameraTwoRoll = Degrees.of(0.0);
                public static final Angle kCameraTwoYaw = Degrees.of(-21.0);

                public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
                public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

                public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
                public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(1, 1, 1);

                public static final AngularVelocity kMaxTurretVisionSpeed = RPM.of(30);
        }

        public static final class NumericalConstants {
                public static final double kEpsilon = 1e-6;
                public static final Angle kFullRotation = Radians.of(2.0 * Math.PI);
                public static final Angle kNoRotation = Radians.of(0.0);
                public static final LinearAcceleration kGravity = MetersPerSecondPerSecond.of(9.807);
        }

        public static final class TurretConstants {
                public static final int kMotorId = 18; // Was 20
                public static final Angle kMinAngle = Rotations.of(0.1);
                public static final Angle kMaxAngle = Rotations.of(0.854);

                public static final int kPositionBufferLength = 4000;
                public static final Time kEncoderReadingDelay = Seconds.of(0.005);

                public static final Time kEncoderReadInterval = Seconds.of(0.01);

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

                public static final Translation2d kTurretOffset = new Translation2d(Meters.of(0.0), Meters.of(0.0));

                public static final Angle kTurretAngleTolerance = Degrees.of(2.0);

                // TODO: Change to real numbers
                public static Angle kNonAimTurretAngle = Degrees.of(25.0);
                public static int kTurretMotorAmpLimit = 10;

                public static final Angle kAngularDistanceToFrontOfRobot = Rotations.of(0.605);
        }

        public static final class ShooterConstants {
                public static final int kShooterMotorId = 5;
                public static final int kHoodMotorId = 4; // Was 31

                public static final double kHoodP = 5.0; // Only use this high P when converion factor is 1.
                public static final double kHoodI = 0.0;
                public static final double kHoodD = 0.0;

                public static final double kShooterP = 0.0001;
                public static final double kShooterI = 0.0;
                public static final double kShooterD = 0.0;
                public static final double kShooterFF = 0.0019;

                // Teeth on encoder gear to teeth on shaft, teeth on shaft to teeth on hood part
                // NOTE: Need to use 14D so the result is a double, otherwise you end up with
                // zero.
                // 2.48 * 0.0642201834862385 = 0.1592660550458716
                // 16.5 motor rotations to one absolute encoder rotation (roughly)
                // NOTE: gear ration commented out for now is it isn't used
                // public static final double kHoodGearRatio = (62D / 25) * (14D / 218);
                public static final int kHoodSmartCurrentLimit = 20;
                public static final Angle kFeedAngle = Degrees.of(90.0);

                public static final AngularVelocity kPlaceholderWheelVelocity = RPM.of(2000);
                public static final LinearVelocity kMuzzleVelocity = MetersPerSecond.of(10);

                public static final LinearVelocity kMaxMuzzleVelocity = MetersPerSecond.of(10.0);

                public static final ShootingEntry[] kShootingEntries = {
                                new ShootingEntry(Meters.of(0.0), kPlaceholderWheelVelocity, kMaxMuzzleVelocity, null,
                                                Seconds.of(1.0),
                                                kFeedAngle),
                                new ShootingEntry(Meters.of(1.0), kPlaceholderWheelVelocity, kMaxMuzzleVelocity, null,
                                                Seconds.of(1.0),
                                                kFeedAngle),
                                new ShootingEntry(Meters.of(2.0), kPlaceholderWheelVelocity, kMaxMuzzleVelocity, null,
                                                Seconds.of(1.0),
                                                kFeedAngle),
                                new ShootingEntry(Meters.of(3.0), kPlaceholderWheelVelocity, kMaxMuzzleVelocity, null,
                                                Seconds.of(1.0),
                                                kFeedAngle),
                                new ShootingEntry(Meters.of(4.0), kPlaceholderWheelVelocity, kMaxMuzzleVelocity, null,
                                                Seconds.of(1.0),
                                                kFeedAngle),
                                new ShootingEntry(Meters.of(5.0), kPlaceholderWheelVelocity, kMaxMuzzleVelocity, null,
                                                Seconds.of(1.0),
                                                kFeedAngle),
                };

                public static final Angle kHoodTolerence = Degrees.of(2.0);

                public static final LinearVelocity kMaxStationaryVelocity = MetersPerSecond.of(1e-1);

                public static final double kHoodMinAbsolutePosition = 0.0;
                public static final double kHoodMaxAbsolutePosition = 0.55;

                public static final double kHoodDegreeConversionFactor = kHoodMaxAbsolutePosition / 30;

                // TODO: Change to real numbers
                public static AngularVelocity kNonAimShooterVelocity = RPM.of(500);
                public static Angle kNonAimHoodAngle = Degrees.of(15.0);
                public static AngularVelocity kFeedingWheelVelocity = RPM.of(60);
                public static Angle kHoodFeedingPosition = Degrees.of(25.0);
                public static Measure<AngleUnit> kTurretAngleRestrictiveShooterAngle = Degrees.of(10);
        }

        public static final class StagingConstants {
                public static int kFeedIntoHoodMotor = 16;
                public static double kFeedIntoHoodSpeed = 0.10;

                public static final int kAgitationMotorId = 9;
                public static final double kAgitationSpeed = -0.15;

                public static final int kRollerMotorId = 12;
                public static final double kRollerSpeed = 0.15;
        }

        public static final class Fixtures {
                public static final Translation2d kBlueAllianceHub = new Translation2d(Inches.of(182.11),
                                Inches.of(154.84));
                public static final Translation2d kRedAllianceHub = new Translation2d(Inches.of(651.22 - 182.11),
                                Inches.of(158.84));

                // From a top down perspective of the field with the red alliance on the left
                // side
                public static final Translation2d kTopFeedPose = new Translation2d();
                public static final Translation2d kBottomFeedPose = new Translation2d();

                public static final Distance kFieldYMidpoint = Inches.of(158.84);

                public static final Distance kBlueSideNeutralBorder = Inches.of(182.11);
                public static final Distance kRedSideNeutralBorder = Inches.of(651.22 - 182.11);

                public static enum FieldLocations {
                        AllianceSide,
                        NeutralSide,
                        OpponentSide
                }

                public static final HashMap<FieldLocations, String> kFieldLocationStringMap = new HashMap<>();

                static {
                        kFieldLocationStringMap.put(FieldLocations.AllianceSide, "Alliance Side");
                        kFieldLocationStringMap.put(FieldLocations.NeutralSide, "Neutral Side");
                        kFieldLocationStringMap.put(FieldLocations.OpponentSide, "Opponent Side");
                }

                // Placeholders
                public static final Angle kBlueLeftSideFeedHeading = Degrees.of(40);
                public static final Angle kBlueRightSideFeedHeading = Degrees.of(160);

                // Placeholders
                public static final Angle kRedLeftSideFeedHeading = Degrees.of(-40);
                public static final Angle kRedRightSideFeedHeading = Degrees.of(-160);

                public static final Pose2d kRedHubAprilTag = AprilTagFieldLayout
                                .loadField(AprilTagFields.k2026RebuiltAndymark)
                                .getTagPose(3).get().toPose2d();
        }

        public static final class IntakeConstants {
                public static final double kP1 = 0.05;
                public static final double kI1 = 0;
                public static final double kD1 = 0;

                public static final double kP2 = 0.05;
                public static final double kI2 = 0;
                public static final double kD2 = 0;

                public static final double kPIn = 0.0003;
                public static final double kIIn = 0;
                public static final double kDIn = 0.0005;
                public static final double kFFIn = 0.00192;

                public static final int kDeployMotor1Id = 13;
                public static final int kDeployMotor2Id = 8;
                public static final int kIntakeMotorId = 7;

                // 10 teeth on pinion, 20 teeth on rack
                public static final Angle kDeployRotations = Rotations.of(9.5);
                public static final Angle kRetractRotations = Rotations.of(0.0);

                public static final Angle kMaxExtension = Rotations.of(9.5);
                public static final Angle kMinExtension = Rotations.of(0.0);

                public static final int kDeployMotorCurrentLimit = 40;
                public static final int kIntakeMotorCurrentLimit = 80;

                public static final AngularVelocity kDefaultIntakeSpeed = RPM.of(-2000);
        }
}
