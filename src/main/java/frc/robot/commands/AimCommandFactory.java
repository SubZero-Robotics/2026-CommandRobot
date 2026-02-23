package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.Fixtures;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.ShootingEntry;
import frc.robot.utils.TargetSolution;
import frc.robot.utils.UtilityFunctions;

public class AimCommandFactory {

    private DriveSubsystem m_drive;
    private TurretSubsystem m_turret;
    private ShooterSubsystem m_shooter;

    public AimCommandFactory(DriveSubsystem drive, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_drive = drive;
        m_turret = turret;
        m_shooter = shooter;
    }

    public Command Aim(Supplier<Boolean> isFeedingLeftSide) {
        return new RunCommand(() -> {
            Fixtures.FieldLocations location = m_drive.getRobotLocation();
            Pose2d robotPose = m_drive.getPose();
            Angle robotAngle = Radians.of(robotPose.getRotation().getRadians());

            switch (location) {
                case AllianceSide: {
                    Translation2d fixtureTranslation = DriverStation.getAlliance().get() == Alliance.Blue
                            ? Fixtures.kBlueAllianceHub
                            : Fixtures.kRedAllianceHub;

                    double dy = fixtureTranslation.getX() - robotPose.getX();
                    double dx = fixtureTranslation.getY() - robotPose.getY();

                    Angle absHeading = Radians.of(Math.atan2(dy, dx));
                    Angle relHeading = absHeading.minus(robotAngle);

                    m_turret.moveToAngle(relHeading);
                    break;
                }
                case NeutralLeftSide: {
                    // Heading changes 180 degrees depending on which alliance you are on
                    Angle offset = DriverStation.getAlliance().get() == Alliance.Blue ? Degrees.of(0) : Degrees.of(180);
                    Angle absHeading = isFeedingLeftSide.get() ? offset.minus(Degrees.of(50))
                            : offset.plus(Degrees.of(50));
                    Angle relHeading = absHeading.minus(robotAngle);

                    robotPose = m_drive.getPose();
                    break;
                }
                case NeutralRightSide: {
                    break;
                }
                case OpponentSide: {
                    break;
                }
                default:
                    break;
            }
        }, m_drive, m_turret).until(() -> {
            return true;
        });
    }

    public TargetSolution GetHubAimSolution() {
        Translation2d hubPosition = DriverStation.getAlliance().get() == Alliance.Blue ? Fixtures.kBlueAllianceHub
                : Fixtures.kRedAllianceHub;

        Translation2d turretTranslation = m_drive.getPose().getTranslation().plus(TurretConstants.kTurretOffset);

        Translation2d translationToHub = hubPosition.minus(turretTranslation);

        Distance turretToHubDistance = Meters
                .of(Math.hypot(translationToHub.getMeasureY().in(Meters), translationToHub.getMeasureX().in(Meters)));
        Angle turretToHubAngle = Radians
                .of(Math.atan2(translationToHub.getMeasureY().in(Meters), translationToHub.getMeasureX().in(Meters)));

        ChassisSpeeds robotSpeeds = m_drive.getChassisSpeeds();

        return getInterpolatedShootingParameters(turretToHubDistance,
                MetersPerSecond.of(robotSpeeds.vxMetersPerSecond), MetersPerSecond.of(robotSpeeds.vyMetersPerSecond),
                turretToHubAngle);
    }

    public Command MoveTurretToHeadingCommand(Angle heading) {
        return new RunCommand(() -> {
            MoveTurretToHeading(heading);
        }, m_turret);
    }
    public Command MoveHoodToAbsoluteCommand(Angle angle) {
        return new InstantCommand(() -> {
            System.out.println("Move Hood to position " + angle);
            m_shooter.MoveHoodToPosition(angle);
        });
    }

    public void MoveTurretToHeading(Angle heading) {
        Angle robotHeading = UtilityFunctions.WrapAngle(m_drive.getHeading());

        Angle robotRelativeTurretAngle = UtilityFunctions.WrapAngle(heading.minus(robotHeading));

        if (withinRange(TurretConstants.kFeedMinAngle1, TurretConstants.kFeedMaxAngle1, robotRelativeTurretAngle)
                || withinRange(TurretConstants.kFeedMinAngle2, TurretConstants.kFeedMaxAngle2,
                        robotRelativeTurretAngle)) {
            m_turret.moveToAngle(robotRelativeTurretAngle);
        } else {
            // Gets which ray the robot is closest to
            Angle closest = getClosestAngle(robotRelativeTurretAngle, TurretConstants.kFeedMinAngle1,
                    TurretConstants.kFeedMaxAngle1,
                    TurretConstants.kFeedMinAngle2, TurretConstants.kFeedMaxAngle2);

            // The overshoot is negative if the robot has to move in a negative direction;
            // same for positive
            Angle overshoot = robotRelativeTurretAngle.minus(closest).in(Degrees) < 0.0
                    ? TurretConstants.kOvershootAmount
                    : TurretConstants.kOvershootAmount.times(-1.0);

            closest = closest.plus(overshoot);

            Angle driveTarget = heading.minus(closest);

            m_drive.moveToAngle(driveTarget);
            m_turret.moveToAngle(closest);
        }
    }

    public Command PointTurretToFixture(Pose2d fixture) {
        return new RunCommand(() -> {
            Pose2d robotPose = m_drive.getPose();

            double dx = fixture.getX() - robotPose.getX();
            double dy = fixture.getY() - robotPose.getY();

            Angle angle = Radians.of(Math.atan2(dy, dx)).minus(Radians.of(robotPose.getRotation().getRadians()));

            MoveTurretToHeading(angle);
        }, m_turret);
    }

    private static boolean withinRange(Angle min, Angle max, Angle a) {
        a = UtilityFunctions.WrapAngle(a);
        min = UtilityFunctions.WrapAngle(min);
        max = UtilityFunctions.WrapAngle(max);

        return a.gt(min) && a.lt(max);
    }

    private static Angle getClosestAngle(Angle a, Angle... others) {
        a = UtilityFunctions.WrapAngle(a);

        // for (Angle as : others) {
        // System.out.print(as.in(Degrees) + " ");
        // }
        // System.out.print(a.in(Degrees) + " is robot heading");
        // System.out.println();

        Angle closest = UtilityFunctions.WrapAngle(others[0]);
        double closestDistance = UtilityFunctions.angleDiff(a, closest).abs(Degrees);

        for (int i = 1; i < others.length; i++) {
            Angle candidate = UtilityFunctions.WrapAngle(others[i]);
            double dif = UtilityFunctions.angleDiff(a, candidate).abs(Degrees);

            if (dif < closestDistance) {
                closest = candidate;
                closestDistance = dif;

                // System.out.println(closest + " " + others.length);
            }
        }

        return closest;
    }

    private static int getFirstEntryIndex(Distance distance) {
        int low = 0;
        int high = ShooterConstants.kShootingEntries.length;
        int mid = 0;

        while (low < high) {
            mid = (low + high) / 2;
            ShootingEntry midEntry = ShooterConstants.kShootingEntries[mid];

            if (distance.gt(midEntry.distance())) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }

        ShootingEntry closestEntry = ShooterConstants.kShootingEntries[mid];

        int previousEntryIndex;

        if (distance.lt(closestEntry.distance())) {
            if (mid == 0) {
                previousEntryIndex = mid;
            } else {
                previousEntryIndex = mid - 1;
            }
        } else {
            if (mid == ShooterConstants.kShootingEntries.length - 1) {
                previousEntryIndex = mid - 1;
            } else {
                previousEntryIndex = mid;
            }
        }

        return previousEntryIndex;
    }

    private static TargetSolution getInterpolatedShootingParameters(Distance distance, LinearVelocity vx,
            LinearVelocity vy, Angle turretAngle) {

        LinearVelocity robotVelocity = MetersPerSecond.of(Math.hypot(vx.in(MetersPerSecond), vy.in(MetersPerSecond)));

        int firstEntryIndex = getFirstEntryIndex(distance);

        ShootingEntry firstEntry = ShooterConstants.kShootingEntries[firstEntryIndex];
        ShootingEntry secondEntry = ShooterConstants.kShootingEntries[firstEntryIndex + 1];

        Angle phi = Radians.of(0.0);

        if (robotVelocity.gt(ShooterConstants.kMaxStationaryVelocity)) {
            Time timeOfFlight = Seconds.of(UtilityFunctions.interpolate(firstEntry.distance().in(Meters),
                    secondEntry.distance().in(Meters), firstEntry.timeOfFlight().in(Seconds),
                    secondEntry.timeOfFlight().in(Seconds), distance.in(Meters)));

            LinearVelocity radialVelocityTorwardsHub = MetersPerSecond
                    .of(vy.in(MetersPerSecond) * Math.sin(turretAngle.in(Radians))
                            + vx.in(MetersPerSecond) * Math.cos(turretAngle.in(Radians)));

            LinearVelocity tangentialVelocityFromHub = MetersPerSecond
                    .of(vx.in(MetersPerSecond) * Math.sin(turretAngle.in(Radians))
                            + vy.in(MetersPerSecond) * Math.cos(turretAngle.in(Radians)));

            Distance sideDistance = tangentialVelocityFromHub.times(timeOfFlight);
            distance = distance.minus(radialVelocityTorwardsHub.times(timeOfFlight));

            phi = Radians.of(Math.atan(sideDistance.in(Meters) / distance.in(Meters)));

            int transformedFirstEntryIndex = getFirstEntryIndex(distance);

            firstEntry = ShooterConstants.kShootingEntries[transformedFirstEntryIndex];
            secondEntry = ShooterConstants.kShootingEntries[transformedFirstEntryIndex + 1];
        }

        AngularVelocity wheelSpeed = RadiansPerSecond.of(UtilityFunctions.interpolate(firstEntry.distance().in(Meters),
                secondEntry.distance().in(Meters), firstEntry.wheelVelocity().in(RadiansPerSecond),
                secondEntry.wheelVelocity().in(RadiansPerSecond), distance.in(Meters)));

        Angle hoodAngle = Radians.of(UtilityFunctions.interpolate(firstEntry.distance().in(Meters),
                secondEntry.distance().in(Meters), firstEntry.shooterAngle().in(Radians),
                secondEntry.shooterAngle().in(Radians), distance.in(Meters)));

        return new TargetSolution(hoodAngle, wheelSpeed, phi, distance, turretAngle);
    }
}