package frc.robot.commands;

import java.lang.annotation.Target;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

    private boolean m_isAiming = false;

    private AngularVelocity m_wheelVelocity;

    public AimCommandFactory(DriveSubsystem drive, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_drive = drive;
        m_turret = turret;
        m_shooter = shooter;
    }

    public Command AimCommand(Supplier<Boolean> isFeedingLeftSide) {
        return new RunCommand(() -> {
            Aim(isFeedingLeftSide);
            m_isAiming = true;
        }, m_drive, m_turret).finallyDo(() -> {
            m_isAiming = false;
            m_wheelVelocity = ShooterConstants.kNonAimShooterVelocity;
        });
    }

    private void Aim(Supplier<Boolean> isFeedingLeftSide) {
        Fixtures.FieldLocations location = m_drive.getRobotLocation();

        switch (location) {
            case AllianceSide: {
                TargetSolution solution = GetHubAimSolution();

                MoveTurretToHeading(solution.hubAngle());
                m_shooter.MoveHoodToPosition(solution.hoodAngle());

                m_wheelVelocity = solution.wheelSpeed();
                break;
            }
            case NeutralSide: {
                // Heading changes 180 degrees depending on which alliance you are on
                Angle offset = DriverStation.getAlliance().get() == Alliance.Blue ? Degrees.of(0) : Degrees.of(180);
                Angle absHeading = isFeedingLeftSide.get() ? offset.minus(Degrees.of(50))
                        : offset.plus(Degrees.of(50));

                m_shooter.MoveHoodToPosition(ShooterConstants.kHoodFeedingPosition);
                m_wheelVelocity = ShooterConstants.kFeedingWheelVelocity;
                MoveTurretToHeading(absHeading);
                break;
            }
            case OpponentSide: {
                System.out.println("Why are you here???");
            }
            default:
                break;
        }
    }

    public Command AimHoodToPositionCommand(Angle angle) {
        return new RunCommand(() -> {
            m_shooter.MoveHoodToPosition(angle);
        }).until(m_shooter::AtTarget);
    }

    public Command AimTurretRelativeToRobot(Angle angle) {
        return new RunCommand(() -> {
            m_turret.moveToAngle(angle);
        }, m_turret).until(m_turret::atTarget);
    }

    public Command ShootCommand() {
        return new ConditionalCommand(new RunCommand(this::Shoot, m_shooter),
                AimHoodToPositionCommand(ShooterConstants.kNonAimHoodAngle)
                        .alongWith(AimTurretRelativeToRobot(TurretConstants.kNonAimTurretAngle))
                        .andThen(new RunCommand(this::Shoot, m_shooter)),
                () -> m_isAiming);
    }

    private void Shoot() {
        m_shooter.Spin(m_wheelVelocity);
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

        Angle[] currentRange = getCurrentTurretRange();

        if (withinAngles(currentRange, robotRelativeTurretAngle)) {
            m_turret.moveToAngle(robotRelativeTurretAngle);
        } else {
            // Gets which ray the robot is closest to
            Angle closest = getClosestAngle(robotRelativeTurretAngle, currentRange);

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

    public Command Aim(Angle turretAngle, Angle hoodAngle) {
        return new InstantCommand(() -> {
            m_turret.moveToAngle(turretAngle);
            m_shooter.MoveHoodToPosition(hoodAngle);
        });
    }

    public Command Shoot(AngularVelocity shooterWheelVelocity) {
        return new InstantCommand(() -> {
            m_shooter.Spin(shooterWheelVelocity);
        });
    }

    // TODO: Make this better

    // Our valid shooting ranges are going to change based on the shooter hood
    // angle. If the hood angle is too low, then shooting the ball would lead to it
    // hitting the side of the robot or other balls currently being held in the
    // robot
    private Angle[] getCurrentTurretRange() {
        if (m_shooter.GetHoodAngle().lt(ShooterConstants.kTurretAngleRestrictiveShooterAngle)) {
            return TurretConstants.kRestrictedAngles;
        }

        return TurretConstants.kUnrestrictedAngles;
    }

    // Must be organized where at every even index it contains the minimum and every
    // odd index contains the max angle
    private boolean withinAngles(Angle[] angles, Angle candidate) {
        if (angles.length % 2 != 0) {
            return false;
        }

        for (int i = 0; i <= angles.length; i += 2) {
            Angle min = angles[i];
            Angle max = angles[i + 1];
            if (withinRange(min, max, candidate))
                return true;
        }

        return false;
    }
}