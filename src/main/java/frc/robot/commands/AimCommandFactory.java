package frc.robot.commands;

import java.lang.annotation.Target;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Fixtures;
import frc.robot.Constants.NumericalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingEntry;
import frc.robot.utils.TargetSolution;
import frc.robot.utils.UtilityFunctions;

public class AimCommandFactory {

    private DriveSubsystem m_drive;
    private TurretSubsystem m_turret;
    private ShooterSubsystem m_shooter;

    private boolean m_isAiming = false;

    private AngularVelocity m_wheelVelocity = RPM.of(5000);

    private Translation2d m_lockedTag;

    private StagingSubsystem m_stager = new StagingSubsystem();

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

                m_drive.moveByAngle(solution.phi());

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

    // public Command ShootCommand() {
    // return new ConditionalCommand(new RunCommand(this::Shoot, m_shooter),
    // AimHoodToPositionCommand(ShooterConstants.kNonAimHoodAngle)
    // .alongWith(AimTurretRelativeToRobot(TurretConstants.kNonAimTurretAngle))
    // .andThen(new RunCommand(this::Shoot, m_shooter)),
    // () -> m_isAiming).alongWith(new RunCommand(() -> {
    // m_stager.Agitate();
    // m_stager.Feed();
    // m_stager.Roll();
    // }, m_stager)).finallyDo(m_shooter::Stop);
    // }

    public Command RunAllStager() {
        return new RunCommand(() -> {
            m_stager.Agitate();
            m_stager.Feed();
            m_stager.Roll();
        }, m_stager).finallyDo(() -> {
            m_stager.StopAgitate();
            m_stager.StopFeed();
            m_stager.StopRoll();
        });
    }

    private void Shoot() {
        m_shooter.Spin(m_wheelVelocity);
    }

    public Command ShootCommand() {
        return new InstantCommand(() -> {
            Shoot();
        });
    }

    public void ShootAtVelocity(AngularVelocity velocity) {
        m_shooter.Spin(velocity);
    }

    public Command stopShootCommand() {
        return new InstantCommand(() -> {
            StopShoot();
        });
    }

    public void StopShoot() {
        m_shooter.Stop();
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

    public Command moveHoodToAngleCommand(Angle angle) {
        return new InstantCommand(() -> {
            MoveHoodToAngle(angle);
        });
    }

    public void MoveHoodToAngle(Angle angle) {
        System.out.println("Move Hood to angle " + angle.in(Degrees) + " degrees.");
        m_shooter.MoveHoodToPosition(angle);
    }

    public Command PointAtHub(boolean isRed) {
        return new RunCommand(() -> {
            Translation2d hubPosition = isRed ? Fixtures.kRedAllianceHub : Fixtures.kBlueAllianceHub;
            Translation2d robotPose = m_drive.getPose().getTranslation();

            double dx = hubPosition.getMeasureX().minus(robotPose.getMeasureX()).in(Meters);
            double dy = hubPosition.getMeasureY().minus(robotPose.getMeasureY()).in(Meters);

            Angle angle = Radians.of(Math.atan2(dy, dx));

            MoveTurretToHeading(angle);
            System.out.println(angle);
        }).finallyDo(m_drive::disableFaceHeading);
    }

    public void MoveTurretToHeading(Angle heading) {
        Angle robotHeading = UtilityFunctions.WrapAngle(m_drive.getHeading());

        Angle robotRelativeTurretAngle = UtilityFunctions.WrapAngle(heading.minus(robotHeading));

        // Angle[] currentRange = getCurrentTurretRange();
        Angle[] currentRange = TurretConstants.kUnrestrictedAngles;

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

            // System.out.println();
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

    // Aims the camera at april tags within range
    public Command IdleCameraAim() {

        // TODO: Finish
        return new ConditionalCommand(new RunCommand(() -> {
            Angle absoluteMinAngle = m_drive.getHeading().plus(TurretConstants.kTurretCameraIdleViewMinAngle);
            Angle absoluteMaxAngle = m_drive.getHeading().plus(TurretConstants.kTurretCameraIdleViewMaxAngle);
            Pose2d robotPose = m_drive.getPose();
            Angle robotRotation = robotPose.getRotation().getMeasure();
            Translation2d robotTranslation = robotPose.getTranslation();

            Angle toTagAngle = angleFromTranslation(robotTranslation, m_lockedTag);

            if (!withinRange(robotRotation.plus(absoluteMinAngle), robotRotation.plus(absoluteMaxAngle), toTagAngle)) {
                ArrayList<Translation2d> aprilTagsInView = aprilTagsWithinRange(absoluteMinAngle, absoluteMaxAngle,
                        robotTranslation);

                if (aprilTagsInView.isEmpty())
                    return;

                var tags = aprilTagsWithinRange(absoluteMinAngle, absoluteMaxAngle, robotTranslation);
                Translation2d closestTag = getClosestAngleApriltag(TurretConstants.kTurretCameraMidPoint,
                        robotTranslation,
                        (Translation2d[]) tags.toArray());

                Angle angleToTag = angleFromTranslation(robotTranslation, closestTag);
                Angle turretRelativeAngleToTag = UtilityFunctions.WrapAngle(angleToTag.minus(robotRotation));

                m_lockedTag = closestTag;
                m_turret.moveToAngle(turretRelativeAngleToTag);
            }
        }, m_turret), null, () -> m_turret.getCurrentCommand() == null);
    }

    private ArrayList<Translation2d> aprilTagsWithinRange(Angle min, Angle max, Translation2d referenceTranslation) {
        ArrayList<Translation2d> anglesInRange = new ArrayList<>();

        for (int i = 1; i <= 32; i++) {
            Translation2d tag = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark).getTagPose(i).get()
                    .toPose2d().getTranslation();

            Angle angleToTag = angleFromTranslation(referenceTranslation,
                    tag);

            if (angleToTag.gt(min) && angleToTag.lt(max)) {
                anglesInRange.add(tag);
            }
        }

        return anglesInRange;
    }

    private static Angle angleFromTranslation(Translation2d reference, Translation2d target) {
        double dx = target.minus(reference).getX();
        double dy = target.minus(reference).getY();

        return Radians.of(Math.atan2(dy, dx));
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

        if (others.length == 0) {
            return null;
        }

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

    private static Translation2d getClosestAngleApriltag(Angle referenceAngle, Translation2d robot,
            Translation2d... tags) {
        if (tags.length == 0)
            return null;

        referenceAngle = UtilityFunctions.WrapAngle(referenceAngle);

        double closestDistance = 2 * Math.PI;
        Translation2d closestPosition = new Translation2d();

        for (Translation2d tag : tags) {
            Angle candidate = UtilityFunctions.WrapAngle(angleFromTranslation(robot, tag));
            double dif = UtilityFunctions.angleDiff(referenceAngle, candidate).abs(Degrees);

            if (dif < closestDistance) {
                closestDistance = dif;
                closestPosition = tag;
            }
        }

        return closestPosition;
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

        for (int i = 0; i < angles.length - 1; i += 2) {
            Angle min = angles[i];
            Angle max = angles[i + 1];
            if (withinRange(min, max, candidate))
                return true;
        }

        return false;
    }
}