package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
                    Pose2d fixturePose = DriverStation.getAlliance().get() == Alliance.Blue ? Fixtures.kBlueAllianceHub
                            : Fixtures.kRedAllianceHub;

                    double dy = fixturePose.getX() - robotPose.getX();
                    double dx = fixturePose.getY() - robotPose.getY();

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

    public void AimToHub() {
        Pose2d hubPosition = DriverStation.getAlliance().get() == Alliance.Blue ? Fixtures.kBlueAllianceHub
                : Fixtures.kRedAllianceHub;

        Pose2d turretPose = m_drive.getPose().plus(TurretConstants.kTurretOffset).rotateBy(new Rotation2d());
        ChassisSpeeds robotSpeeds = m_drive.getChassisSpeeds();

        Pose2d distanceToHub = hubPosition.relativeTo(turretPose);

        LinearVelocity xTurretVelocity = MetersPerSecond.of(ShooterConstants.kMuzzleVelocity.in(MetersPerSecond)
                * Math.cos(turretPose.getRotation().getRadians()))
                .plus(MetersPerSecond.of(robotSpeeds.vxMetersPerSecond));

        LinearVelocity yTurretVelocity = MetersPerSecond.of(ShooterConstants.kMuzzleVelocity.in(MetersPerSecond)
                * Math.sin(turretPose.getRotation().getRadians()))
                .plus(MetersPerSecond.of(robotSpeeds.vyMetersPerSecond));

        Distance xOffTarget = Meters.of((xTurretVelocity.in(MetersPerSecond) / distanceToHub.getMeasureX().in(Meters))
                * xTurretVelocity.in(MetersPerSecond));
        Distance yOffTarget = Meters.of((yTurretVelocity.in(MetersPerSecond) / distanceToHub.getMeasureY().in(Meters))
                * yTurretVelocity.in(MetersPerSecond));

        Pose2d newPoseToAimTo = hubPosition.plus(new Transform2d(xOffTarget, yOffTarget, new Rotation2d()));
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

    private static Angle angleDiff(Angle a1, Angle a2) {
        Angle diff = a1.minus(a2);
        return UtilityFunctions.WrapTo180(diff);
    }

    private static Angle getClosestAngle(Angle a, Angle... others) {
        a = UtilityFunctions.WrapAngle(a);

        // for (Angle as : others) {
        // System.out.print(as.in(Degrees) + " ");
        // }
        // System.out.print(a.in(Degrees) + " is robot heading");
        // System.out.println();

        Angle closest = UtilityFunctions.WrapAngle(others[0]);
        double closestDistance = angleDiff(a, closest).abs(Degrees);

        for (int i = 1; i < others.length; i++) {
            Angle candidate = UtilityFunctions.WrapAngle(others[i]);
            double dif = angleDiff(a, candidate).abs(Degrees);

            if (dif < closestDistance) {
                closest = candidate;
                closestDistance = dif;

                // System.out.println(closest + " " + others.length);
            }
        }

        return closest;
    }
}