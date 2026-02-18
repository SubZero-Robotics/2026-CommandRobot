package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix.Util;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Fixtures;
import frc.robot.Constants.NumericalConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.UtilityFunctions;

public class AimCommandFactory {

    private DriveSubsystem m_drive;
    private TurretSubsystem m_turret;

    public AimCommandFactory(DriveSubsystem drive, TurretSubsystem turret) {
        m_drive = drive;
        m_turret = turret;
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

    public Command MoveTurretToHeadingCommand(Angle heading) {
        return new RunCommand(() -> {
            MoveTurretToHeading(heading);
        }, m_turret);
    }

    public void MoveTurretToHeading(Angle heading) {
        Angle robotHeading = UtilityFunctions.WrapAngle(m_drive.getHeading());
        Angle robotRelativeTurretAngle = heading.minus(robotHeading);

        if (withinRange(TurretConstants.kFeedMinAngle1, TurretConstants.kFeedMaxAngle1, robotHeading)
                || withinRange(TurretConstants.kFeedMinAngle2, TurretConstants.kFeedMaxAngle2, robotHeading)) {
            m_turret.moveToAngle(robotRelativeTurretAngle);
        } else {
            // Gets which ray the robot is closest to
            Angle closest = getClosestAngle(heading, TurretConstants.kFeedMinAngle1, TurretConstants.kFeedMaxAngle1,
                    TurretConstants.kFeedMinAngle2, TurretConstants.kFeedMaxAngle2);

            // The overshoot is negative if the robot has to move in a negative direction;
            // same for positive
            Angle overshoot = robotRelativeTurretAngle.minus(closest).in(Degrees) > 0.0
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
        return a.gt(min) && a.lt(max);
    }

    private static Angle angleDiff(Angle a1, Angle a2) {
        a1 = UtilityFunctions.WrapTo180(a1);
        a2 = UtilityFunctions.WrapTo180(a2);

        return UtilityFunctions.WrapTo180(a1.minus(a2));
    }

    private static Angle getClosestAngle(Angle a, Angle... others) {
        a = UtilityFunctions.WrapTo180(a);

        Angle closest = UtilityFunctions.WrapTo180(others[others.length - 1]);
        double closestDistance = angleDiff(others[others.length - 1], a).abs(Degrees);

        for (int i = 0; i < others.length - 1; i++) {
            final double curDist = angleDiff(UtilityFunctions.WrapTo180(others[i]), a).abs(Degrees);

            if (curDist < closestDistance) {
                closestDistance = curDist;
                closest = others[i];
            }
        }

        return closest;
    }
}