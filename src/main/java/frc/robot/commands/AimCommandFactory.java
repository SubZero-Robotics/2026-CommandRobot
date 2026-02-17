package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

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
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

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

    public Command HoldTurretHeading(Angle heading) {
        return new RunCommand(() -> {
            System.out.println(m_drive.getHeading());
            m_turret.moveToAngle(heading.minus(m_drive.getHeading()));
        }, m_turret);
    }

    public Command PointTurretToFixture(Pose2d fixture) {
        return new RunCommand(() -> {
            Pose2d robotPose = m_drive.getPose();

            double dx = fixture.getX() - robotPose.getX();
            double dy = fixture.getY() - robotPose.getY();

            Angle angle = Radians.of(robotPose.getRotation().getRadians()).minus(Radians.of(Math.atan2(dy, dx)));

            m_turret.moveToAngle(angle);
        }, m_turret);
    }
}