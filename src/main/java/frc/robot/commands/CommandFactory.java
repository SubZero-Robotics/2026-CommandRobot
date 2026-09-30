package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command3.Command;
import frc.robot.Robot;
import frc.robot.constants.Fixtures;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.NumericalConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.mechanisms.*;
import frc.robot.utils.CommandUtils;
import frc.robot.utils.ShootingEntry;
import frc.robot.utils.TargetSolution;
import frc.robot.utils.UtilityFunctions;

public class CommandFactory {

    private DriveSubsystem m_drive;
    private TurretSubsystem m_turret;
    private ShooterSubsystem m_shooter;

    private boolean m_isAiming = false;

    private AngularVelocity m_wheelVelocity = RPM.of(4000);

    private Translation2d m_lockedTag;

    private StagingSubsystem m_stager = new StagingSubsystem();
    private IntakeSubsystem m_intake = new IntakeSubsystem();
    private ClimberSubsystem m_climber = new ClimberSubsystem();

    TargetSolution m_solution;

    public CommandFactory() {
        // m_drive = drive;
        // m_turret = turret;
        // m_shooter = shooter;
    }

    public Command AimCommand(boolean isFeedingLeftSide) {
        return CommandUtils.runRepeatedly(() -> {
            Aim(isFeedingLeftSide);
            m_isAiming = true;
        }, m_turret, m_shooter).named("Aim");
    }

    public void StopAim() {
        m_isAiming = false;
        m_shooter.MoveHoodToPosition(ShooterConstants.kDefaultHoodPosition);
    }

    public void SetSubsystems(DriveSubsystem drive, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_drive = drive;
        m_turret = turret;
        m_shooter = shooter;
    }

    public Command StopAimCommand() {
        return CommandUtils.runOnce(this::StopAim).named("Stop Aim");
    }

    public void periodic() {
        DogLog.log("In periodic command factor", true);
        m_solution = GetHubAimSolution();

        m_wheelVelocity = m_solution.wheelSpeed();

        DogLog.log("Turret Rotation in deg", m_turret.getRotation().in(Degrees));
        DogLog.log("RPM target", m_wheelVelocity.in(RPM));
        DogLog.log("In periodic command factor", false);
    }

    public void AimTurretToFront() {
        m_turret.moveToAngle(TurretConstants.kTurretTorwardsFront);
    }

    public Command AimTurretToFrontCommand() {
        return CommandUtils.runOnce(this::AimTurretToFrontCommand).named("Aim Turret To Front");
    }

    public Command MoveHoodToDefaultPosition() {
        return CommandUtils.runOnce(() -> {
            m_shooter.MoveHoodToPosition(ShooterConstants.kDefaultHoodPosition);
        }).named("Move Hood To Default Position");
    }

    public Command MoveTurretToFront() {
        return CommandUtils.runOnce(() -> {
            m_turret.moveToAngle(TurretConstants.kTurretTorwardsFront);
        }).named("Move Turret To Front");
    }

    public void AutoAimAtHub() {
        TargetSolution solution;

        if (m_solution == null) {
            solution = GetHubAimSolution();
        } else {
            solution = m_solution;
        }

        m_wheelVelocity = solution.wheelSpeed();
        MoveTurretToHeading(solution.hubAngle(), false);
    }

    public Command AutoAimAtHubCommand() {
        return CommandUtils.runOnce(this::AutoAimAtHub).named("Auto Aim At Hub");
    }

    private void Aim(boolean isFeedingLeftSide) {
        Fixtures.FieldLocations location = m_drive.getRobotLocation();

        switch (location) {
            case AllianceSide: {

                TargetSolution solution;

                if (m_solution == null) {
                    solution = GetHubAimSolution();
                } else {
                    solution = m_solution;
                }

                MoveTurretToHeading(solution.hubAngle().minus (solution.phi()), true);
                // DogLog.log("Range from hub (meters)", solution.distance().in(Meters));
                // System.out.println(solution.phi());
                m_shooter.MoveHoodToPosition(solution.hoodAngle());
                m_wheelVelocity = solution.wheelSpeed();
                break;
            }
            case NeutralSide: {
                // Heading changes 180 degrees depending on which alliance you are on
                Angle offset = MatchState.getAlliance().get() == Alliance.RED ? Degrees.of(0) : Degrees.of(180);
                Angle absHeading = isFeedingLeftSide ? offset.minus(Fixtures.kFeedOffset)
                        : offset.plus(Fixtures.kFeedOffset);

                absHeading = UtilityFunctions.WrapAngle(absHeading);

                m_shooter.MoveHoodToPosition(ShooterConstants.kHoodFeedingPosition);
                m_wheelVelocity = ShooterConstants.kFeedingWheelVelocity;
                MoveTurretToHeading(absHeading, true);
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
        return CommandUtils.runRepeatedly(() -> {
            m_shooter.MoveHoodToPosition(angle);
        }).until(m_shooter::AtHoodTarget).named("Aim Hood To Position");
    }

    public Command AimTurretRelativeToRobot(Angle angle) {
        return CommandUtils.runRepeatedly(() -> {
            m_turret.moveToAngle(angle);
        }, m_turret).until(m_turret::atTarget).named("Aim Turret Relative To Robot");
    }

    public Command RunAllStager() {
        return CommandUtils.runOnce(() -> {
            m_stager.Agitate();
            m_stager.Feed();
            m_stager.Roll();
        }, m_stager).named("Run All Stager");
    }

    public Command StopStagingCommand() {
        return CommandUtils.runOnce(() -> {
            StopStaging();
        }).named("Stop Staging");
    }

    public void StopStaging() {
        m_stager.StopAgitate();
        m_stager.StopFeed();
        m_stager.StopRoll();
    }

    private void Shoot() {
        // System.out.println(m_wheelVelocity + " is wheel velocity");
        m_shooter.Spin(m_wheelVelocity);
    }

    public Command ShootCommand() {
        return CommandUtils.runRepeatedly(() -> {
            Shoot();
        }).whenCanceled(this::StopShoot).named("Shoot");
    }

    public void ShootAtVelocity(AngularVelocity velocity) {
        m_shooter.Spin(velocity);
    }

    public Command StopShootCommand() {
        return CommandUtils.runOnce(() -> {
            StopShoot();
            m_wheelVelocity = NumericalConstants.kNoRotations;
        }).named("Stop Shoot");
    }

    public void StopShoot() {
        m_shooter.Stop();
    }

    public Command StopIntake() {
        return CommandUtils.runOnce(() -> {
            m_intake.stopIntake();
        }).named("Stop Intake");
    }

    public Command RetractIntake() {
        return CommandUtils.runOnce(() -> {
            m_intake.retractIntake();
        }).named("Retract Intake Deploy Motors").andThen(StopIntake()).named("Retract Intake");
    }

    public Command OutTake() {
        return CommandUtils.runOnce(() -> {
            m_intake.spinIntake(IntakeConstants.kDefaultIntakeSpeed.times(-1));
        }).named("Out Take");
    }

    public Command DeployIntake() {
        return CommandUtils.runOnce(() -> {
            m_intake.deployIntake();
        }).named("Deploy Intake Deploy Motors").alongWith(SpinIntake()).named("Deploy Intake");
    }

    public Command SpinIntake() {
        return CommandUtils.runOnce(() -> {
            m_intake.spinIntake(IntakeConstants.kDefaultIntakeSpeed);
        }).named("Spin Intake");
    }

    public TargetSolution GetHubAimSolution() {
        Translation2d hubPosition = MatchState.getAlliance().get() == Alliance.BLUE ? Fixtures.kBlueAllianceHub
                : Fixtures.kRedAllianceHub;

        Pose2d robotPose = m_drive.getPose();

        Distance turretX = TurretConstants.kTurretCenterDistanceFromRobotCenter
                .times(Math.cos(robotPose.getRotation().getMeasure()
                        .plus(TurretConstants.kAngularDistanceToFrontOfRobot).in(Radians)))
                .plus(robotPose.getTranslation().getMeasureX());

        Distance turretY = TurretConstants.kTurretCenterDistanceFromRobotCenter
                .times(Math.sin(robotPose.getRotation().getMeasure()
                        .plus(TurretConstants.kAngularDistanceToFrontOfRobot).in(Radians)))
                .plus(robotPose.getTranslation().getMeasureY());

        Translation2d turretTranslation = new Translation2d(turretX, turretY);

        Translation2d translationToHub = hubPosition.minus(turretTranslation);

        Distance turretToHubDistance = Meters
                .of(Math.hypot(translationToHub.getMeasureY().in(Meters), translationToHub.getMeasureX().in(Meters)));
        Angle turretToHubAngle = Radians
                .of(Math.atan2(translationToHub.getMeasureY().in(Meters), translationToHub.getMeasureX().in(Meters)));

        ChassisVelocities robotSpeeds = m_drive.getChassisSpeeds();

        return getInterpolatedShootingParameters(turretToHubDistance,
                MetersPerSecond.of(robotSpeeds.vx), MetersPerSecond.of(robotSpeeds.vy),
                turretToHubAngle);
    }

    public Command MoveTurretToHeadingCommand(Angle heading) {
        return CommandUtils.runRepeatedly(() -> {
            MoveTurretToHeading(heading, true);
        }, m_turret).named("Move Turret To Heading");
    }

    public Command MoveHoodToAngleCommand(Angle angle) {
        return CommandUtils.runOnce(() -> {
            MoveHoodToAngle(angle);
        }).named("Move Hood To Angle");
    }

    public void MoveHoodToAngle(Angle angle) {
        // System.out.println("Move Hood to angle " + angle.in(Degrees) + " degrees.");
        m_shooter.MoveHoodToPosition(angle);
    }

    public Command PointAtHub(boolean isRed) {
        return CommandUtils.runRepeatedly(() -> {
            Translation2d hubPosition = isRed ? Fixtures.kRedAllianceHub : Fixtures.kBlueAllianceHub;
            Translation2d robotPose = m_drive.getPose().getTranslation();

            double dx = hubPosition.getMeasureX().minus(robotPose.getMeasureX()).in(Meters);
            double dy = hubPosition.getMeasureY().minus(robotPose.getMeasureY()).in(Meters);

            Angle angle = Radians.of(Math.atan2(dy, dx));

            MoveTurretToHeading(angle, true);
            System.out.println(angle);
        }).whenCanceled(m_drive::disableFaceHeading).named("Point At Hub");
    }

    public void MoveTurretToHeading(Angle heading, boolean moveDrivetrain) {
        // Weird bug with red side position data
        Angle offset = MatchState.getAlliance().get() == Alliance.RED && Robot.isReal()
                ? NumericalConstants.kHalfRotation
                : NumericalConstants.kNoRotation;

        heading = heading.plus(offset);

        Angle robotHeading = UtilityFunctions.WrapAngle(m_drive.getHeading());

        Angle robotRelativeTurretAngle = UtilityFunctions.WrapAngle(heading.minus(robotHeading));

        // Angle[] currentRange = getCurrentTurretRange();
        Angle[] currentRange = getCurrentTurretRange();

        if (withinAngles(currentRange, robotRelativeTurretAngle)) {
            m_turret.moveToAngle(robotRelativeTurretAngle);
        } else {
            // Gets which ray the robot is closest to
            Angle closest = getClosestAngle(robotRelativeTurretAngle, currentRange);

            // The overshoot is negative if the robot has to move in a negative direction;
            // same for positive

            if (moveDrivetrain) {
                Angle overshoot = UtilityFunctions.angleDiff(robotRelativeTurretAngle, closest).in(Degrees) < 0.0
                        ? TurretConstants.kOvershootAmount
                        : TurretConstants.kOvershootAmount.times(-1.0);

                closest = closest.plus(overshoot);

                Angle driveTarget = heading.minus(closest);

                // System.out.println();
                m_drive.moveToAngle(driveTarget);
                m_turret.moveToAngle(closest);
            } else {
                m_turret.moveToAngle(closest);
            }
        }
    }

    public Command PointTurretToFixture(Pose2d fixture) {
        return CommandUtils.runRepeatedly(() -> {
            Pose2d robotPose = m_drive.getPose();

            double dx = fixture.getX() - robotPose.getX();
            double dy = fixture.getY() - robotPose.getY();

            Angle angle = Radians.of(Math.atan2(dy, dx)).minus(Radians.of(robotPose.getRotation().getRadians()));

            MoveTurretToHeading(angle, true);
        }, m_turret).named("Point Turret To Fixture");
    }

    public Command ReverseStager() {
        return CommandUtils.runOnce(() -> {
            m_stager.reverseAgitater();
            m_stager.reverseRoller();
            m_stager.reverseFeeder();
        }).named("Reverse Stager");
    }

    public void ClimbUp() {
        m_climber.climbUp();
    }

    public void ClimbDown() {
        m_climber.climbDown();
    }

    public void StopClimb() {
        m_climber.Stop();
    }

    public Command MoveTurretToRobotRelativeHeadingCommand(Angle angle) {
        return CommandUtils.runOnce(() -> {
            m_turret.moveToAngle(angle);
        }).named("Move Turret To Robot Relative Heading");
    }

    public Command ClimbUpCommand() {
        // return
        // MoveTurretToRobotRelativeHeadingCommand(TurretConstants.kTurretTorwardsFront)
        // .alongWith(Commands.waitUntil(m_turret::atTarget))

        return CommandUtils.runRepeatedly(this::ClimbUp)
                // .until(m_climber::atMax) // TODO: put this back
                .whenCanceled(this::StopClimb).named("Climb Up");
    }

    public Command ClimbDownCommand() {
        // return
        // MoveTurretToRobotRelativeHeadingCommand(TurretConstants.kTurretTorwardsFront)
        // .alongWith(Commands.waitUntil(m_turret::atTarget))

        return CommandUtils.runRepeatedly(this::ClimbDown)
                // .until(m_climber::atMin) TODO: put this back
                .whenCanceled(this::StopClimb).named("Climb Down");
    }

    // Aims the camera at april tags within range
    public Command IdleCameraAim() {

        // TODO: Finish
        Command idleAim = CommandUtils.runRepeatedly(() -> {
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
        }, m_turret).named("Idle Camera Aim Loop");

        // Commands v3 has no ConditionalCommand. The v2 version passed null for the false
        // branch, so it does nothing when the turret is busy.
        return Command.noRequirements(coroutine -> {
            if (m_turret.getRunningCommands().isEmpty()) {
                coroutine.await(idleAim);
            }
        }).named("Idle Camera Aim");
    }

    private ArrayList<Translation2d> aprilTagsWithinRange(Angle min, Angle max, Translation2d referenceTranslation) {
        ArrayList<Translation2d> anglesInRange = new ArrayList<>();

        for (int i = 1; i <= 32; i++) {
            Translation2d tag = Field.loadField(Fields.FRC_2026_REBUILT_ANDY_MARK).getTagPose(i).get()
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
        Angle a1 = UtilityFunctions.WrapAngle(a);
        Angle min1 = UtilityFunctions.WrapAngle(min);
        Angle max1 = UtilityFunctions.WrapAngle(max);

        return a1.gt(min1) && a1.lt(max1);
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
        DogLog.log("In shooting entry search function", true);
        int low = 0;
        int high = ShooterConstants.kShootingEntries.length;
        int mid = 0;

        int i = 0;

        while (low < high) {
            mid = (low + high) / 2;
            ShootingEntry midEntry = ShooterConstants.kShootingEntries[mid];

            if (distance.gt(midEntry.distance())) {
                low = mid + 1;
            } else {
                high = mid;
            }

            i++;

            if (i > 20) {
                System.err.println("Shooting entry loop has exceeded 20 iterations.");
                return 1;
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

        DogLog.log("In shooting entry search function", false);

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

        DogLog.log("First Entry", firstEntry.toString());
        DogLog.log("Second Entry", secondEntry.toString());

        // DogLog.log("Last entry", firstEntry.toString());
        // DogLog.log("Next entry ", secondEntry.toString());

        return new TargetSolution(hoodAngle, wheelSpeed, phi, distance, turretAngle);
    }

    public Command AutoIntakeOut() {
        return CommandUtils.runOnce(() -> {

        }).named("Auto Intake Out");
    }

    public Command Aim(Angle turretAngle, Angle hoodAngle) {
        return CommandUtils.runOnce(() -> {
            m_turret.moveToAngle(turretAngle);
            m_shooter.MoveHoodToPosition(hoodAngle);
        }).named("Aim Turret And Hood");
    }

    public Command Shoot(AngularVelocity shooterWheelVelocity) {
        return CommandUtils.runRepeatedly(() -> {
            m_shooter.Spin(shooterWheelVelocity);
        }).whenCanceled(m_shooter::Stop).named("Shoot At Velocity");
    }

    // TODO: Make this better

    // Our valid shooting ranges are going to change based on the shooter hood
    // angle. If the hood angle is too low, then shooting the ball would lead to it
    // hitting the side of the robot or other balls currently being held in the
    // robot
    private Angle[] getCurrentTurretRange() {
        if (m_shooter.GetHoodAngle().gt(ShooterConstants.kTurretAngleRestrictiveShooterAngle)) {
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