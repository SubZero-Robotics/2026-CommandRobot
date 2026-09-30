package frc.robot.commands;

import java.util.ArrayList;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Fixtures;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NumericalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.AimMath;
import frc.robot.utils.RobotGeometry;
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

    private final AimMath m_aimMath = new AimMath(ShooterConstants.kShootingEntries,
            ShooterConstants.kMaxStationaryVelocity);

    TargetSolution m_solution;

    public CommandFactory() {
        // m_drive = drive;
        // m_turret = turret;
        // m_shooter = shooter;
    }

    public Command AimCommand(boolean isFeedingLeftSide) {
        return new RunCommand(() -> {
            Aim(isFeedingLeftSide);
            m_isAiming = true;
        }, m_turret, m_shooter);
    }

    public void StopAim() {
        m_isAiming = false;
        m_shooter.MoveHoodToPosition(ShooterConstants.kDefaultHoodPosition);
        // Hand rotation back to the driver if Aim was still turning the drivetrain
        m_drive.disableFaceHeading();
    }

    public void SetSubsystems(DriveSubsystem drive, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_drive = drive;
        m_turret = turret;
        m_shooter = shooter;
    }

    public Command StopAimCommand() {
        return Commands.runOnce(this::StopAim);
    }

    public void periodic() {
        DogLog.log("In periodic command factor", true);
        m_solution = GetHubAimSolution();

        // While aiming, Aim() owns the wheel speed (e.g. the neutral-zone feed speed). Overwriting it here
        // would let the shoot command read the hub speed whenever it runs before the aim command.
        if (!m_isAiming) {
            m_wheelVelocity = m_solution.wheelSpeed();
        }

        DogLog.log("Turret Rotation in deg", m_turret.getRotation().in(Degrees));
        DogLog.log("RPM target", m_wheelVelocity.in(RPM));
        // Turret-to-hub distance used for the table lookup (to the lead-adjusted virtual target when moving)
        DogLog.log("Hub aim distance (m)", m_solution.distance().in(Meters));
        DogLog.log("Lead angle phi (deg)", m_solution.phi().in(Degrees));
        DogLog.log("In periodic command factor", false);
    }

    public void AimTurretToFront() {
        m_turret.moveToAngle(TurretConstants.kTurretTorwardsFront);
    }

    public Command AimTurretToFrontCommand() {
        return Commands.runOnce(this::AimTurretToFrontCommand);
    }

    public Command MoveHoodToDefaultPosition() {
        return Commands.runOnce(() -> m_shooter.MoveHoodToPosition(ShooterConstants.kDefaultHoodPosition));
    }

    public Command MoveTurretToFront() {
        return Commands.runOnce(() -> m_turret.moveToAngle(TurretConstants.kTurretTorwardsFront));
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
        return Commands.runOnce(this::AutoAimAtHub);
    }

    private void Aim(boolean isFeedingLeftSide) {
        Fixtures.FieldLocations location = m_drive.getRobotLocation();

        // No alliance from the Driver Station yet; switching on null would throw
        if (location == null) {
            return;
        }

        switch (location) {
            case AllianceSide: {

                TargetSolution solution;

                if (m_solution == null) {
                    solution = GetHubAimSolution();
                } else {
                    solution = m_solution;
                }

                MoveTurretToHeading(UtilityFunctions.subtractRotation(solution.hubAngle(), solution.phi()), true);
                // DogLog.log("Range from hub (meters)", solution.distance().in(Meters));
                // System.out.println(solution.phi());
                m_shooter.MoveHoodToPosition(solution.hoodAngle());
                m_wheelVelocity = solution.wheelSpeed();
                break;
            }
            case NeutralSide: {
                // Heading changes 180 degrees depending on which alliance you are on
                Angle offset = DriverStation.getAlliance().get() == Alliance.Red ? Degrees.of(0) : Degrees.of(180);
                Angle absHeading = isFeedingLeftSide ? UtilityFunctions.subtractRotation(offset, Fixtures.kFeedOffset)
                        : UtilityFunctions.addRotation(offset, Fixtures.kFeedOffset);

                absHeading = UtilityFunctions.WrapAngle(absHeading);

                m_shooter.MoveHoodToPosition(ShooterConstants.kHoodFeedingPosition);
                m_wheelVelocity = ShooterConstants.kFeedingWheelVelocity;
                MoveTurretToHeading(absHeading, true);
                break;
            }
            case OpponentSide:
                // Nothing to aim at from the opponent's side
            default:
                break;
        }
    }

    public Command AimHoodToPositionCommand(Angle angle) {
        return new RunCommand(() -> {
            m_shooter.MoveHoodToPosition(angle);
        }).until(m_shooter::AtHoodTarget);
    }

    public Command AimTurretRelativeToRobot(Angle angle) {
        return new RunCommand(() -> {
            m_turret.moveToAngle(angle);
        }, m_turret).until(m_turret::atTarget);
    }

    public Command RunAllStager() {
        return Commands.runOnce(() -> {
            m_stager.Agitate();
            m_stager.Feed();
            m_stager.Roll();
        }, m_stager);
    }

    public Command StopStagingCommand() {
        return Commands.runOnce(this::StopStaging);
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
        return new RunCommand(() -> {
            Shoot();
        }).finallyDo(this::StopShoot);
    }

    public void ShootAtVelocity(AngularVelocity velocity) {
        m_shooter.Spin(velocity);
    }

    public Command StopShootCommand() {
        return Commands.runOnce(() -> {
            StopShoot();
            m_wheelVelocity = NumericalConstants.kNoRotations;
        });
    }

    public void StopShoot() {
        m_shooter.Stop();
    }

    public Command StopIntake() {
        return Commands.runOnce(() -> m_intake.stopIntake());
    }

    public Command RetractIntake() {
        return Commands.runOnce(() -> m_intake.retractIntake()).andThen(StopIntake());
    }

    public Command OutTake() {
        return Commands.runOnce(() -> m_intake.spinIntake(IntakeConstants.kDefaultIntakeSpeed.times(-1)));
    }

    public Command DeployIntake() {
        return Commands.runOnce(() -> m_intake.deployIntake()).alongWith(SpinIntake());
    }

    public Command SpinIntake() {
        return Commands.runOnce(() -> m_intake.spinIntake(IntakeConstants.kDefaultIntakeSpeed));
    }

    public TargetSolution GetHubAimSolution() {
        Translation2d hubPosition = DriverStation.getAlliance().get() == Alliance.Blue ? Fixtures.kBlueAllianceHub
                : Fixtures.kRedAllianceHub;

        return m_aimMath.solve(m_drive.getPose(), TurretConstants.kTurretOffset,
                hubPosition, m_drive.getChassisSpeeds());
    }

    public Command MoveTurretToHeadingCommand(Angle heading) {
        return new RunCommand(() -> {
            MoveTurretToHeading(heading, true);
        }, m_turret);
    }

    public Command MoveHoodToAngleCommand(Angle angle) {
        return Commands.runOnce(() -> MoveHoodToAngle(angle));
    }

    public void MoveHoodToAngle(Angle angle) {
        // System.out.println("Move Hood to angle " + angle.in(Degrees) + " degrees.");
        m_shooter.MoveHoodToPosition(angle);
    }

    public Command PointAtHub(boolean isRed) {
        return new RunCommand(() -> {
            Translation2d hubPosition = isRed ? Fixtures.kRedAllianceHub : Fixtures.kBlueAllianceHub;
            Translation2d robotPose = m_drive.getPose().getTranslation();

            Angle angle = RobotGeometry.bearing(robotPose, hubPosition).getMeasure();

            MoveTurretToHeading(angle, true);
        }).finallyDo(m_drive::disableFaceHeading);
    }

    public void MoveTurretToHeading(Angle heading, boolean moveDrivetrain) {
        // Weird bug with red side position data
        Angle offset = DriverStation.getAlliance().get() == Alliance.Red && Robot.isReal()
                ? Constants.NumericalConstants.kHalfRotation
                : Constants.NumericalConstants.kNoRotation;

        heading = UtilityFunctions.addRotation(heading, offset);

        Angle robotHeading = UtilityFunctions.WrapAngle(m_drive.getHeading());

        Angle robotRelativeTurretAngle = UtilityFunctions.WrapAngle(
                UtilityFunctions.subtractRotation(heading, robotHeading));

        // Angle[] currentRange = getCurrentTurretRange();
        Angle[] currentRange = getCurrentTurretRange();

        if (withinAngles(currentRange, robotRelativeTurretAngle)) {
            m_turret.moveToAngle(robotRelativeTurretAngle);
        } else {
            // Gets which ray the robot is closest to
            Angle closest = UtilityFunctions.closestAngle(robotRelativeTurretAngle, false, currentRange);

            // The overshoot is negative if the robot has to move in a negative direction;
            // same for positive

            if (moveDrivetrain) {
                Angle overshoot = UtilityFunctions.angleDiff(robotRelativeTurretAngle, closest).in(Degrees) < 0.0
                        ? TurretConstants.kOvershootAmount
                        : TurretConstants.kOvershootAmount.times(-1.0);

                closest = UtilityFunctions.addRotation(closest, overshoot);

                Angle driveTarget = UtilityFunctions.subtractRotation(heading, closest);

                // System.out.println();
                m_drive.moveToAngle(driveTarget);
                m_turret.moveToAngle(closest);
            } else {
                m_turret.moveToAngle(closest);
            }
        }
    }

    public Command PointTurretToFixture(Pose2d fixture) {
        return new RunCommand(() -> {
            Pose2d robotPose = m_drive.getPose();

            // MoveTurretToHeading accepts a FIELD heading and converts it once.
            Angle angle = RobotGeometry.bearing(robotPose.getTranslation(), fixture.getTranslation()).getMeasure();

            MoveTurretToHeading(angle, true);
        }, m_turret);
    }

    public Command ReverseStager() {
        return Commands.runOnce(() -> {
            m_stager.reverseAgitater();
            m_stager.reverseRoller();
            m_stager.reverseFeeder();
        });
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
        return Commands.runOnce(() -> m_turret.moveToAngle(angle));
    }

    public Command ClimbUpCommand() {
        // return
        // MoveTurretToRobotRelativeHeadingCommand(TurretConstants.kTurretTorwardsFront)
        // .alongWith(Commands.waitUntil(m_turret::atTarget))

        return (new RunCommand(this::ClimbUp))
                // .until(m_climber::atMax) // TODO: put this back
                .finallyDo(this::StopClimb);
    }

    public Command ClimbDownCommand() {
        // return
        // MoveTurretToRobotRelativeHeadingCommand(TurretConstants.kTurretTorwardsFront)
        // .alongWith(Commands.waitUntil(m_turret::atTarget))

        return (new RunCommand(this::ClimbDown))
                // .until(m_climber::atMin) TODO: put this back
                .finallyDo(this::StopClimb);
    }

    // Aims the camera at april tags within range
    public Command IdleCameraAim() {

        // TODO: Finish
        return new ConditionalCommand(new RunCommand(() -> {
            Angle absoluteMinAngle = UtilityFunctions.addRotation(m_drive.getHeading(),
                    TurretConstants.kTurretCameraIdleViewMinAngle);
            Angle absoluteMaxAngle = UtilityFunctions.addRotation(m_drive.getHeading(),
                    TurretConstants.kTurretCameraIdleViewMaxAngle);
            Pose2d robotPose = m_drive.getPose();
            Angle robotRotation = robotPose.getRotation().getMeasure();
            Translation2d robotTranslation = robotPose.getTranslation();

            Angle toTagAngle = m_lockedTag == null ? null : RobotGeometry.bearing(robotTranslation, m_lockedTag).getMeasure();

            if (toTagAngle == null || !UtilityFunctions.withinArc(absoluteMinAngle, absoluteMaxAngle, toTagAngle)) {
                ArrayList<Translation2d> aprilTagsInView = aprilTagsWithinRange(absoluteMinAngle, absoluteMaxAngle,
                        robotTranslation);

                if (aprilTagsInView.isEmpty())
                    return;

                Translation2d closestTag = getClosestAngleApriltag(
                        UtilityFunctions.addRotation(robotRotation, TurretConstants.kTurretCameraMidPoint),
                        robotTranslation,
                        aprilTagsInView.toArray(Translation2d[]::new));

                Angle angleToTag = RobotGeometry.bearing(robotTranslation, closestTag).getMeasure();
                Angle turretRelativeAngleToTag = UtilityFunctions.WrapAngle(
                        UtilityFunctions.subtractRotation(angleToTag, robotRotation));

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

            Angle angleToTag = RobotGeometry.bearing(referenceTranslation, tag).getMeasure();

            if (UtilityFunctions.withinArc(min, max, angleToTag)) {
                anglesInRange.add(tag);
            }
        }

        return anglesInRange;
    }

    private static Translation2d getClosestAngleApriltag(Angle referenceAngle, Translation2d robot,
            Translation2d... tags) {
        if (tags.length == 0)
            return null;

        referenceAngle = UtilityFunctions.WrapAngle(referenceAngle);

        double closestDistance = Double.POSITIVE_INFINITY;
        Translation2d closestPosition = new Translation2d();

        for (Translation2d tag : tags) {
            Angle candidate = UtilityFunctions.WrapAngle(RobotGeometry.bearing(robot, tag).getMeasure());
            double dif = UtilityFunctions.angularDistance(referenceAngle, candidate);

            if (dif < closestDistance) {
                closestDistance = dif;
                closestPosition = tag;
            }
        }

        return closestPosition;
    }

    public Command AutoIntakeOut() {
        return Commands.runOnce(() -> {});
    }

    public Command Aim(Angle turretAngle, Angle hoodAngle) {
        return Commands.runOnce(() -> {
            m_turret.moveToAngle(turretAngle);
            m_shooter.MoveHoodToPosition(hoodAngle);
        });
    }

    public Command Shoot(AngularVelocity shooterWheelVelocity) {
        return new RunCommand(() -> {
            m_shooter.Spin(shooterWheelVelocity);
        }).finallyDo(m_shooter::Stop);
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
            if (UtilityFunctions.withinWindow(min, max, candidate))
                return true;
        }

        return false;
    }
}