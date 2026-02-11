// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utils.ShuffleboardPid;
import frc.robot.utils.VisionEstimation;
import frc.robot.utils.Vision;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Fixtures;
import frc.robot.Constants.NumericalConstants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

public class DriveSubsystem extends SubsystemBase {

    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    private boolean m_isManualRotate = true;
    private Angle m_targetAutoAngle = Radians.of(0.0);

    private double m_latestTime = Timer.getFPGATimestamp();

    private ShuffleboardPid m_pidController = new ShuffleboardPid(DriveConstants.kAutoRotationP,
            DriveConstants.kAutoRotationI, DriveConstants.kAutoRotationD, "Auto Rotate PID");

    // The gyro sensor
    private final Pigeon2 pidgey = new Pigeon2(DriveConstants.kPidgeyCanId, "rio");
    private final Pigeon2SimState m_simPidgey = pidgey.getSimState();

    private final Field2d m_field = new Field2d();

    private final Vision m_vision = new Vision(Optional.empty(), this::addVisionMeasurement);

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(pidgey.getYaw().getValue()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            new Rotation2d(pidgey.getYaw().getValue()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            }, new Pose2d(new Translation2d(), new Rotation2d()));

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {

        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();

            // TODO: Find a better solution to ensure config is initialized when
            // AutoBuilder.configure() is reached
            return;
        }

        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE
                // ChassisSpeeds. Also optionally outputs individual module
                // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                        // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }

                    return false;
                });
    }

    ChassisSpeeds getRobotRelativeSpeeds() {
        var fl = m_frontLeft.getState();
        var fr = m_frontRight.getState();
        var rl = m_rearLeft.getState();
        var rr = m_rearRight.getState();

        return DriveConstants.kDriveKinematics.toChassisSpeeds(fl, fr, rl, rr);
    }

    public Command moveToAngle(Angle angle) {
        return new InstantCommand(
                () -> {
                    m_isManualRotate = false;
                    m_targetAutoAngle = angle;
                });
    }

    public Command faceCardinalHeadingRange(Angle minAngle, Angle maxAngle) {
        return new InstantCommand(() -> {
            Angle robotAngle = getHeading();
            // System.out.println(robotAngle);

            if (withinRange(minAngle, maxAngle, robotAngle)) {
                m_isManualRotate = true;
            } else {
                m_isManualRotate = false;
                System.out.println(getClosestAngle(minAngle, maxAngle, robotAngle));
                m_targetAutoAngle = getClosestAngle(minAngle, maxAngle, robotAngle);
            }
        }, this);
    }

    public Command facePose(Pose2d fixture) {
        return new RunCommand(() -> {
            Pose2d robotPose = getPose();

            double xFixtureDist = fixture.getX() - robotPose.getX();
            double yFixtureDist = fixture.getY() - robotPose.getY();

            double totalDistance = Math.hypot(xFixtureDist, yFixtureDist);

            // Floating point value correction
            if (Math.abs(totalDistance) < NumericalConstants.kEpsilon)
                return;

            m_targetAutoAngle = Radians.of(Math.atan2(yFixtureDist, xFixtureDist));

            m_isManualRotate = false;
        });
    }

    public Command disableFaceHeading() {
        return new InstantCommand(() -> {
            m_isManualRotate = true;
        });
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block

        if (Robot.isSimulation()) {
            ChassisSpeeds chassisSpeed = DriveConstants.kDriveKinematics.toChassisSpeeds(
                    m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(),
                    m_rearRight.getState());

            // System.out.println(chassisSpeed);

            m_simPidgey.setSupplyVoltage(RobotController.getBatteryVoltage());
            m_simPidgey.setRawYaw(
                    getHeading().in(Degrees) + Radians.of(chassisSpeed.omegaRadiansPerSecond).in(Degrees)
                            * DriveConstants.kPeriodicInterval.in(Seconds));

            m_odometry.update(
                    new Rotation2d(getHeading()),
                    new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_rearLeft.getPosition(),
                            m_rearRight.getPosition()
                    });
        }

        // System.out.println("Current rotation: " +
        // getPose().getRotation().getRadians());

        m_poseEstimator.update(new Rotation2d(getHeading()), getModulePositions());
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        // System.out.println(m_poseEstimator.getEstimatedPosition());

        m_vision.periodic();

        SmartDashboard.putData(m_field);

        m_pidController.periodic();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public SwerveModulePosition[] getModulePositions() {

        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(), m_frontRight.getPosition(),
                m_rearLeft.getPosition(), m_rearRight.getPosition()
        };
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                new Rotation2d(pidgey.getYaw().getValue()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to
     *                      the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain

        if (!m_isManualRotate)
            System.out
                    .println("Setpoint: " + getOptimalAngle(m_targetAutoAngle, getHeading()).in(Radians) + ", Current: "
                            + getHeading().in(Radians));

        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeed.magnitude();
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeed.magnitude();
        double rotDelivered = (m_isManualRotate) ? rot * DriveConstants.kMaxAngularSpeed.magnitude()
                : m_pidController.calculate(getHeading().in(Radians),
                        getOptimalAngle(m_targetAutoAngle, getHeading()).in(Radians));

        System.out.println("Target " + m_targetAutoAngle + ", Current" + getHeading());

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                new Rotation2d(getHeading()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeed.magnitude());

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);

        double latestTime = Timer.getFPGATimestamp();
        double timeElapsed = latestTime - m_latestTime < 0.20 ? latestTime - m_latestTime
                : DriveConstants.kPeriodicInterval.in(Seconds);

        m_latestTime = latestTime;
    }

    public void drive(ChassisSpeeds speeds) {
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeed.magnitude());
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        pidgey.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Angle getHeading() {
        return pidgey.getYaw().getValue();
    }

    public void addVisionMeasurement(VisionEstimation estimation) {
        System.out.println(estimation.m_pose);
        m_poseEstimator.addVisionMeasurement(estimation.m_pose, estimation.m_timestamp, estimation.m_stdDevs);
    }

    public Fixtures.FieldLocations getRobotLocation() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d robotPose = getPose();

        double x = robotPose.getX();
        double y = robotPose.getY();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                if (x < Fixtures.kBlueSideNeutralBorder.in(Meters) && x > Fixtures.kRedSideNeutralBorder.in(Meters)) {
                    if (y < Fixtures.kFieldYMidpoint.in(Meters)) {
                        return Fixtures.FieldLocations.NeutralLeftSide;
                    } else {
                        return Fixtures.FieldLocations.NeutralRightSide;
                    }
                } else if (x > Fixtures.kBlueSideNeutralBorder.in(Meters)) {
                    return Fixtures.FieldLocations.AllianceSide;
                } else {
                    return Fixtures.FieldLocations.OpponentSide;
                }
            } else if (alliance.get() == Alliance.Red) {
                if (x > Fixtures.kRedSideNeutralBorder.in(Meters) && x < Fixtures.kBlueSideNeutralBorder.in(Meters)) {
                    if (y < Fixtures.kFieldYMidpoint.in(Meters)) {
                        return Fixtures.FieldLocations.NeutralRightSide;
                    } else {
                        return Fixtures.FieldLocations.NeutralLeftSide;
                    }
                } else if (x < Fixtures.kRedSideNeutralBorder.in(Meters)) {
                    return Fixtures.FieldLocations.AllianceSide;
                } else {
                    return Fixtures.FieldLocations.OpponentSide;
                }
            }
        }

        return null;
    }

    private static Angle wrapAngle(Angle heading) {
        Angle robotRotations = Radians
                .of(Math.floor(heading.in(Radians) / (2 * Math.PI)) * 2.0 * Math.PI);

        Angle wrap = heading.minus(robotRotations);

        if (wrap.lt(Radians.of(0))) {
            wrap = wrap.plus(TurretConstants.kFullRotation);
        }

        return wrap;
    }

    private static Angle getOptimalAngle(Angle target, Angle robotHeading) {
        Angle wrappedRobotAngle = wrapAngle(robotHeading);

        Angle delta = target.minus(wrappedRobotAngle);

        // Ensuring that the angle is always positive to ensure it is wrapped correctly
        if (delta.lt(Radians.of(0.0)))
            delta = delta.plus(Radians.of(2 * Math.PI));

        // Wrapping the delta to make it at most 180 deg
        if (delta.gt(Radians.of(Math.PI)))
            delta = delta.minus(Radians.of(2.0 * Math.PI));

        return delta.plus(robotHeading);
    }

    private static boolean withinRange(Angle min, Angle max, Angle angle) {
        angle = wrapAngle(angle);
        min = getOptimalAngle(angle, min);
        max = getOptimalAngle(angle, max);
        return angle.gt(max) && angle.lt(min);
    }

    private static Angle getClosestAngle(Angle t1, Angle t2, Angle angle) {
        t1 = wrapAngle(t1);
        t2 = wrapAngle(t2);
        angle = wrapAngle(angle);

        return t1.minus(angle).abs(Rotations) < t2.minus(angle).abs(Rotations) ? t1 : t2;
    }
}
