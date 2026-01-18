// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
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
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

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

    private PIDController m_pidController = new PIDController(DriveConstants.kAutoRotationP,
            DriveConstants.kAutoRotationI, DriveConstants.kAutoRotationD);

    // The gyro sensor
    private final Pigeon2 pidgey = new Pigeon2(DriveConstants.kPidgeyCanId, "rio");
    private final Pigeon2SimState m_simPidgey = pidgey.getSimState();

    private final Field2d m_field = new Field2d();

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(pidgey.getYaw().getValue()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
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

    public Command enableFacePose(Pose2d fixture) {
        return new RunCommand(() -> {
            Pose2d robotPose = getPose();

            double xFixtureDist = fixture.getX() - robotPose.getX();
            double yFixtureDist = fixture.getY() - robotPose.getY();

            double angleToFixture = Math.atan2(yFixtureDist, xFixtureDist);

            System.out.println(angleToFixture);

            m_targetAutoAngle = Radians.of(angleToFixture);

            m_isManualRotate = false;
        });
    }

    public Command disableFacePose() {
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

        poseEstimator.update(new Rotation2d(getHeading()), getModulePositions());
        m_field.setRobotPose(poseEstimator.getEstimatedPosition());

        SmartDashboard.putData(m_field);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
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

        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeed.magnitude();
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeed.magnitude();
        double rotDelivered = (m_isManualRotate) ? rot * DriveConstants.kMaxAngularSpeed.magnitude()
                : m_pidController.calculate(getHeading().in(Radians), getOptimalAngle(m_targetAutoAngle).in(Radians));

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

    public Angle getNonContinuousHeading() {
        if (getHeading().in(Radians) > 2.0 * Math.PI) {
            return getHeading();
        }

        double rotations = Math.floor(getHeading().in(Radians) / (2 * Math.PI));
        return Radians.of(getHeading().in(Radians) - rotations);
    }

    private Angle getOptimalAngle(Angle target) {
        Angle robotHeading = getHeading();

        // Full robot rotations in radians
        Angle robotRotations = Radians.of(Radians.convertFrom(Math.floor(robotHeading.in(Radians) / (2 * Math.PI)), Rotations));

        // Both are the same angle, just one is negative and one is positive
        Angle pTargetAngle = robotRotations.plus(target);
        Angle nTargetAngle = robotRotations.plus(target.minus(Radians.of(2 * Math.PI)));

        // If either angle is less than 180 degrees relative to the robot's current angle, it is the most optimal path
        if (robotHeading.minus(pTargetAngle).abs(Radians) < Math.PI) {
            return pTargetAngle;
        }

        return nTargetAngle;
    }
}
