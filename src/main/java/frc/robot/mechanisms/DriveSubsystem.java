// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.mechanisms;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;
import frc.robot.Robot;
import frc.robot.utils.CommandUtils;
import frc.robot.utils.ShuffleboardPid;
import frc.robot.utils.TurretPosition;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.VisionEstimation;
import frc.robot.utils.Vision;
import frc.robot.constants.CANConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.Fixtures;
import frc.robot.constants.NumericalConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.DriveConstants.RangeType;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
// TODO: Restore when PathPlannerLib publishes a WPILib 2027 alpha-7 / Commands v3 build
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import dev.doglog.DogLog;

import static org.wpilib.units.Units.*;

import java.util.Optional;
import java.util.function.Function;

public class DriveSubsystem implements Mechanism {

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

    private double m_latestTime = Timer.getTimestamp();

    private ShuffleboardPid m_pidController = new ShuffleboardPid(DriveConstants.kAutoRotationP,
            DriveConstants.kAutoRotationI, DriveConstants.kAutoRotationD, "Auto Rotate PID");

    // The gyro sensor
    private final Pigeon2 pidgey = new Pigeon2(DriveConstants.kPidgeyCanId, new CANBus(CANConstants.kCanPort));
    private final Pigeon2SimState m_simPidgey = pidgey.getSimState();

    private final Field2d m_field = new Field2d();

    private final Vision m_vision;

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
    public DriveSubsystem(Function<Double, TurretPosition> turretPositionSupplier) {

        m_vision = new Vision(Optional.of(turretPositionSupplier), this::addVisionMeasurement,
                this::getAngularVelocity);

        Scheduler.getDefault().addPeriodic(this::periodic);

        // Usage reporting for MAXSwerve template
        HAL.reportUsage("RobotDrive", "MAXSwerve");

        // TODO: Restore when PathPlannerLib publishes a WPILib 2027 alpha-7 / Commands v3 build.
        // PathPlannerLib's only 2027 build targets WPILib alpha-5 and Commands v2.
        // RobotConfig config;
        // try {
        //     config = RobotConfig.fromGUISettings();
        // } catch (Exception e) {
        //     e.printStackTrace();
        //
        //     // TO DO: Find a better solution to ensure config is initialized when
        //     // AutoBuilder.configure() is reached
        //     return;
        // }
        //
        // AutoBuilder.configure(
        //         () -> getPose(), // Robot pose supplier
        //         (Pose2d pose) -> resetOdometry(pose), // Method to reset odometry (will be called if your auto has a
        //                                               // starting pose)
        //         () -> getRobotRelativeSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //         (speeds, feedforwards) -> drive(speeds, "Path planner"), // Method that will drive the robot given ROBOT
        //                                                                  // RELATIVE
        //         // ChassisSpeeds. Also optionally outputs individual module
        //         // feedforwards
        //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
        //                 // holonomic drive trains
        //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        //         ),
        //         config,
        //         () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red
        //             // alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        //             var alliance = MatchState.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == Alliance.RED;
        //             }
        //
        //             return false;
        //         });
    }

    ChassisVelocities getRobotRelativeSpeeds() {
        var fl = m_frontLeft.getState();
        var fr = m_frontRight.getState();
        var rl = m_rearLeft.getState();
        var rr = m_rearRight.getState();

        return DriveConstants.kDriveKinematics.toChassisVelocities(fl, fr, rl, rr);
    }

    public Command moveToAngleCommand(Angle angle) {
        return CommandUtils.runOnce(
                () -> {
                    moveToAngle(angle);
                }).named("Move To Angle");
    }

    public void moveToAngle(Angle angle) {
        m_isManualRotate = false;
        m_targetAutoAngle = angle;

        System.out.println("Is Manual Rotate is False in moveToAngle()");
    }

    public void moveByAngle(Angle angle) {
        m_isManualRotate = false;
        System.out.println("Is Manual Rotate is False in moveByAngle()");
        m_targetAutoAngle = getHeading().plus(angle);
    }

    public RangeType faceCardinalHeadingRange(Angle minAngle, Angle maxAngle) {
        Angle robotAngle = getHeading();
        // System.out.println(robotAngle);

        if (withinRange(minAngle, maxAngle, robotAngle)) {
            m_isManualRotate = true;
            return RangeType.Within;
        } else {
            m_isManualRotate = false;
            System.out.println("Is Manual Rotate is False in faceCardinalHeadingRange");
            m_targetAutoAngle = getClosestAngle(minAngle, maxAngle, robotAngle);
            return m_targetAutoAngle.isEquivalent(minAngle) ? RangeType.CloseMin : RangeType.CloseMax;
        }
    }

    public Command facePose(Pose2d fixture) {
        return CommandUtils.runRepeatedly(() -> {
            Pose2d robotPose = getPose();

            double xFixtureDist = fixture.getX() - robotPose.getX();
            double yFixtureDist = fixture.getY() - robotPose.getY();

            double totalDistance = Math.hypot(xFixtureDist, yFixtureDist);

            // Floating point value correction
            if (Math.abs(totalDistance) < NumericalConstants.kEpsilon)
                return;

            m_targetAutoAngle = Radians.of(Math.atan2(yFixtureDist, xFixtureDist));

            m_isManualRotate = false;
            System.out.println("Is Manual Rotate is False in facePose()");
        }).named("Face Pose");
    }

    public void disableFaceHeading() {
        m_isManualRotate = true;
    }

    public void periodic() {
        DogLog.log("In periodic drive subsystem", true);
        // Update the odometry in the periodic block
        double start = Timer.getTimestamp();

        if (Robot.isSimulation()) {
            ChassisVelocities chassisSpeed = DriveConstants.kDriveKinematics.toChassisVelocities(
                    m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(),
                    m_rearRight.getState());

            // System.out.println(chassisSpeed);

            m_simPidgey.setSupplyVoltage(RobotController.getBatteryVoltage());
            m_simPidgey.setRawYaw(
                    getGyroHeading().in(Degrees) + Radians.of(chassisSpeed.omega).in(Degrees)
                            * DriveConstants.kPeriodicInterval.in(Seconds));

            m_odometry.update(
                    new Rotation2d(getGyroHeading()),
                    new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_rearLeft.getPosition(),
                            m_rearRight.getPosition()
                    });
        }

        if (!m_isManualRotate
                && UtilityFunctions.WrapAngle(UtilityFunctions.WrapAngle(getHeading()).minus(m_targetAutoAngle))
                        .abs(Degrees) < DriveConstants.kTurnToAngleTolerance.in(Degrees)) {
            m_isManualRotate = true;
        }

        // System.out.println("Current rotation: " +
        // getPose().getRotation().getRadians());

        m_poseEstimator.update(new Rotation2d(getGyroHeading()), getModulePositions());
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        // System.out.println(m_poseEstimator.getEstimatedPosition());

        m_vision.periodic();

        m_pidController.periodic();

        Telemetry.log("Field", m_field);

        double end = Timer.getTimestamp();

        DogLog.log("Drivetrain periodic time (ms)", (end - start) * 1000.0);

        DogLog.log("In periodic drive subsystem", false);

        // SmartDashboard.putBoolean("Is manual rotate", m_isManualRotate);

        // DogLog.log("X dist to april tag in meters",
        // getPose().getTranslation().minus(Fixtures.kRedHubAprilTag).getX());
        // DogLog.log("Y dist to april tag in meters",
        // getPose().getTranslation().minus(Fixtures.kRedHubAprilTag).getY());
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

    public void ZeroGyro() {
        pidgey.setYaw(NumericalConstants.kNoRotation);
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
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

        // if (!m_isManualRotate)
        // System.out
        // .println("Setpoint: " + getOptimalAngle(m_targetAutoAngle,
        // getHeading()).in(Radians) + ", Current: "
        // + getHeading().in(Radians));

        final double latestTime = Timer.getTimestamp();
        final double timeElapsed = latestTime - m_latestTime < 0.20 ? latestTime - m_latestTime
                : DriveConstants.kPeriodicInterval.in(Seconds);

        m_latestTime = latestTime;

        if (Math.abs(rot) > OIConstants.kDriveDeadband) {
            m_isManualRotate = true;
        }

        final double pidCalculation = m_pidController.calculate(getHeading().in(Radians),
                getOptimalAngle(m_targetAutoAngle, getHeading()).in(Radians));

        final double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeed.magnitude();
        final double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeed.magnitude();
        final double rotDelivered = (m_isManualRotate)
                ? rot * DriveConstants.kMaxAngularSpeed.magnitude()
                : pidCalculation;

        // System.out.println("Target " + m_targetAutoAngle + ", Current" +
        // getHeading());

        // final var swerveModuleStates =
        // DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
        // fieldRelative
        // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
        // rotDelivered,
        // new Rotation2d(getHeading()))
        // : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered),
        // timeElapsed));
        // SwerveDriveKinematics.desaturateWheelSpeeds(
        // swerveModuleStates, DriveConstants.kMaxSpeed.magnitude());

        var speeds = (fieldRelative
                ? new ChassisVelocities(xSpeedDelivered, ySpeedDelivered, rotDelivered)
                        .toRobotRelative(new Rotation2d(getHeading()))
                : new ChassisVelocities(xSpeedDelivered, ySpeedDelivered, rotDelivered))
                .discretize(timeElapsed);

        drive(speeds, "Joystick runner");

        // m_frontLeft.setDesiredState(swerveModuleStates[0]);
        // m_frontRight.setDesiredState(swerveModuleStates[1]);
        // m_rearLeft.setDesiredState(swerveModuleStates[2]);
        // m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void drive(ChassisVelocities speeds, String caller) {
        var states = DriveConstants.kDriveKinematics.toSwerveModuleVelocities(speeds);

        DogLog.log("First commanded motor speeds", states[0]);
        DogLog.log("Caller", caller);

        states = SwerveDriveKinematics.desaturateWheelVelocities(states, DriveConstants.kMaxSpeed.magnitude());

        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_rearLeft.setDesiredState(states[2]);
        m_rearRight.setDesiredState(states[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleVelocity[] desiredStates) {
        desiredStates = SwerveDriveKinematics.desaturateWheelVelocities(
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
        // return m_poseEstimator.getEstimatedPosition().getRotation().getMeasure();
    }

    public Angle getGyroHeading() {
        return pidgey.getYaw().getValue();
    }

    public void addVisionMeasurement(VisionEstimation estimation) {
        // System.out.println("Vision applied.");
        m_poseEstimator.addVisionMeasurement(estimation.m_pose,
                estimation.m_timestamp, estimation.m_stdDevs);
    }

    public ChassisVelocities getChassisSpeeds() {

        return DriveConstants.kDriveKinematics.toChassisVelocities(m_frontLeft.getState(), m_frontRight.getState(),
                m_rearLeft.getState(), m_rearRight.getState())
                .toFieldRelative(new Rotation2d(getHeading()));

    }

    public Field2d getField() {
        return m_field;
    }

    public Fixtures.FieldLocations getRobotLocation() {
        Optional<Alliance> alliance = MatchState.getAlliance();
        Pose2d robotPose = getPose();

        double x = robotPose.getX();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.BLUE) {
                if (x > Fixtures.kBlueSideNeutralBorder.in(Meters) && x < Fixtures.kRedSideNeutralBorder.in(Meters)) {
                    return Fixtures.FieldLocations.NeutralSide;
                } else if (x < Fixtures.kBlueSideNeutralBorder.in(Meters)) {
                    return Fixtures.FieldLocations.AllianceSide;
                } else {
                    return Fixtures.FieldLocations.OpponentSide;
                }
            } else if (alliance.get() == Alliance.RED) {
                if (x < Fixtures.kRedSideNeutralBorder.in(Meters) && x > Fixtures.kBlueSideNeutralBorder.in(Meters)) {
                    return Fixtures.FieldLocations.NeutralSide;
                } else if (x > Fixtures.kRedSideNeutralBorder.in(Meters)) {
                    return Fixtures.FieldLocations.AllianceSide;
                } else {
                    return Fixtures.FieldLocations.OpponentSide;
                }
            }
        }

        return null;
    }

    private AngularVelocity getAngularVelocity() {
        return DegreesPerSecond.of(pidgey.getAngularVelocityZDevice().getValueAsDouble());
    }

    private static Angle getOptimalAngle(Angle target, Angle robotHeading) {
        Angle wrappedRobotAngle = UtilityFunctions.WrapAngle(robotHeading);

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
        angle = UtilityFunctions.WrapAngle(angle);
        min = getOptimalAngle(angle, min);
        max = getOptimalAngle(angle, max);
        return angle.gt(max) && angle.lt(min);
    }

    private static Angle getClosestAngle(Angle t1, Angle t2, Angle angle) {
        t1 = UtilityFunctions.WrapAngle(t1);
        t2 = UtilityFunctions.WrapAngle(t2);
        angle = UtilityFunctions.WrapAngle(angle);

        return t1.minus(angle).abs(Rotations) < t2.minus(angle).abs(Rotations) ? t1 : t2;
    }
}
