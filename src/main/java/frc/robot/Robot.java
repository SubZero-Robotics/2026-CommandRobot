// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import dev.doglog.DogLog;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.system.Timer;
import org.wpilib.driverstation.Alliance;
import org.wpilib.smartdashboard.Field2d;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.commands.CommandFactory;
import frc.robot.mechanisms.DriveSubsystem;
import frc.robot.mechanisms.ShooterSubsystem;
import frc.robot.mechanisms.TurretSubsystem;
import frc.robot.opmodes.auto.PathPlannerAutoStub;
import frc.robot.utils.*;

// Commands v3 robot. This class holds what used to be in RobotContainer. The teleop and
// autonomous opmodes in frc.robot.opmodes call into it the way TimedRobot's mode methods did.
public class Robot extends OpModeRobot {
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);

        private final TurretSubsystem m_turret = new TurretSubsystem();
        private final ShooterSubsystem m_shooter = new ShooterSubsystem();
        private final DriveSubsystem m_drive;
        private boolean m_intakeOut = false;

        CommandFactory m_commandFactory;
        Field2d m_field;

        // For getting data points for the lookup table
        Angle commandedShooterAngle;
        AngularVelocity commandedWheelVelocity;

        Tunable<Angle> m_hoodAngleGetter = DogLog.tunable("Hood Angle (in degrees)",
                        ShooterConstants.kHoodStartingAngle);
        Tunable<AngularVelocity> m_shooterVelocityGetter = DogLog.tunable("Motor Velocity (in RPM)",
                        ShooterConstants.kShooterStartVelocity);

        TunableBoolean m_zeroGyroGetter = DogLog.tunable("Zero Gyro", false);

        // SmartDashboard was removed in WPILib 2027, so these dashboard values are tunables.
        TunableDouble m_requestedWheelSpeed = Tunables.addDouble("Wheelspeed in rotations per second", 0.0);
        TunableDouble m_requestedShooterAngle = Tunables.addDouble("Shooter hood angle in degrees", 0.0);
        TunableDouble m_requestedTurretAngle = Tunables.addDouble("Turret angle in degrees", 0.0);

        public Robot() {
                m_drive = new DriveSubsystem(m_turret::getRotationAtTime);
                m_commandFactory = new CommandFactory();
                m_commandFactory.SetSubsystems(m_drive, m_turret, m_shooter);

                // TODO: Restore when PathPlannerLib publishes a WPILib 2027 alpha-7 / Commands v3
                // build. PathPlannerLib's only 2027 build targets WPILib alpha-5 and Commands v2.
                // NamedCommands.registerCommand("Deploy Intake",
                //                 m_commandFactory.DeployIntake());
                // NamedCommands.registerCommand("Retract Intake",
                //                 m_commandFactory.RetractIntake());
                // NamedCommands.registerCommand("Aim", m_commandFactory.AutoAimAtHubCommand());
                // NamedCommands.registerCommand("Shoot",
                //                 m_commandFactory.ShootCommand().alongWith(m_commandFactory.RunAllStager())
                //                                 .finallyDo(() -> {
                //                                         m_commandFactory.StopStaging();
                //                                         m_commandFactory.StopShoot();
                //                                 }).raceWith(new WaitCommand(AutoConstants.kShootTime)));

                // Each auto is its own autonomous opmode, picked on the Driver Station. This
                // replaces the SendableChooser.
                for (String autoName : PathPlannerAutoStub.kAutoNames) {
                        addOpMode(RobotMode.AUTONOMOUS, autoName, () -> new PathPlannerAutoStub(this, autoName));
                }
                publishOpModes();

                // Configure default commands
                // m_drive.setDefaultCommand(
                // // The left stick controls translation of the robot.
                // // Turning is controlled by the X axis of the right stick.
                // new RunCommand(
                // () -> m_drive.drive(
                // -MathUtil.applyDeadband(m_driverController.getLeftY(),
                // OIConstants.kDriveDeadband),
                // -MathUtil.applyDeadband(m_driverController.getLeftX(),
                // OIConstants.kDriveDeadband),
                // -MathUtil.applyDeadband(m_driverController.getRightX(),
                // OIConstants.kDriveDeadband),
                // true),
                // m_drive));

                m_field = m_drive.getField();

                // addPeriodic(m_robotContainer.pushTurretEncoderReading(),
                // Constants.TurretConstants.kEncoderReadInterval.in(Seconds));
        }

        @Override
        public void robotPeriodic() {
                Scheduler.getDefault().run();
                periodic();
        }

        // Called by the teleop opmode when it is selected on the Driver Station. The bindings
        // only apply while that opmode is selected.
        public void configureBindings() {
                // m_driverController.a()
                // .whileTrue(m_aimFactory.MoveTurretToHeadingCommand(Degrees.of(40)));

                // m_driverController.b()
                // .whileTrue(m_aimFactory.Aim(Degrees.of(SmartDashboard.getNumber("Turret angle
                // in degrees", 0.0)),
                // Degrees.of(SmartDashboard.getNumber("Shooter hood angle in degrees", 0.0))));

                // m_driverController.a().whileTrue(m_aimFactory.ShootCommand());

                // System.out.println("Bindings configured");
                // m_driverController.x().whileTrue(m_aimFactory.PointAtHub(true));

                // m_driverController.x().whileTrue(m_aimFactory.MoveTurretToHeadingCommand(Degrees.of(40)));

                // m_driverController.x().whileTrue(
                // m_aimFactory.Shoot(ShooterConstants.kFeedingWheelVelocity)
                // .finallyDo(() -> m_aimFactory.Shoot(RPM.of(0.0))));

                // m_driverController.y().whileTrue(m_aimFactory.RunAllStager());

                // m_driverController.rightBumper().whileTrue(m_aimFactory.AimCommand(false));
                // m_driverController.leftBumper().whileTrue(m_aimFactory.AimCommand(true));

                // m_driverController.a().whileTrue(m_aimFactory.RunAllStager());

                // m_driverController.y().onTrue(new InstantCommand(() -> {
                // double shooterVelocity = m_shooterVelocityGetter.get();
                // m_aimFactory.ShootAtVelocity(RPM.of(shooterVelocity));
                // System.out.println("Shooting at velocity of " + shooterVelocity + " RPM.");
                // }).andThen(new
                // WaitCommand(ShooterConstants.kRampTime)).andThen(m_aimFactory.RunAllStager())
                // .finallyDo(m_aimFactory::StopShoot));

                m_driverController.leftBumper().whileTrue(m_commandFactory.AimCommand(true))
                                .onFalse(m_commandFactory.StopAimCommand());
                m_driverController.rightBumper().whileTrue(m_commandFactory.AimCommand(false))
                                .onFalse(m_commandFactory.StopAimCommand());

                m_driverController.rightTrigger().whileTrue(m_commandFactory.RunAllStager())
                                .onTrue(Command.waitUntil(m_shooter::AtWheelVelocityTarget)
                                                .named("Wait For Shooter Velocity").andThen(
                                                                m_commandFactory.ShootCommand().until(
                                                                                () -> m_driverController.rightTrigger()
                                                                                                .getAsBoolean() == false)
                                                                                .named("Shoot Until Released"))
                                                .named("Shoot When At Velocity"))
                                .onFalse(m_commandFactory.StopShootCommand()
                                                .alongWith(m_commandFactory.StopStagingCommand())
                                                .named("Stop Shoot And Staging"));

                m_driverController.a().onTrue(CommandUtils.runOnce(m_drive::ZeroGyro).named("Zero Gyro"));

                m_driverController.x().whileTrue(m_commandFactory.MoveTurretToFront());
                m_driverController.y().onTrue(m_commandFactory.ReverseStager())
                                .onFalse(m_commandFactory.StopStagingCommand());

                // m_driverController.povUp().whileTrue(m_commandFactory.ClimbUpCommand());
                // m_driverController.povDown().whileTrue(m_commandFactory.ClimbDownCommand());

                // m_driverController.leftTrigger()
                // .onTrue(m_commandFactory.DeployIntake().alongWith(m_commandFactory.SpinIntake()))
                // .onFalse(m_commandFactory.RetractIntake().alongWith(m_commandFactory.StopIntake()));

                m_driverController.leftTrigger()
                                .onTrue(CommandUtils.conditional(
                                                m_commandFactory.DeployIntake()
                                                                .alongWith(m_commandFactory.SpinIntake())
                                                                .named("Deploy And Spin Intake"),
                                                m_commandFactory.RetractIntake()
                                                                .alongWith(m_commandFactory.StopIntake())
                                                                .named("Retract And Stop Intake"),
                                                () -> {
                                                        m_intakeOut = !m_intakeOut;
                                                        return m_intakeOut;
                                                }).named("Toggle Intake"));

                // m_driverController.povUp()
                // .whileTrue(m_commandFactory.ClimbDownCommand().finallyDo(m_commandFactory::StopClimb));
                // m_driverController.povDown()
                // .whileTrue(m_commandFactory.ClimbUpCommand().finallyDo(m_commandFactory::StopClimb));

                // m_driverController.x().onTrue(new InstantCommand(() -> {
                // // double hoodAngle = m_hoodAngleGetter.get();
                // // m_aimFactory.MoveHoodToAngle(Degrees.of(hoodAngle));
                // }));

                // m_driverController.a().onTrue(m_aimFactory.MoveHoodToAbsoluteCommand(Degrees.of(15)));

                // m_driverController.b().onTrue(m_aimFactory.ShootCommand()).onFalse(m_aimFactory.StopShoot());

        }

        public Runnable pushTurretEncoderReading() {
                return () -> {
                        m_turret.pushCurrentEncoderReading();
                };
        }

        public Command feedPosition(Alliance alliance) {
                return CommandUtils.runRepeatedly(() -> {

                }, m_drive, m_turret).named("Feed Position");
        }

        // Called periodically by the teleop and autonomous opmodes.
        public void teleopPeriodic() {
                DogLog.log("In Teleop Periodic Robotcontainer", true);
                m_turret.addDriveHeading(UtilityFunctions.WrapAngle(m_drive.getHeading()));

                // double solutionStart = Timer.getFPGATimestamp();
                // TargetSolution solution = m_commandFactory.GetHubAimSolution();
                // double solutionEnd = Timer.getFPGATimestamp();

                // Pose2d robotPose = m_drive.getPose();

                // Distance xDist = Meters.of(solution.distance().in(Meters)
                // * Math.cos(solution.hubAngle().minus(solution.phi()).in(Radians)))
                // .plus(robotPose.getMeasureX());
                // Distance yDist = Meters.of(solution.distance().in(Meters)
                // * Math.sin(solution.hubAngle().minus(solution.phi()).in(Radians)))
                // .plus(robotPose.getMeasureY());

                // Pose2d targetPose = new Pose2d(xDist, yDist, new Rotation2d());

                // m_field.getObject("targetPose").setPose(targetPose);

                double start = Timer.getTimestamp();
                m_commandFactory.periodic();
                double end = Timer.getTimestamp();

                if (m_zeroGyroGetter.get()) {
                        m_drive.ZeroGyro();
                }

                DogLog.log("Drivetrain command", CommandUtils.currentCommandName(m_drive));
                DogLog.log("Turret Command", CommandUtils.currentCommandName(m_turret));
                DogLog.log("Shooter command", CommandUtils.currentCommandName(m_shooter));
                DogLog.log("Time for command factory periodic in ms", (end - start) * 1000.0);
                DogLog.log("In Teleop Periodic Robotcontainer", false);
        }

        public void periodic() {
                // commandedWheelVelocity = RPM.of(SmartDashboard.getNumber("Wheelspeed in
                // rotations per second", 0.0));
                // commandedShooterAngle = Degrees.of(SmartDashboard.getNumber("Shooter hood
                // angle in degrees", 0.0));

                // DogLog.log("At Shooter Velocity Target", m_shooter.AtWheelVelocityTarget());

                // System.out.println(m_drive.getRobotLocation());
        }

        private Angle getSmartdashBoardRequestedShooterAngle() {
                return Degrees.of(m_requestedShooterAngle.get());
        }

        private AngularVelocity getSmartdashboardRequestedWheelSpeed() {
                return RPM.of(m_requestedWheelSpeed.get());
        }

        private Angle getSmartdashBoardRequestedTurretAngle() {
                return Degrees.of(m_requestedTurretAngle.get());
        }

        // Called by the teleop opmode when it starts (the old teleopInit).
        public void teleopInit() {
                // Configure default commands
                m_drive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                m_drive.runRepeatedly(
                                                () -> m_drive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true))
                                                .named("Joystick Drive"));
        }
}
