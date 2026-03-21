// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.*;

public class RobotContainer {
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);

        private String m_autoSelected;

        private final TurretSubsystem m_turret = new TurretSubsystem();
        private final ShooterSubsystem m_shooter = new ShooterSubsystem();
        private final DriveSubsystem m_drive;
        private boolean m_intakeOut = false;

        CommandFactory m_commandFactory;
        Field2d m_field;

        // For getting data points for the lookup table
        Angle commandedShooterAngle;
        AngularVelocity commandedWheelVelocity;

        DoubleSubscriber m_hoodAngleGetter = DogLog.tunable("Hood Angle (in degrees)",
                        ShooterConstants.kHoodStartingAngle);
        DoubleSubscriber m_shooterVelocityGetter = DogLog.tunable("Motor Velocity (in RPM)",
                        ShooterConstants.kShooterStartVelocity);

        BooleanSubscriber m_zeroGyroGetter = DogLog.tunable("Zero Gyro", false);

        SendableChooser<String> m_sendable = new SendableChooser<>();

        public RobotContainer() {
                m_drive = new DriveSubsystem(m_turret::getRotationAtTime);
                m_commandFactory = new CommandFactory();
                m_commandFactory.SetSubsystems(m_drive, m_turret, m_shooter);

                NamedCommands.registerCommand("Deploy Intake",
                                m_commandFactory.DeployIntake());
                NamedCommands.registerCommand("Retract Intake",
                                m_commandFactory.RetractIntake());
                NamedCommands.registerCommand("Aim", m_commandFactory.AutoAimAtHubCommand());
                NamedCommands.registerCommand("Shoot",
                                m_commandFactory.ShootCommand().alongWith(m_commandFactory.RunAllStager())
                                                .finallyDo(() -> {
                                                        m_commandFactory.StopStaging();
                                                        m_commandFactory.StopShoot();
                                                }).raceWith(new WaitCommand(AutoConstants.kShootTime)));

                SmartDashboard.putNumber("Wheelspeed in rotations per second", 0.0);
                SmartDashboard.putNumber("Shooter hood angle in degrees", 0.0);
                SmartDashboard.putNumber("Turret angle in degrees", 0.0);

                m_sendable.addOption("Right Forward Auto", "Right Forward Auto");
                m_sendable.addOption("Left Forward Auto", "Left Forward Auto");
                m_sendable.addOption("Right Backwards Auto", "Right Backwards Auto");
                m_sendable.addOption("Left Backwards Auto", "Left Backwards Auto");
                m_sendable.addOption("Left Side Neutral Auto", "Left Side Neutral Auto");
                m_sendable.addOption("Right Side Neutral Auto", "Right Side Neutral Auto");
                m_sendable.setDefaultOption("Right Forward Auto", "Right Forward Auto");
                m_sendable.addOption("Simple Shoot Auto", "Simple Shoot Auto");

                // Configure the button bindings
                configureBindings();

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
                SmartDashboard.putData(m_sendable);
        }

        private void configureBindings() {
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

                // m_driverController.b().onTrue(new InstantCommand(() -> {
                // double shooterVelocity = m_shooterVelocityGetter.get();
                // double hoodAngle = m_hoodAngleGetter.get();
                // m_commandFactory.MoveHoodToAngle(Degrees.of(hoodAngle));
                // m_commandFactory.ShootAtVelocity(RPM.of(shooterVelocity));
                // System.out.println("Shooting at velocity of " + shooterVelocity + " RPM.");
                // System.out.println("Turret at angle " + hoodAngle + " Degrees");
                // }).andThen(new
                // WaitCommand(ShooterConstants.kRampTime)).andThen(m_commandFactory.RunAllStager())
                // .finallyDo(m_commandFactory::StopShoot));

                m_driverController.leftBumper().whileTrue(m_commandFactory.AimCommand(true))
                                .onFalse(m_commandFactory.StopAimCommand());
                m_driverController.rightBumper().whileTrue(m_commandFactory.AimCommand(false))
                                .onFalse(m_commandFactory.StopAimCommand());

                m_driverController.rightTrigger().whileTrue(m_commandFactory.RunAllStager())
                                .onTrue(Commands.waitUntil(m_shooter::AtWheelVelocityTarget).andThen(
                                                m_commandFactory.ShootCommand().until(
                                                                () -> m_driverController.rightTrigger()
                                                                                .getAsBoolean() == false)))
                                .onFalse(m_commandFactory.StopShootCommand()
                                                .alongWith(m_commandFactory.StopStagingCommand()));

                m_driverController.a().onTrue(new InstantCommand(m_drive::ZeroGyro));

                m_driverController.x().whileTrue(m_commandFactory.MoveTurretToFront());
                m_driverController.y().onTrue(m_commandFactory.ReverseStager())
                                .onFalse(m_commandFactory.StopStagingCommand());

                m_driverController.povUp().whileTrue(m_commandFactory.ClimbUpCommand());
                m_driverController.povDown().whileTrue(m_commandFactory.ClimbDownCommand());

                // m_driverController.leftTrigger()
                // .onTrue(m_commandFactory.DeployIntake().alongWith(m_commandFactory.SpinIntake()))
                // .onFalse(m_commandFactory.RetractIntake().alongWith(m_commandFactory.StopIntake()));

        

                m_driverController.leftTrigger()
                                .onTrue(new ConditionalCommand(
                                                m_commandFactory.DeployIntake()
                                                                .alongWith(m_commandFactory.SpinIntake()),
                                                m_commandFactory.RetractIntake()
                                                                .alongWith(m_commandFactory.StopIntake()),
                                                () -> {
                                                        m_intakeOut = !m_intakeOut;
                                                        return m_intakeOut;
                                                }));

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

        public Command getAutonomousCommand() {
                m_autoSelected = m_sendable.getSelected();

                DogLog.log("Auto Selected", m_autoSelected);

                return new PathPlannerAuto(m_autoSelected);
        }

        public Runnable pushTurretEncoderReading() {
                return () -> {
                        m_turret.pushCurrentEncoderReading();
                };
        }

        public Command feedPosition(Alliance alliance) {
                return new RunCommand(() -> {

                }, m_drive, m_turret);
        }

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

                double start = Timer.getFPGATimestamp();
                m_commandFactory.periodic();
                double end = Timer.getFPGATimestamp();

                if (m_zeroGyroGetter.get()) {
                        m_drive.ZeroGyro();
                }

                DogLog.log("Drivetrain command",
                                m_drive.getCurrentCommand() == null ? "null" : m_drive.getCurrentCommand().toString());
                DogLog.log("Turret Command",
                                m_turret.getCurrentCommand() == null ? "null"
                                                : m_turret.getCurrentCommand().toString());
                DogLog.log("Shooter command", m_shooter.getCurrentCommand() == null ? "null"
                                : m_shooter.getCurrentCommand().toString());
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
                return Degrees.of(SmartDashboard.getNumber("Shooter hood angle in degrees", 0.0));
        }

        private AngularVelocity getSmartdashboardRequestedWheelSpeed() {
                return RPM.of(SmartDashboard.getNumber("Wheelspeed in rotations per second", 0.0));
        }

        private Angle getSmartdashBoardRequestedTurretAngle() {
                return Degrees.of(SmartDashboard.getNumber("Turret angle in degrees", 0.0));
        }

        public void teleopInit() {
                // Configure default commands
                m_drive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_drive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                m_drive));
        }
}
