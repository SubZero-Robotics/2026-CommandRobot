// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.commands.PathPlannerAuto;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Fixtures;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimCommandFactory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.*;

public class RobotContainer {

    private final IntakeSubsystem m_intake = new IntakeSubsystem();

    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);

    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private final TurretSubsystem m_turret = new TurretSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final DriveSubsystem m_drive = new DriveSubsystem(m_turret::getRotationAtTime);

    AimCommandFactory m_aimFactory = new AimCommandFactory(m_drive, m_turret, m_shooter);
    Field2d m_field;

    // For getting data points for the lookup table
    Angle commandedShooterAngle;
    AngularVelocity commandedWheelVelocity;

    DoubleSubscriber m_hoodAngleGetter = DogLog.tunable("Hood Angle (in degrees)", ShooterConstants.kHoodStartingAngle);
    DoubleSubscriber m_shooterVelocityGetter = DogLog.tunable("Motor Velocity (in RPM)",
            ShooterConstants.kShooterStartVelocity);

    public RobotContainer() {
        m_chooser.setDefaultOption("Example Auto", AutoConstants.kExampleAutoName);
        SmartDashboard.putData("Auto Choices", m_chooser);
        // SmartDashboard.putNumber("Wheelspeed in rotations per second", 0.0);
        // SmartDashboard.putNumber("Shooter hood angle in degrees", 0.0);
        // SmartDashboard.putNumber("Turret angle in degrees", 0.0);

        // Configure the button bindings
        configureBindings();

        // Configure default commands
        m_drive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_drive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true),
                        m_drive));

        m_field = m_drive.getField();
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

        m_driverController.x().whileTrue(spinIntake());

        m_driverController.y().onTrue(new InstantCommand(() -> {
            double shooterVelocity = m_shooterVelocityGetter.get();
            m_aimFactory.ShootAtVelocity(RPM.of(shooterVelocity));
            System.out.println("Shooting at velocity of " + shooterVelocity + " RPM.");
        }));

        m_driverController.x().onTrue(new InstantCommand(() -> {
            double hoodAngle = m_hoodAngleGetter.get();
            m_aimFactory.MoveHoodToAngle(Degrees.of(hoodAngle));
        }));

        // m_driverController.a().onTrue(m_aimFactory.MoveHoodToAbsoluteCommand(Degrees.of(15)));

        // m_driverController.b().onTrue(m_aimFactory.ShootCommand()).onFalse(m_aimFactory.StopShoot());

    }

    public Command getAutonomousCommand() {
        m_autoSelected = m_chooser.getSelected();

        System.out.print(m_autoSelected);

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

    public Command spinIntake() {
        return new RunCommand(() -> {
            m_intake.spinIntake(IntakeConstants.kDefaultIntakeSpeed);
        }).finallyDo(m_intake::stopIntake);
    }

    public Command retractIntake() {
        return new InstantCommand(() -> {
            m_intake.retractIntake();
        });
    }

    public Command outTake() {
        return new RunCommand(() -> {
            m_intake.spinIntake(IntakeConstants.kDefaultIntakeSpeed.times(-1));
        });
    }

    public Command DeployIntake() {
        return new InstantCommand(() -> {
            m_intake.deployIntake();
        });

    }

    public Command SpinIntake() {
        return new InstantCommand(() -> {
            m_intake.spinIntake(IntakeConstants.kDefaultIntakeSpeed);
        });
    }

    public void teleopPeriodic() {
        m_turret.addDriveHeading(UtilityFunctions.WrapAngle(m_drive.getHeading()));

        TargetSolution solution = m_aimFactory.GetHubAimSolution();

        Pose2d robotPose = m_drive.getPose();

        Distance xDist = Meters.of(solution.distance().in(Meters)
                * Math.cos(solution.hubAngle().minus(solution.phi()).in(Radians))).plus(robotPose.getMeasureX());
        Distance yDist = Meters.of(solution.distance().in(Meters)
                * Math.sin(solution.hubAngle().minus(solution.phi()).in(Radians))).plus(robotPose.getMeasureY());

        Pose2d targetPose = new Pose2d(xDist, yDist, new Rotation2d());

        m_field.getObject("targetPose").setPose(targetPose);
    }

    public void periodic() {
        commandedWheelVelocity = RPM.of(SmartDashboard.getNumber("Wheelspeed in rotations per second", 0.0));
        commandedShooterAngle = Degrees.of(SmartDashboard.getNumber("Shooter hood angle in degrees", 0.0));

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
}
