// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Fixtures;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimCommandFactory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.TargetSolution;
import frc.robot.utils.UtilityFunctions;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {

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

    ClimberSubsystem m_climber = new ClimberSubsystem();

    public RobotContainer() {
        m_chooser.setDefaultOption("Example Auto", AutoConstants.kExampleAutoName);
        // SmartDashboard.putData("Auto Choices", m_chooser);

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
        m_driverController.rightBumper().whileTrue(climbUp());
        m_driverController.leftBumper().whileTrue(climbDown());
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

    public Command climbUp() {
        return new RunCommand(() -> {
            m_climber.climbUp();
        })
                .until(m_climber::atMax)
                .finallyDo(() -> m_climber.Stop());
    }

    public Command climbDown() {
        return new RunCommand(() -> {
            m_climber.climbDown();
        })
                .until(m_climber::atMin)
                .finallyDo(() -> m_climber.Stop());
    }
}
