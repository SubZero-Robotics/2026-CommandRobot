// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

    private final DriveSubsystem m_drive = new DriveSubsystem();

    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);

    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private final TurretSubsystem m_turret;

    public RobotContainer() {
        m_turret = new TurretSubsystem(m_drive::getPose);

        m_chooser.setDefaultOption("Example Auto", AutoConstants.kExampleAutoName);
        SmartDashboard.putData("Auto Choices", m_chooser);

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
    }

    private void configureBindings() {
        m_driverController.a()
                .whileTrue(
                        m_turret.pointToHeading(Degrees.of(90.0)));
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
}
