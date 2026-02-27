// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {

    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final ClimberSubsystem m_climb = new ClimberSubsystem();

    private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public RobotContainer() {
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
        m_driverController.x().whileTrue(m_drive.enableFacePose(new Pose2d()));
        m_driverController.x().whileFalse(m_drive.disableFacePose());

        m_driverController.rightBumper().whileTrue(climbUp());
        m_driverController.leftBumper().whileTrue(climbDown());
    }

    public Command getAutonomousCommand() {
        m_autoSelected = m_chooser.getSelected();

        System.out.print(m_autoSelected);

        return new PathPlannerAuto(m_autoSelected);
    }

    public Command climbUp() {
        return new RunCommand(() -> {
            m_climb.climbUp();
        })
        .until(m_climb::atMax)
        .finallyDo(() -> m_climb.Stop());
    }

    public Command climbDown() {
        return new RunCommand(() -> {
            m_climb.climbDown();
        })
        .until(m_climb::atMin)
        .finallyDo(() -> m_climb.Stop());
    }
}
