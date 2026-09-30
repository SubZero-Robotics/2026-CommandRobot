package frc.robot.opmodes.auto;

import java.util.List;

import org.wpilib.opmode.OpMode;

import dev.doglog.DogLog;
import frc.robot.Robot;

// Autonomous opmode for one of the PathPlanner autos. Robot registers one of these for each
// name in kAutoNames, so the auto is picked on the Driver Station instead of a SendableChooser.
public class PathPlannerAutoStub implements OpMode {
    // The autos that were in the SendableChooser. "Right Forward Auto" was the default.
    public static final List<String> kAutoNames = List.of(
            "Right Forward Auto",
            "Left Forward Auto",
            "Right Backwards Auto",
            "Left Backwards Auto",
            "Left Side Neutral Auto",
            "Right Side Neutral Auto",
            "Simple Shoot Auto");

    private final Robot m_robot;
    private final String m_autoName;

    public PathPlannerAutoStub(Robot robot, String autoName) {
        m_robot = robot;
        m_autoName = autoName;
    }

    @Override
    public void start() {
        DogLog.log("Auto Selected", m_autoName);

        // TODO: Restore when PathPlannerLib publishes a WPILib 2027 alpha-7 / Commands v3 build.
        // PathPlannerLib's only 2027 build targets WPILib alpha-5 and Commands v2, so the auto
        // does not run yet. Also restore the AutoBuilder setup in DriveSubsystem and the
        // NamedCommands in Robot.
        // Scheduler.getDefault().schedule(new PathPlannerAuto(m_autoName));
    }

    @Override
    public void periodic() {
        m_robot.teleopPeriodic();
    }
}
