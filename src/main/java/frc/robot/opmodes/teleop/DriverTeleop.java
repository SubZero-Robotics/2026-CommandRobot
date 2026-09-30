package frc.robot.opmodes.teleop;

import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

import frc.robot.Robot;

// The driver-controlled teleop opmode. It runs what the old TimedRobot teleop methods ran.
@Teleop
public class DriverTeleop implements OpMode {
    private final Robot m_robot;

    // Called when this opmode is selected on the Driver Station
    public DriverTeleop(Robot robot) {
        m_robot = robot;
        m_robot.configureBindings();
    }

    @Override
    public void start() {
        m_robot.teleopInit();
    }

    @Override
    public void periodic() {
        m_robot.teleopPeriodic();
    }
}
