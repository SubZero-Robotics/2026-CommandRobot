// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.domain.led.Animation;
import com.lumynlabs.domain.event.Event;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
// Optional<LumynDeviceConfig> cfg = mCx.LoadConfigurationFromDeploy();
// cfg.ifPresent(mCx::ApplyConfiguration);



  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  // ConnectorXAnimate m_leds;

  public Robot() {

    m_robotContainer = new RobotContainer();
    addPeriodic(m_robotContainer.pushTurretEncoderReading(),
    Constants.TurretConstants.kEncoderReadInterval.in(Seconds));
}

  @Override
  public void robotInit () {
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  
    
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.getLedSubsystem().EnableLedSolid(Color.kAquamarine);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    m_robotContainer.getLedSubsystem().disableLeds();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
