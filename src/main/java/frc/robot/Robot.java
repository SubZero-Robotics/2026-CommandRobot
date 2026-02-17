// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  ConnectorX cX = new ConnectorX();


  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  // ConnectorXAnimate m_leds;

  public Robot() {

    m_robotContainer = new RobotContainer();
}

  @Override
  public void robotInit () {
    //Connect to roboRIO USB ports
    boolean cxConnected = cX.Connect(USBPort.kUSB1);
    
    //check connection status
    if (cxConnected) {
      System.out.println("Device Connected!");
    }
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  
    // Register an event handler if you want to use ConnectorX to watch for sensor signals.
    cX.AddEventHandler((Event e) -> {
     switch (e.type) {
       case Connected:
         System.out.println("ConnectorX Error: Device Connected!");
         break;
        case Disconnected:
          System.out.println("ConnectorX Error: Device Disconnected!");
          break;
        case Error:
          System.out.println("ConnectorX Error: " + e);
          break;
       default:
         // do nothing. Will drain the event queue of events.
     }
  });
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    cX.leds
      .SetAnimation(Animation.Fill)
      .ForZone("LED-Strip")
      .WithColor(new Color(new Color8Bit(0,255,0)))
      .WithDelay(Units.Milliseconds.of(1000))
      .RunOnce(false);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
