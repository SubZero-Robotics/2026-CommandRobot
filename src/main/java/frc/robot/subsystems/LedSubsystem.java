package frc.robot.subsystems;

import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.domain.event.Event;
import com.lumynlabs.domain.led.Animation;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase{

    private String m_LedStrip = "LED-Strip";

    ConnectorX cX = new ConnectorX();

    public LedSubsystem() {

            //Connect to roboRIO USB ports
        boolean cxConnected = cX.Connect(USBPort.kUSB1);
        
        //check connection status
        if (cxConnected) {
        System.out.println("ConnectorX Connected!");
        }

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

    public void EnableLedSolid(Color color) {

        cX.leds
      .SetAnimation(Animation.Fill)
      .ForZone(m_LedStrip)
      .WithColor(color)
      .WithDelay(Units.Milliseconds.of(0))
      .RunOnce(false);
    }

    public void DisableLedSolid(Color color) {

        cX.leds
      .SetAnimation(Animation.Fill)
      .ForZone(m_LedStrip)
      .WithColor(color)
      .WithDelay(Units.Milliseconds.of(0))
      .RunOnce(false);
      
    }

    public void disableLeds() {
        cX.leds.SetColor(m_LedStrip, Color.kBlack);
    }


}
