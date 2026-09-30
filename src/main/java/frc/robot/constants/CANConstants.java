package frc.robot.constants;

import org.wpilib.hardware.bus.CANPort;

public final class CANConstants {
        // Systemcore CAN port that the robot's CAN bus is wired to. This replaces the
        // roboRIO's single "rio" bus.
        // TODO: Confirm which Systemcore port (CAN_S0 through CAN_S4) the bus is wired to
        public static final CANPort kCanPort = CANPort.CAN_S0;
}
