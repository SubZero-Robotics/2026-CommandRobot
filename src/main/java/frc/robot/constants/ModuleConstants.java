package frc.robot.constants;

public final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).

        public static final int kDrivingMotorPinionTeeth = 14;
        public static final int kSpurGearTeeth = 21;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 21 teeth on the first-stage spur gear, 14
        // teeth on the bevel pinion

        public static final double kDrivingMotorReduction = (45.0 * kSpurGearTeeth)
                        / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                        * kWheelCircumferenceMeters)
                        / kDrivingMotorReduction;
}
