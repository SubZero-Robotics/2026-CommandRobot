package frc.robot.constants;

public final class ClimberConstants {
        public static final int kMotorCanId = 17;

        public static final double kUpVelocity = 0.2;
        public static final double kDownVelocity = -0.1;

        public static final double klimitMaxExtension = 3.0;
        public static final double kLimitMinExtension = 0.0;
        // TODO : Get the Constants for max and minimum distance for the climber

        public static final double kMaxExtension = klimitMaxExtension - 0.1;
        public static final double kMinExtension = kLimitMinExtension + 0.1;
}
