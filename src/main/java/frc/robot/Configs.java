package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        // REVLib 2027 removed the SPARK conversion factors, so the SPARKs now report and
        // control in native units (motor rotations and RPM). These factors convert between
        // native units and the units the module code works in.
        public static final double kDrivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                / ModuleConstants.kDrivingMotorReduction; // meters per motor rotation
        public static final double kDrivingVelocityFactor = kDrivingFactor / 60.0; // meters per second per RPM
        public static final double kTurningFactor = 2 * Math.PI; // radians per rotation

        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate feed forward gain.
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    // The gains are scaled by kDrivingVelocityFactor so they act on the same
                    // meters per second error as they did with the old conversion factors.
                    .pid(0.04 * kDrivingVelocityFactor, 0, 0)
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward * kDrivingVelocityFactor);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    // The gains are scaled by kTurningFactor so they act on the same radian
                    // error as they did with the old conversion factors.
                    .pid(1 * kTurningFactor, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route. The wrap range is the absolute encoder's native range of one
                    // rotation, which matches the old 0 to 2 * pi radian input range.
                    .positionWrappingEnabled(true);
        }
    }
}
