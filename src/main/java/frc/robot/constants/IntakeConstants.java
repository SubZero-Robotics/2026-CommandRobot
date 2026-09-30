package frc.robot.constants;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public final class IntakeConstants {
        public static final double kP1 = 0.05;
        public static final double kI1 = 0;
        public static final double kD1 = 0;

        public static final double kP2 = 0.05;
        public static final double kI2 = 0;
        public static final double kD2 = 0;

        public static final double kPIn = 0.0003;
        public static final double kIIn = 0;
        public static final double kDIn = 0.0005;
        public static final double kFFIn = 0.00192;

        public static final int kDeployMotor1Id = 13;
        public static final int kDeployMotor2Id = 8;
        public static final int kIntakeMotorId = 7;

        // 10 teeth on pinion, 20 teeth on rack
        public static final Angle kDeployRotations = Rotations.of(9.6);
        public static final Angle kRetractRotations = Rotations.of(0.0);

        public static final Angle kMaxExtension = Rotations.of(9.6);
        public static final Angle kMinExtension = Rotations.of(0.0);

        public static final int kDeployMotorCurrentLimit = 60;
        public static final int kIntakeMotorCurrentLimit = 80;

        public static final AngularVelocity kDefaultIntakeSpeed = RPM.of(-2200);
}
