package frc.robot.constants;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

public final class VisionConstants {

        public static final String kCameraName1 = "Photonvision";
        public static final String kCameraName2 = "Photonvision2";

        // Distance to fill pose3d z value assuming robot is on the ground
        public static Distance kEncoderZOffset = Inches.of(5.0);

        // Confidence of encoder readings for vision; should be tuned
        public static final double kEncoderConfidence = 0.15;

        public static final Transform3d kRobotToCamOne = new Transform3d(
                        new Translation3d(Inches.of(-3.854), Inches.of(-4.358), Inches.of(20.585)),
                        new Rotation3d(0, 180 - 23, 0));

        // These are not final numbers
        public static final Transform3d kRobotToCamTwo = new Transform3d(
                        new Translation3d(Inches.of(8.375), Inches.of(-2.16), Inches.of(-20.668)),
                        new Rotation3d(0, 0, 0));

        public static final Field kTagLayout = Field
                        .loadField(Fields.DEFAULT_FIELD);

        // Placeholder numbers
        public static final Distance kTurretCameraDistanceToCenter = Meters.of(0.13);
        public static final Distance kCameraTwoZ = Inches.of(18.0);

        public static final Translation3d kTurretCenterOfRotation = new Translation3d(Inches.of(-6.25),
                        Inches.of(6.151),
                        Inches.of(18));

        public static final Angle kCameraTwoPitch = Degrees.of(15.0);
        public static final Angle kCameraTwoRoll = Degrees.of(0.0);
        public static final Angle kCameraTwoYaw = Degrees.of(-21.0);

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(1, 1, 1);

        public static final AngularVelocity kMaxTurretVisionSpeed = RPM.of(30);
}
