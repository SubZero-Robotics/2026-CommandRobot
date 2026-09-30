package frc.robot.constants;

import java.util.HashMap;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Translation2d;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

public final class Fixtures {
        public static final Translation2d kBlueAllianceHub = new Translation2d(Inches.of(182.11),
                        Inches.of(154.84));
        public static final Translation2d kRedAllianceHub = new Translation2d(Inches.of(651.22 - 182.11),
                        Inches.of(158.84));

        // From a top down perspective of the field with the red alliance on the left
        // side
        public static final Translation2d kTopFeedPose = new Translation2d();
        public static final Translation2d kBottomFeedPose = new Translation2d();

        public static final Distance kFieldYMidpoint = Inches.of(158.84);

        public static final Distance kBlueSideNeutralBorder = Inches.of(182.11);
        public static final Distance kRedSideNeutralBorder = Inches.of(651.22 - 182.11);

        public static enum FieldLocations {
                AllianceSide,
                NeutralSide,
                OpponentSide
        }

        public static final HashMap<FieldLocations, String> kFieldLocationStringMap = new HashMap<>();

        static {
                kFieldLocationStringMap.put(FieldLocations.AllianceSide, "Alliance Side");
                kFieldLocationStringMap.put(FieldLocations.NeutralSide, "Neutral Side");
                kFieldLocationStringMap.put(FieldLocations.OpponentSide, "Opponent Side");
        }

        // Placeholders
        public static final Angle kFeedOffset = Degrees.of(12);

        public static final Translation2d kRedHubAprilTag = Field
                        .loadField(Fields.FRC_2026_REBUILT_WELDED) // TODO: Change to normal field
                        .getTagPose(3).get().toPose2d().getTranslation();
}
