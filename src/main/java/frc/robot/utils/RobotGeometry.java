package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.NumericalConstants;

/** Hardware-independent geometry, with all translations in meters. */
public final class RobotGeometry {
    private RobotGeometry() {}

    public static Translation2d turretPosition(Pose2d robotPose, Translation2d robotRelativeOffset) {
        return robotPose.getTranslation().plus(robotRelativeOffset.rotateBy(robotPose.getRotation()));
    }

    /** Avoid Rotation2d's undefined-vector diagnostic for coincident points. */
    public static Rotation2d bearing(Translation2d displacement) {
        return displacement.getNorm() <= NumericalConstants.kEpsilon
                ? Rotation2d.kZero : displacement.getAngle();
    }

    public static Rotation2d bearing(Translation2d reference, Translation2d target) {
        return bearing(target.minus(reference));
    }

    /** Camera height is absolute robot-relative Z, not added to the turret-center Z. */
    public static Transform3d turretCameraTransform(Translation3d turretCenter, Distance radius,
            Distance height, Rotation2d mountingYaw, Rotation3d cameraRotation) {
        Translation2d offset = new Translation2d(radius, Meters.zero())
                .rotateBy(Rotation2d.fromRadians(cameraRotation.getZ()).minus(mountingYaw));
        Translation2d position = turretCenter.toTranslation2d().plus(offset);
        return new Transform3d(new Translation3d(position.getMeasureX(), position.getMeasureY(), height),
                cameraRotation);
    }
}
