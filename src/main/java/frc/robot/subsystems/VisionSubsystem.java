package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.VisionEstimation;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera m_camera1 = new PhotonCamera(VisionConstants.kCameraName1);
    PhotonCamera m_camera2 = new PhotonCamera(VisionConstants.kCameraName2);

    Supplier<Angle> m_turretAngleSupplier;

    PhotonPoseEstimator m_poseEstimatorOne = new PhotonPoseEstimator(null, VisionConstants.kRobotToCamOne);
    PhotonPoseEstimator m_poseEstimatorTwo = new PhotonPoseEstimator(null, VisionConstants.kRobotToCamTwo);

    Consumer<VisionEstimation> m_visionConsumer;
    private Matrix<N3, N1> curStdDevs;

    public VisionSubsystem(Supplier<Angle> turretAngleSupplier, Consumer<VisionEstimation> visionConsumer) {
        m_turretAngleSupplier = turretAngleSupplier;
        m_visionConsumer = visionConsumer;
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> visionEstimationCameraOne = Optional.empty();

        for (var result : m_camera1.getAllUnreadResults()) {
            visionEstimationCameraOne = m_poseEstimatorOne.estimateCoprocMultiTagPose(result);

            if (visionEstimationCameraOne.isEmpty()) {
                visionEstimationCameraOne = m_poseEstimatorOne.estimateLowestAmbiguityPose(result);
            }

            updateEstimationStdDevs(visionEstimationCameraOne, result.getTargets(), m_poseEstimatorOne);

            visionEstimationCameraOne.ifPresent(estimation -> {
                m_visionConsumer.accept(new VisionEstimation(estimation.estimatedPose.toPose2d(),
                        Timer.getFPGATimestamp(), getCurrentStdDevs()));
            });
        }

        Optional<EstimatedRobotPose> visionEstimationCameraTwo = Optional.empty();
        m_poseEstimatorTwo.setRobotToCameraTransform(getTurretCameraTransform());

        for (var result : m_camera2.getAllUnreadResults()) {
            visionEstimationCameraTwo = m_poseEstimatorTwo.estimateCoprocMultiTagPose(result);

            if (visionEstimationCameraTwo.isEmpty()) {
                visionEstimationCameraTwo = m_poseEstimatorTwo.estimateLowestAmbiguityPose(result);
            }

            updateEstimationStdDevs(visionEstimationCameraTwo, result.getTargets(), m_poseEstimatorTwo);

            visionEstimationCameraTwo.ifPresent(estimation -> {
                m_visionConsumer.accept(new VisionEstimation(estimation.estimatedPose.toPose2d(),
                        Timer.getFPGATimestamp(), getCurrentStdDevs()));
            });
        }
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator poseEstimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    private Transform3d getTurretCameraTransform() {
        Angle turretAngle = m_turretAngleSupplier.get();

        Distance cameraXOffset = Meters
                .of(VisionConstants.kTurretCameraDistanceToCenter.in(Meters) * Math.cos(turretAngle.in(Radians)));
        Distance cameraYOffset = Meters
                .of(VisionConstants.kTurretCameraDistanceToCenter.in(Meters) * Math.sin(turretAngle.in(Radians)));

        Distance cameraX = VisionConstants.kTurretAxisOfRotation.getMeasureX().plus(cameraXOffset);
        Distance cameraY = VisionConstants.kTurretAxisOfRotation.getMeasureY().plus(cameraYOffset);
        Distance cameraZ = VisionConstants.kTurretAxisOfRotation.getMeasureZ();

        return new Transform3d(cameraX, cameraY, cameraZ,
                new Rotation3d(VisionConstants.kCameraTwoPitch, VisionConstants.kCameraTwoRoll, turretAngle));
    }

    private Matrix<N3, N1> getCurrentStdDevs() {
        return curStdDevs;
    }
}
