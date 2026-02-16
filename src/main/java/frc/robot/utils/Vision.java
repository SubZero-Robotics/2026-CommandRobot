package frc.robot.utils;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class Vision {

    PhotonCamera m_camera1 = new PhotonCamera(VisionConstants.kCameraName1);
    PhotonCamera m_camera2 = new PhotonCamera(VisionConstants.kCameraName2);

    Optional<Function<Double, TurretPosition>> m_turretPositionSupplier;
    Supplier<AngularVelocity> m_robotAngularVelocitySupplier;

    PhotonPoseEstimator m_poseEstimatorOne = new PhotonPoseEstimator(VisionConstants.kTagLayout,
            VisionConstants.kRobotToCamOne);
    PhotonPoseEstimator m_poseEstimatorTwo = new PhotonPoseEstimator(VisionConstants.kTagLayout,
            VisionConstants.kRobotToCamTwo);

    Consumer<VisionEstimation> m_visionConsumer;
    private Matrix<N3, N1> curStdDevs;

    public Vision(Optional<Function<Double, TurretPosition>> turretPositionSupplier,
            Consumer<VisionEstimation> visionConsumer, Supplier<AngularVelocity> robotAngularVelocitySupplier) {
        m_turretPositionSupplier = turretPositionSupplier;
        m_visionConsumer = visionConsumer;
        m_robotAngularVelocitySupplier = robotAngularVelocitySupplier;
    }

    public void periodic() {
        Optional<EstimatedRobotPose> visionEstimationCameraOne = Optional.empty();

        for (var result : m_camera1.getAllUnreadResults()) {
            visionEstimationCameraOne = m_poseEstimatorOne.estimateLowestAmbiguityPose(result);

            if (visionEstimationCameraOne.isEmpty()) {
                visionEstimationCameraOne = m_poseEstimatorOne.estimateLowestAmbiguityPose(result);
            }

            updateEstimationStdDevs(visionEstimationCameraOne, result.getTargets(), m_poseEstimatorOne);

            visionEstimationCameraOne.ifPresent(estimation -> {
                m_visionConsumer.accept(new VisionEstimation(estimation.estimatedPose.toPose2d(),
                        estimation.timestampSeconds, getCurrentStdDevs()));
            });
        }

        Optional<EstimatedRobotPose> visionEstimationCameraTwo = Optional.empty();
        for (var result : m_camera2.getAllUnreadResults()) {
            Transform3d cameraTransform;

            cameraTransform = getTurretCameraTransform(result.getTimestampSeconds());

            if (cameraTransform == null) {
                System.out.println("Turret exceeded max velocity valid for reading april tags.");
                break;
            }

            m_poseEstimatorTwo.setRobotToCameraTransform(cameraTransform);
            visionEstimationCameraTwo = m_poseEstimatorTwo.estimateCoprocMultiTagPose(result);

            if (visionEstimationCameraTwo.isEmpty()) {
                visionEstimationCameraTwo = m_poseEstimatorTwo.estimateLowestAmbiguityPose(result);
            }

            updateEstimationStdDevs(visionEstimationCameraTwo, result.getTargets(), m_poseEstimatorTwo);

            visionEstimationCameraTwo.ifPresent(estimation -> {
                m_visionConsumer.accept(new VisionEstimation(estimation.estimatedPose.toPose2d(),
                        estimation.timestampSeconds, getCurrentStdDevs()));
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

    private Transform3d getTurretCameraTransform(double estimationTime) {
        if (m_turretPositionSupplier.isEmpty())
            return VisionConstants.kRobotToCamTwo;

        TurretPosition turretPosition = m_turretPositionSupplier.get().apply(estimationTime);

        // Getting the net velocity of the turret relative to the field
        if (turretPosition == null || turretPosition.velocity().plus(
                m_robotAngularVelocitySupplier.get()).abs(DegreesPerSecond) > VisionConstants.kMaxTurretVisionSpeed
                        .in(DegreesPerSecond)) {
            return null;
        }

        Distance cameraX = VisionConstants.kTurretCameraDistanceToCenter
                .times(Math.cos(turretPosition.angle().in(Radians)))
                .plus(VisionConstants.kTurretAxisOfRotation.getMeasureX());

        Distance cameraY = VisionConstants.kTurretCameraDistanceToCenter
                .times(Math.sin(turretPosition.angle().in(Radians)))
                .plus(VisionConstants.kTurretAxisOfRotation.getMeasureY());

        Translation3d cameraPosition = new Translation3d(cameraX, cameraY,
                VisionConstants.kCameraTwoZ);

        Rotation3d cameraRotation = new Rotation3d(VisionConstants.kCameraTwoRoll, VisionConstants.kCameraTwoPitch,
                turretPosition.angle().plus(VisionConstants.kCameraTwoYaw));

        return new Transform3d(cameraPosition, cameraRotation);
    }

    private Matrix<N3, N1> getCurrentStdDevs() {
        return curStdDevs;
    }
}
