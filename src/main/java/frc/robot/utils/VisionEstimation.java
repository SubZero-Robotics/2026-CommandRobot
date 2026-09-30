package frc.robot.utils;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

public class VisionEstimation {
    public Pose2d m_pose;
    public double m_timestamp;
    public Matrix<N3, N1> m_stdDevs;

    public VisionEstimation(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        m_pose = pose;
        m_timestamp = timestamp;
        m_stdDevs = stdDevs;
    }
}
