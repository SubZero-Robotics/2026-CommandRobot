package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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
