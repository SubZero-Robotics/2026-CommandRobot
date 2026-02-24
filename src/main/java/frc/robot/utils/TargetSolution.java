package frc.robot.utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

// Phi is how much the aiming heading should be adjusted to properly correct for velocity
public record TargetSolution(Angle hoodAngle, AngularVelocity wheelSpeed, Angle phi, Distance distance,
        Angle hubAngle) {

}
