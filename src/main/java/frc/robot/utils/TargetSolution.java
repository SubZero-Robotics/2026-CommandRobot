package frc.robot.utils;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

// Phi is how much the aiming heading should be adjusted to properly correct for velocity
public record TargetSolution(Angle hoodAngle, AngularVelocity wheelSpeed, Angle phi, Distance distance,
                Angle hubAngle) {

}
