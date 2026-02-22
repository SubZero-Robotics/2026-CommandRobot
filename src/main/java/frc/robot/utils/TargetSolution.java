package frc.robot.utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public record TargetSolution(Angle hoodAngle, AngularVelocity wheelSpeed, Angle phi, Distance distance,
        Angle hubAngle) {

}
