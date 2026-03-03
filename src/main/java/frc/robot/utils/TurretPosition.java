package frc.robot.utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public record TurretPosition(Angle angle, AngularVelocity velocity,
                double timestamp) {
}
