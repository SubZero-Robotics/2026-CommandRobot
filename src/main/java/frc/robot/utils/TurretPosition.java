package frc.robot.utils;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public record TurretPosition(Angle angle, AngularVelocity velocity,
                double timestamp) {
}
