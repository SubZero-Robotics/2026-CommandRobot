package frc.robot.utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public record ShootingEntry(Distance distance, AngularVelocity wheelVelocity, LinearVelocity muzzleVelocity,
        Distance maxHeight,
        Time timeOfFlight, Angle shooterAngle) {

}
