package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public record ShootingEntry(Distance distance, AngularVelocity wheelVelocity, LinearVelocity muzzleVelocity,
        Distance maxHeight,
        Time timeOfFlight, Angle shooterAngle) {

    @Override
    public String toString() {
        return "Distance (meters): " + distance.in(Meters) + ", Angular Wheel Velocity (RPM): " + wheelVelocity.in(RPM)
                + ", Muzzle Velocity (MPS): " + muzzleVelocity + ", Max height (Meters): "
                + maxHeight.in(Meters) + "Time of Flight (seconds): " + timeOfFlight.in(Seconds)
                + ", Shooter Angle (Degrees): " + shooterAngle.in(Degrees);
    }
}
