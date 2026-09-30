package frc.robot.utils;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;

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
