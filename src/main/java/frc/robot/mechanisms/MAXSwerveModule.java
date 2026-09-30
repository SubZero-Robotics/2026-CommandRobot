// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Distance;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radian;
import static org.wpilib.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;
import frc.robot.Robot;
import frc.robot.constants.CANConstants;
import frc.robot.constants.DriveConstants;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private LinearVelocity m_simDriverEncoderVelocity = MetersPerSecond.of(0.0);
  private Distance m_simDriverEncoderPosition = Meters.of(0.0);
  private Angle m_simCurrentAngle = Radians.of(0.0);

  private double m_chassisAngularOffset = 0;
  private SwerveModuleVelocity m_desiredState = new SwerveModuleVelocity(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkFlex(CANConstants.kCanPort, drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(CANConstants.kCanPort, turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(getTurningEncoderRadians());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */

  public void updateSimDriverPosition(SwerveModuleVelocity desiredState) {
    m_simDriverEncoderVelocity = MetersPerSecond.of(desiredState.velocity);
    m_simDriverEncoderPosition = m_simDriverEncoderPosition
        .plus(m_simDriverEncoderVelocity.times(DriveConstants.kPeriodicInterval));
  }

  public SwerveModuleVelocity getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.

    if (Robot.isReal())
      return new SwerveModuleVelocity(
          m_drivingEncoder.getVelocity().get() * Configs.MAXSwerveModule.kDrivingVelocityFactor,
          new Rotation2d(getTurningEncoderRadians() - m_chassisAngularOffset));

    return new SwerveModuleVelocity(m_simDriverEncoderVelocity,
        new Rotation2d(m_simCurrentAngle.in(Radian) - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.

    if (Robot.isReal())
      return new SwerveModulePosition(
          m_drivingEncoder.getPosition().get() * Configs.MAXSwerveModule.kDrivingFactor,
          new Rotation2d(getTurningEncoderRadians() - m_chassisAngularOffset));

    return new SwerveModulePosition(m_simDriverEncoderPosition,
        new Rotation2d(m_simCurrentAngle.in(Radian) - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleVelocity desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleVelocity correctedDesiredState = new SwerveModuleVelocity();
    correctedDesiredState.velocity = desiredState.velocity;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState = correctedDesiredState.optimize(new Rotation2d(getTurningEncoderRadians()));

    // Command driving and turning SPARKS towards their respective setpoints. The SPARKs
    // work in native units (RPM and rotations), so convert from meters per second and
    // radians.
    m_drivingClosedLoopController.setSetpoint(
        correctedDesiredState.velocity / Configs.MAXSwerveModule.kDrivingVelocityFactor, ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(
        correctedDesiredState.angle.getRadians() / Configs.MAXSwerveModule.kTurningFactor, ControlType.kPosition);

    m_desiredState = desiredState;

    if (Robot.isSimulation()) {
      updateSimDriverPosition(correctedDesiredState);
      m_simCurrentAngle = Radians.of(correctedDesiredState.angle.getRadians());
    }
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  // The turning encoder reports native rotations, so convert to radians.
  private double getTurningEncoderRadians() {
    return m_turningEncoder.getPosition().get() * Configs.MAXSwerveModule.kTurningFactor;
  }
}