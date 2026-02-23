import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

class ShooterSybsystemTests {
    static final double DELTA = 1e-2; // acceptable deviation range for floating point compares
  ShooterSubsystem m_shooterSubsystem;
  private SparkMax m_shooterMotor; 
  private SparkMaxSim m_simShooterMotor;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_shooterSubsystem = new ShooterSubsystem(); // create our subsystem
    m_shooterMotor = new SparkMax(ShooterConstants.kShooterMotorId, MotorType.kBrushless);
    m_simShooterMotor = new SparkMaxSim(m_shooterMotor, DCMotor.getNEO(1));
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    m_shooterSubsystem.close(); // destroy our object
  }

  @Test // marks this method as a test
  void shouldStopSpinning() {
    m_simShooterMotor.StopShooting();
    assertEquals(
        0.0, m_simShooterMotor.getSpeed(), DELTA); // make sure that the value set to the motor is 0
  }
}