package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.constants.CanIdConstants;
import frc.robot.util.hardware.Sparks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** The extender must stop, and stay stopped, whenever its position can't be trusted. */
public class ExtenderSafetyTest {
  // One instance for the class: REVLib allows only one SparkMax object per CAN ID.
  private static ExtenderSubsystem m_extender;
  private static SparkRelativeEncoderSim m_leaderEncoder;
  private static SparkRelativeEncoderSim m_followerEncoder;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    if (m_extender == null) {
      m_extender = new ExtenderSubsystem();
      m_leaderEncoder = new SparkRelativeEncoderSim(spark(CanIdConstants.kExtenderMotor1));
      m_followerEncoder = new SparkRelativeEncoderSim(spark(CanIdConstants.kExtenderMotor2));
    }
    // Start every test retracted, homed and fault-free.
    m_leaderEncoder.setPosition(0);
    m_followerEncoder.setPosition(0);
    m_extender.finishHoming(true);
    assertTrue(m_extender.isHomed());
    assertFalse(m_extender.isFaulted());
  }

  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
    DriverStationSim.resetData();
  }

  private static SparkMax spark(int canId) {
    return Sparks.all().stream().filter(s -> s.getDeviceId() == canId).findFirst().orElseThrow();
  }

  /** Advance one 20 ms loop with both sides at the given position. */
  private void loop(double meters) {
    m_leaderEncoder.setPosition(meters);
    m_followerEncoder.setPosition(meters);
    SimHooks.stepTiming(0.02);
    m_extender.periodic();
  }

  /** Drive out to the given position at a realistic speed. */
  private void moveTo(double meters) {
    for (double pos = 0; pos < meters; pos += 0.004) { // 0.2 m/s
      loop(pos);
    }
    loop(meters);
  }

  @Test
  void normalMotionIsNotMistakenForAReset() {
    moveTo(Units.inchesToMeters(10));
    assertTrue(m_extender.isHomed());
    assertFalse(m_extender.isFaulted());
  }

  @Test
  void sparkRebootWhileExtendedStopsAndUnhomes() {
    moveTo(Units.inchesToMeters(10));
    // A rebooted SPARK's encoder snaps back to 0 while the box is still out.
    loop(0.0);
    assertTrue(m_extender.isFaulted(), "encoder jump must trip the safety");
    assertFalse(m_extender.isHomed(), "position is no longer trusted");

    // Stays stopped on later loops; it does not recover on its own.
    loop(0.0);
    loop(0.0);
    assertTrue(m_extender.isFaulted());
    assertFalse(m_extender.isHomed());
  }

  @Test
  void failedHomingLeavesExtenderStoppedAndUnhomed() {
    moveTo(Units.inchesToMeters(6));
    m_extender.finishHoming(false); // timed out or interrupted
    assertFalse(m_extender.isHomed());

    // Later loops must not bring it back.
    loop(Units.inchesToMeters(6));
    loop(Units.inchesToMeters(6));
    assertFalse(m_extender.isHomed());
  }

  @Test
  void successfulHomingRecoversFromAFault() {
    moveTo(Units.inchesToMeters(10));
    loop(0.0); // trip the reset detection
    assertTrue(m_extender.isFaulted());

    m_extender.finishHoming(true);
    assertTrue(m_extender.isHomed());
    assertFalse(m_extender.isFaulted());
    // Zeroing during homing is expected and must not trip the jump check.
    loop(0.0);
    assertFalse(m_extender.isFaulted());
  }

  @Test
  void jumpDetectionThresholds() {
    assertFalse(ExtenderSubsystem.isEncoderJump(0.10, 0.11, 0.02), "1 cm in 20 ms is real motion");
    assertTrue(ExtenderSubsystem.isEncoderJump(0.25, 0.0, 0.02), "25 cm in 20 ms is a reset");
    assertFalse(ExtenderSubsystem.isEncoderJump(0.25, 0.0, 0.0), "no time step, no judgement");
    // A slow 200 ms loop at top speed (~9 cm) is still real motion.
    assertFalse(ExtenderSubsystem.isEncoderJump(0.10, 0.19, 0.2));
  }
}
