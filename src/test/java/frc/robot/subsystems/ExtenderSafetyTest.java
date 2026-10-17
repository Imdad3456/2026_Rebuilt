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
import frc.robot.constants.ExtenderConstants;
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
  private static com.revrobotics.sim.SparkMaxSim m_leaderSim;

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
      m_leaderSim = new com.revrobotics.sim.SparkMaxSim(spark(CanIdConstants.kExtenderMotor1),
          edu.wpi.first.math.system.plant.DCMotor.getNEO(1));
    }
    m_leaderSim.setAppliedOutput(0); // not pushing unless a test says so
    m_leaderSim.setMotorCurrent(0);
    m_leaderEncoder.setVelocity(0);
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
    return (SparkMax) Sparks.all().stream().filter(s -> s.getDeviceId() == canId).findFirst().orElseThrow();
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

  /** Pull the leader ahead of the follower gradually (not a jump) until the skew check trips. */
  private void skewLeaderAhead(double meters) {
    for (double pos = 0; pos <= meters; pos += 0.004) {
      m_leaderEncoder.setPosition(pos);
      m_followerEncoder.setPosition(0);
      SimHooks.stepTiming(0.02);
      m_extender.periodic();
    }
  }

  @Test
  void clearFaultsResumesAfterSkew() {
    skewLeaderAhead(Units.inchesToMeters(3));
    assertTrue(m_extender.isFaulted(), "skew must trip");
    assertTrue(m_extender.isHomed(), "skew doesn't make the encoders untrustworthy");

    // Clearing only sticks once the skew is actually gone: walk the leader back to the follower
    // gradually (a sudden change would look like an encoder reset).
    for (double pos = Units.inchesToMeters(3); pos > 0; pos -= 0.004) {
      m_leaderEncoder.setPosition(pos);
      m_followerEncoder.setPosition(0);
      SimHooks.stepTiming(0.02);
      m_extender.periodic();
    }
    loop(0.0);
    assertTrue(m_extender.isFaulted(), "the fault stays latched until cleared");

    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(m_extender.clearFaults());
    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().run();
    assertFalse(m_extender.isFaulted());
    assertTrue(m_extender.isHomed(), "resumes without re-homing");
  }

  @Test
  void clearFaultsDoesNotTrustAResetEncoder() {
    moveTo(Units.inchesToMeters(10));
    loop(0.0); // SPARK reboot: encoder jumps to 0
    assertTrue(m_extender.isFaulted());

    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(m_extender.clearFaults());
    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().run();
    assertFalse(m_extender.isFaulted());
    assertFalse(m_extender.isHomed(), "still needs homing; position is unknown");
  }

  @Test
  void homingZeroIsNotMistakenForAReboot() {
    // The box was 10 in off when the operator homed it.
    moveTo(Units.inchesToMeters(10));
    m_extender.finishHoming(true);
    // A real SPARK reports the new zero a frame or two later: the old reading lingers first.
    loop(Units.inchesToMeters(10));
    loop(Units.inchesToMeters(10));
    loop(0.0);
    assertFalse(m_extender.isFaulted(), "our own zeroing must not look like a SPARK reboot");
    assertTrue(m_extender.isHomed());

    // After the settle window, a real reset is still caught.
    for (int i = 0; i < 20; i++) {
      loop(0.0);
    }
    moveTo(Units.inchesToMeters(10));
    loop(0.0);
    assertTrue(m_extender.isFaulted(), "a genuine reset is still detected");
  }

  @Test
  void briefCurrentImbalanceDoesNotTrip() {
    // Q2 at NCFAY: one sample of leader 60.7 A / follower 0 A, 0.2 s into auto, latched the
    // extender off for the whole match. A startup surge must not trip it.
    m_leaderSim.setMotorCurrent(70);
    for (int i = 0; i < 5; i++) {   // 0.1 s
      loop(0.0);
    }
    m_leaderSim.setMotorCurrent(0);
    loop(0.0);
    assertFalse(m_extender.isFaulted(), "a 0.1 s surge is not an imbalance");

    // Sustained, it still trips.
    m_leaderSim.setMotorCurrent(70);
    for (int i = 0; i < 25; i++) {  // 0.5 s
      loop(0.0);
    }
    assertTrue(m_extender.isFaulted(), "a sustained imbalance must still trip");
    assertTrue(m_extender.isHomed(), "an imbalance doesn't make the encoders untrustworthy");
  }

  @Test
  void stopsPushingWhenTheBoxCantReachTheTarget() {
    var scheduler = edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance();
    scheduler.schedule(m_extender.extend()); // target: fully out
    scheduler.run();
    // The box hits something 0.5 in short of the target and stops there.
    double out = Units.metersToInches(ExtenderConstants.kExtended);
    double stop = out - 0.5;
    moveTo(Units.inchesToMeters(stop));
    m_leaderEncoder.setVelocity(0);
    m_leaderSim.setAppliedOutput(0.1); // still pushing

    for (int i = 0; i < 10; i++) {   // 0.2 s: a brief pause is fine
      loop(Units.inchesToMeters(stop));
    }
    assertEquals(out, m_extender.getTargetInches(), 1e-6, "don't give up on a brief pause");

    for (int i = 0; i < 30; i++) {   // 0.6 s more: clearly stalled
      loop(Units.inchesToMeters(stop));
    }
    assertEquals(stop, m_extender.getTargetInches(), 0.05, "accept where it stopped; stop pushing");
    assertFalse(m_extender.isFaulted(), "stopping short isn't a fault");
  }

  @Test
  void keepsPushingWhileMovingOrIdle() {
    var scheduler = edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance();
    scheduler.schedule(m_extender.extend());
    scheduler.run();
    // Moving toward the target with output applied: normal motion, not a stall.
    m_leaderSim.setAppliedOutput(0.3);
    m_leaderEncoder.setVelocity(0.2);
    for (int i = 0; i < 40; i++) {
      loop(Units.inchesToMeters(6));
    }
    assertEquals(Units.metersToInches(ExtenderConstants.kExtended), m_extender.getTargetInches(), 1e-6);
    // Not moving but not pushing either (e.g. disabled): nothing to protect.
    m_leaderEncoder.setVelocity(0);
    m_leaderSim.setAppliedOutput(0);
    for (int i = 0; i < 40; i++) {
      loop(Units.inchesToMeters(6));
    }
    assertEquals(Units.metersToInches(ExtenderConstants.kExtended), m_extender.getTargetInches(), 1e-6);
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
