package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.ExtenderConstants;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Shooting: box to the launch position, shake in and back, repeat; back where it was afterwards. */
public class ExtenderAgitateTest {
  // One instance: REVLib allows one SparkMax object per CAN ID.
  private static ExtenderSubsystem m_extender;
  private static final CommandScheduler s_scheduler = CommandScheduler.getInstance();

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    m_extender = new ExtenderSubsystem();
  }

  @AfterAll
  static void tearDown() {
    s_scheduler.cancelAll();
    SimHooks.resumeTiming();
  }

  private static void run(Command command) {
    s_scheduler.schedule(command);
    s_scheduler.run();
  }

  private static void advance(double seconds) {
    SimHooks.stepTiming(seconds);
    s_scheduler.run();
  }

  @Test
  void shakesWhenOutAndReturns() {
    run(m_extender.extend());
    double out = Units.metersToInches(ExtenderConstants.kExtended);
    double launch = Units.metersToInches(ExtenderConstants.kLaunchPosition);
    double in = Units.metersToInches(ExtenderConstants.kLaunchPosition - ExtenderConstants.kAgitateDepth);
    assertEquals(out, m_extender.getTargetInches(), 1e-6);

    Command agitate = m_extender.agitate();
    run(agitate);
    assertEquals(launch, m_extender.getTargetInches(), 1e-6, "first goes to the launch position");

    double stroke = ExtenderConstants.kAgitateStrokeSeconds;
    advance(stroke);
    assertEquals(in, m_extender.getTargetInches(), 1e-6, "then pulls in");
    advance(stroke);
    assertEquals(launch, m_extender.getTargetInches(), 1e-6, "and back out to launch, repeating");

    agitate.cancel(); // shoot button released
    assertEquals(out, m_extender.getTargetInches(), 1e-6, "back where it was");
  }

  @Test
  void manualModeMovesWithinTravel() {
    run(m_extender.retract());
    double[] stick = {1.0}; // full out
    Command manual = m_extender.manual(() -> stick[0]);
    run(manual);
    advance(1.0);
    assertEquals(Units.metersToInches(ExtenderConstants.kManualSpeed * 1.0), m_extender.getTargetInches(), 0.3,
        "1 s at full stick moves kManualSpeed meters");
    advance(5.0);
    assertEquals(Units.metersToInches(ExtenderConstants.kExtended), m_extender.getTargetInches(), 1e-6,
        "never past full extension");
    stick[0] = -1.0;
    advance(10.0);
    assertEquals(Units.metersToInches(ExtenderConstants.kRetracted), m_extender.getTargetInches(), 1e-6,
        "never past retracted");
    stick[0] = 0.05; // inside the deadband
    advance(1.0);
    assertEquals(Units.metersToInches(ExtenderConstants.kRetracted), m_extender.getTargetInches(), 1e-6);
    manual.cancel();
  }

  @Test
  void doesNothingWhenIn() {
    run(m_extender.retract());
    Command agitate = m_extender.agitate();
    run(agitate);
    for (int i = 0; i < 4; i++) {
      advance(ExtenderConstants.kAgitateStrokeSeconds);
      assertEquals(Units.metersToInches(ExtenderConstants.kRetracted), m_extender.getTargetInches(), 1e-6,
          "must not jab a retracted box into the hard stop");
    }
    agitate.cancel();
  }
}
