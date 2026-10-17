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

/** Shaking the box while shooting: in a little, back out, repeat; back where it was afterwards. */
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
    double in = Units.metersToInches(ExtenderConstants.kExtended - ExtenderConstants.kAgitateDepth);
    assertEquals(out, m_extender.getTargetInches(), 1e-6);

    Command agitate = m_extender.agitate();
    run(agitate);
    assertEquals(in, m_extender.getTargetInches(), 1e-6, "first stroke pulls in");

    double stroke = ExtenderConstants.kAgitateStrokeSeconds;
    advance(stroke);
    assertEquals(out, m_extender.getTargetInches(), 1e-6, "then pushes back out");
    advance(stroke);
    assertEquals(in, m_extender.getTargetInches(), 1e-6, "and repeats");

    agitate.cancel(); // shoot button released
    assertEquals(out, m_extender.getTargetInches(), 1e-6, "back where it was");
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
