package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Motor configs must never be rewritten from the dashboard while on the FMS. */
public class TunableNumberTest {

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
  }

  @AfterEach
  void tearDown() {
    DriverStationSim.resetData();
  }

  private void setFms(boolean attached) {
    DriverStationSim.setFmsAttached(attached);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @Test
  void defaultIsVisibleOnTheDashboard() {
    new TunableNumber("Test/Visible", 4.2);
    // What a dashboard sees: the published value, not just a topic with nothing in it.
    var sub = NetworkTableInstance.getDefault().getDoubleTopic("Tuning/Test/Visible").subscribe(-1.0);
    assertEquals(4.2, sub.get(), 1e-9);
    sub.close();
  }

  @Test
  void reportsChangesOffTheField() {
    TunableNumber number = new TunableNumber("Test/OffField", 1.0);
    setFms(false);
    assertFalse(number.hasChanged());

    try (DoublePublisher pub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Tuning/Test/OffField").publish()) {
      pub.set(2.0);
      assertTrue(number.hasChanged(), "a dashboard edit should be seen");
      assertFalse(number.hasChanged(), "only once per edit");
    }
  }

  @Test
  void ignoresChangesWhileOnFmsThenAppliesAfter() {
    TunableNumber number = new TunableNumber("Test/OnField", 1.0);
    setFms(true);

    try (DoublePublisher pub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Tuning/Test/OnField").publish()) {
      pub.set(3.0);
      assertFalse(number.hasChanged(), "no config writes while on the FMS");

      setFms(false);
      assertTrue(number.hasChanged(), "the edit is picked up after the match");
    }
  }
}
