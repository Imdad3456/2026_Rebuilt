package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.util.hardware.Sparks;
import org.junit.jupiter.api.Test;

/** While shooting, the feeder runs only when the launcher says it's ready, and pauses otherwise. */
public class FeederGatingTest {
  private boolean m_ready = false;

  @Test
  void feederPausesWhileFlywheelRecovers() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    FeederSubsystem feeder = new FeederSubsystem();
    var spark = Sparks.all().stream()
        .filter(s -> s.getDeviceId() == CanIdConstants.kFeederMotor).findFirst().orElseThrow();
    Command cmd = feeder.feedWhile(() -> m_ready);
    CommandScheduler.getInstance().schedule(cmd);

    m_ready = false; // spinning up
    CommandScheduler.getInstance().run();
    assertEquals(0.0, spark.get(), 1e-9, "must not feed before the flywheel is ready");

    m_ready = true; // at speed
    CommandScheduler.getInstance().run();
    assertEquals(FeederConstants.kFeederSpeed.get(), spark.get(), 1e-9);

    m_ready = false; // a ball just left and the RPM dipped
    CommandScheduler.getInstance().run();
    assertEquals(0.0, spark.get(), 1e-9, "must pause while the flywheel recovers");

    m_ready = true; // recovered
    CommandScheduler.getInstance().run();
    assertEquals(FeederConstants.kFeederSpeed.get(), spark.get(), 1e-9);

    cmd.cancel();
    assertEquals(0.0, spark.get(), 1e-9, "stops when the button is released");
  }
}
