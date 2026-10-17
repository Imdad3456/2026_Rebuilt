package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;

/**
 * Runs the robot through the real TimedRobot loop (startCompetition), the same code path the
 * roboRIO uses, including Epilogue logging, PathPlanner and the dashboard/alerts updates:
 * disabled with TestAuto selected, autonomous, the field's disabled gap, then teleop.
 * Fails if anything throws in the robot thread.
 */
public class RealLoopTest {
  private static void setMode(boolean enabled, boolean auto) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.setAutonomous(auto);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  @Test
  void fullMatchSequenceInTheRealLoop() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    setMode(false, false);
    AtomicReference<Throwable> crash = new AtomicReference<>();

    try (Robot robot = new Robot();
        StringPublisher autoPick = NetworkTableInstance.getDefault()
            .getStringTopic("/SmartDashboard/Auto choices/selected").publish()) {
      Thread loop = new Thread(robot::startCompetition, "robot-loop");
      loop.setUncaughtExceptionHandler((t, e) -> crash.set(e));
      loop.setDaemon(true);
      loop.start();

      Thread.sleep(1000);               // disabled, pre-match
      autoPick.set("TestAuto");         // the drive team picks the auto
      Thread.sleep(500);
      setMode(true, true);              // autonomous
      Thread.sleep(3500);
      setMode(false, false);            // the field's disabled gap
      Thread.sleep(1000);
      setMode(true, false);             // teleop
      Thread.sleep(2000);
      assertTrue(robot.getContainerForTest().getDriveCommand().isScheduled(),
          "DriveCommand must be running in teleop");

      robot.endCompetition();
      loop.join(2000);
      assertNull(crash.get(), () -> "robot loop crashed: " + crash.get());
    } finally {
      CommandScheduler.getInstance().cancelAll();
      DriverStationSim.resetData();
    }
  }
}
