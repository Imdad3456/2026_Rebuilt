package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;

/** Builds the real Robot in simulation and runs it through FMS-style mode changes. */
public class RobotSmokeTest {
  private static void setMode(boolean enabled, boolean auto) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.setAutonomous(auto);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @Test
  void realRobotSurvivesAutoDisabledTeleopCycles() {
    assertTrue(HAL.initialize(500, 0));
    Robot robot = new Robot();
    robot.robotInit();
    for (int match = 0; match < 3; match++) {
      setMode(true, true);
      robot.autonomousInit();
      for (int i = 0; i < 50; i++) { robot.autonomousPeriodic(); robot.robotPeriodic(); }

      setMode(false, false);
      robot.disabledInit();
      for (int i = 0; i < 20; i++) { robot.disabledPeriodic(); robot.robotPeriodic(); }

      setMode(true, false);
      robot.teleopInit();
      for (int i = 0; i < 50; i++) { robot.teleopPeriodic(); robot.robotPeriodic(); }
      assertTrue(robot.getContainerForTest().getDriveCommand().isScheduled(),
          "DriveCommand not running in teleop, match " + match);
    }
    CommandScheduler.getInstance().cancelAll();
  }
}
