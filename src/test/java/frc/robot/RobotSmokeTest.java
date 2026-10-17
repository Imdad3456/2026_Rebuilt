package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
    try (Robot robot = new Robot()) {
      robot.robotInit();
      for (String name : new String[] {"Intake", "IntakeStop", "SpinUp", "Shoot", "ExtenderOut",
          "ExtenderIn", "autoLaunch", "autoIntake"}) {
        assertTrue(NamedCommands.hasCommand(name), "PathPlanner named command missing: " + name);
      }
      // settings.json is deployed, so PathPlanner must be configured and the March auto must load.
      assertTrue(AutoBuilder.isConfigured(), "PathPlanner not configured; is settings.json valid?");
      assertNotNull(AutoBuilder.buildAuto("TestAuto"));
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
    } finally {
      CommandScheduler.getInstance().cancelAll();
    }
  }
}
