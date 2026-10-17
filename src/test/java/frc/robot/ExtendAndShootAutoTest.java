package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ExtenderConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.util.hardware.Sparks;
import frc.robot.util.hardware.Talons;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/** The default auto: box out, shoot the preload, stop. No driving. */
public class ExtendAndShootAutoTest {
  private static SparkMax spark(int id) {
    return (SparkMax) Sparks.all().stream().filter(s -> s.getDeviceId() == id).findFirst().orElseThrow();
  }

  private static void loops(Robot robot, int n) {
    for (int i = 0; i < n; i++) {
      SimHooks.stepTiming(0.02);
      robot.autonomousPeriodic();
      robot.robotPeriodic();
    }
  }

  @Test
  @Timeout(60) // fail fast instead of hanging if something waits on a paused clock
  void extendsShootsPreloadThenStops() {
    assertTrue(HAL.initialize(500, 0));
    // Build the robot BEFORE pausing simulated time: Phoenix 6 times its config-apply wait on that
    // clock, so with time paused the launcher's setup would wait forever.
    try (Robot robot = new Robot()) {
      SimHooks.pauseTiming();
      RobotContainer container = robot.getContainerForTest();
      var extender = container.getExtenderForTest();
      var launcher = container.getLauncherForTest();
      SparkMax feeder = spark(CanIdConstants.kFeederMotor);
      var leaderEnc = new SparkRelativeEncoderSim(spark(CanIdConstants.kExtenderMotor1));
      var followerEnc = new SparkRelativeEncoderSim(spark(CanIdConstants.kExtenderMotor2));
      var flywheel = Talons.all().get(0).getSimState();

      Command auto = container.getAutonomousCommand();
      assertEquals("Extend and Shoot", auto.getName(), "must be the default auto");

      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      robot.autonomousInit();
      loops(robot, 1);

      // Box heading out while the flywheel spins up; nothing fed yet.
      assertEquals(Units.metersToInches(ExtenderConstants.kExtended), extender.getTargetInches(), 1e-6);
      assertEquals(AutoConstants.kAutoShotRPM.get(), launcher.getTargetRPM(), 1e-6);
      assertEquals(0.0, feeder.get(), 1e-9, "don't feed before the box is out and at speed");

      // The box travels out (gradually, like the real thing) and the flywheel reaches speed.
      double out = ExtenderConstants.kExtended;
      for (double pos = 0; pos < out; pos += 0.004) {
        leaderEnc.setPosition(pos);
        followerEnc.setPosition(pos);
        loops(robot, 1);
      }
      leaderEnc.setPosition(out);
      followerEnc.setPosition(out);
      flywheel.setRotorVelocity(AutoConstants.kAutoShotRPM.get() / 60.0);
      loops(robot, 15); // at target + flywheel steady
      assertTrue(extender.isAtTarget());
      assertTrue(launcher.isReadyToFeed(), "flywheel at speed");
      assertEquals(FeederConstants.kFeederSpeed.get(), feeder.get(), 1e-9, "shooting the preload");

      // After the preload window everything stops, with the box left out for teleop.
      loops(robot, (int) (AutoConstants.kPreloadShootSeconds / 0.02) + 5);
      assertFalse(auto.isScheduled(), "auto finished");
      assertEquals(0.0, launcher.getTargetRPM(), 1e-6, "flywheel off");
      assertEquals(0.0, feeder.get(), 1e-9, "feeder off");
      assertEquals(Units.metersToInches(out), extender.getTargetInches(), 1e-6, "box stays out");
      assertFalse(extender.isFaulted());
    } finally {
      CommandScheduler.getInstance().cancelAll();
      SimHooks.resumeTiming();
      DriverStationSim.resetData();
    }
  }
}
