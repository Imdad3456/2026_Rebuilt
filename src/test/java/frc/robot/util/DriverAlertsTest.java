package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;

import com.revrobotics.sim.SparkRelativeEncoderSim;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CanIdConstants;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.util.hardware.Sparks;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** What the drive team sees in Elastic: the Alerts list and the pop-up notifications. */
public class DriverAlertsTest {
  private static ExtenderSubsystem s_extender;
  private static DriverAlerts s_alerts;
  private static StringArraySubscriber s_errors;
  private static StringArraySubscriber s_warnings;
  private static StringSubscriber s_popup;

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    RoboRioSim.setVInVoltage(12.8);
    setMode(false);
    s_extender = new ExtenderSubsystem();
    s_alerts = new DriverAlerts(s_extender);
    var nt = NetworkTableInstance.getDefault();
    s_errors = nt.getStringArrayTopic("/SmartDashboard/Alerts/errors").subscribe(new String[0]);
    s_warnings = nt.getStringArrayTopic("/SmartDashboard/Alerts/warnings").subscribe(new String[0]);
    s_popup = nt.getStringTopic("/Elastic/RobotNotifications").subscribe("");
  }

  @AfterAll
  static void tearDown() {
    SimHooks.resumeTiming();
    DriverStationSim.resetData();
  }

  private static void setMode(boolean enabled) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private static void update(String auto) {
    s_alerts.update(auto);
    SmartDashboard.updateValues(); // publishes the Alerts group to NetworkTables
  }

  private static boolean contains(String[] list, String text) {
    return Arrays.stream(list).anyMatch(s -> s.contains(text));
  }

  @Test
  void warnsWhenAutoIsDoNothing() {
    update("Do Nothing");
    assertTrue(contains(s_warnings.get(), "Do Nothing"), Arrays.toString(s_warnings.get()));
    update("TestAuto");
    assertFalse(contains(s_warnings.get(), "Do Nothing"));
  }

  @Test
  void warnsAboutLowBatteryOnlyAfterItStaysLow() {
    RoboRioSim.setVInVoltage(11.9);
    update("TestAuto");
    assertFalse(contains(s_warnings.get(), "Battery low"), "a brief dip is ignored");
    SimHooks.stepTiming(2.5);
    update("TestAuto");
    assertTrue(contains(s_warnings.get(), "Battery low"), Arrays.toString(s_warnings.get()));
    assertTrue(s_popup.get().contains("Battery low"), "pop-up sent: " + s_popup.get());
    RoboRioSim.setVInVoltage(12.8);
    update("TestAuto");
    assertFalse(contains(s_warnings.get(), "Battery low"));
  }

  @Test
  void boxFaultShowsAlertAndPopUp() {
    // Pull the leader ahead of the follower gradually until the skew safety trips.
    var leader = new SparkRelativeEncoderSim(spark(CanIdConstants.kExtenderMotor1));
    var follower = new SparkRelativeEncoderSim(spark(CanIdConstants.kExtenderMotor2));
    setMode(true);
    for (double pos = 0; pos <= Units.inchesToMeters(3) && !s_extender.isFaulted(); pos += 0.004) {
      leader.setPosition(pos);
      follower.setPosition(0);
      SimHooks.stepTiming(0.02);
      s_extender.periodic();
    }
    assertTrue(s_extender.isFaulted());
    update("TestAuto");
    assertTrue(contains(s_errors.get(), "Box stopped"), Arrays.toString(s_errors.get()));
    assertTrue(s_popup.get().contains("Box stopped"), "pop-up sent: " + s_popup.get());
    setMode(false);
  }

  private static com.revrobotics.spark.SparkMax spark(int id) {
    return Sparks.all().stream().filter(s -> s.getDeviceId() == id).findFirst().orElseThrow();
  }
}
