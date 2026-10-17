package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BuildInfo;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.diagnostics.RobotDiagnostics;

/**
 * Plain-language warnings for the drive team.
 *
 * <ul>
 *   <li>The dashboard's Alerts widget lists every problem that is true right now (WPILib
 *       {@link Alert}s, published under /SmartDashboard/Alerts).</li>
 *   <li>When a problem first appears, Elastic also shows a pop-up on the driver station laptop.
 *       Pop-ups fire only on the change, never repeatedly, so they can't flood the screen.</li>
 * </ul>
 *
 * <p>Display only: nothing here changes what the robot does.
 */
public final class DriverAlerts {
  /** Resting voltage below which the battery should be swapped before a match. */
  private static final double kLowBatteryVolts = 12.3;

  private final Alert m_noAuto = new Alert("Auto is set to Do Nothing - pick an auto", AlertType.kWarning);
  private final Alert m_lowBattery = new Alert("", AlertType.kWarning);
  private final Alert m_boxFault = new Alert("", AlertType.kError);
  private final Alert m_boxNotHomed = new Alert("Box not homed - press home (8)", AlertType.kWarning);
  private final Alert m_motors = new Alert("", AlertType.kError);
  private final Alert m_controllers = new Alert("", AlertType.kError);
  private final Alert m_brownout = new Alert("", AlertType.kWarning);
  private final Alert m_build = new Alert(
      "Build " + BuildInfo.GIT_SHA + " has uncommitted changes (not on GitHub)", AlertType.kInfo);

  /** Battery must read low for a couple of seconds (ignores brief dips while disabled). */
  private final Debouncer m_lowBatteryDebounce = new Debouncer(2.0);

  private boolean m_prevBoxFault = false;
  private boolean m_prevMotorsBad = false;
  private boolean m_prevControllersBad = false;
  private boolean m_prevBrownedOut = false;
  private boolean m_prevLowBattery = false;
  private boolean m_prevEnabled = false;

  private final ExtenderSubsystem m_extender;

  public DriverAlerts(ExtenderSubsystem extender) {
    m_extender = extender;
    m_build.set("true".equals(BuildInfo.GIT_DIRTY));
  }

  /** Call periodically (10 Hz is plenty) with the name of the selected auto. */
  public void update(String selectedAutoName) {
    boolean enabled = DriverStation.isEnabled();
    boolean disabled = !enabled;
    double volts = RobotController.getBatteryVoltage();

    // A fresh enable clears last period's brownout note.
    if (enabled && !m_prevEnabled) {
      m_brownout.set(false);
    }
    m_prevEnabled = enabled;

    m_noAuto.set(disabled && DriverStation.isDSAttached() && "Do Nothing".equals(selectedAutoName));

    boolean lowBattery = m_lowBatteryDebounce.calculate(disabled && volts < kLowBatteryVolts);
    m_lowBattery.setText(String.format("Battery low (%.1f V) - swap before queuing", volts));
    m_lowBattery.set(lowBattery);
    if (lowBattery && !m_prevLowBattery) {
      notify(NotificationLevel.WARNING, "Battery low",
          String.format("%.1f V while disabled. Swap the battery before the next match.", volts));
    }
    m_prevLowBattery = lowBattery;

    boolean boxFault = m_extender.isFaulted();
    m_boxFault.setText("Box stopped: " + m_extender.getStatus() + " - press clear (13) or home (8)");
    m_boxFault.set(boxFault);
    m_boxNotHomed.set(!boxFault && !m_extender.isHomed());
    if (boxFault && !m_prevBoxFault) {
      notify(NotificationLevel.ERROR, "Box stopped", m_extender.getStatus()
          + ". Press clear faults (13), or home (8) if it says to.");
    }
    m_prevBoxFault = boxFault;

    boolean motorsBad = !RobotDiagnostics.allMotorsOk();
    m_motors.setText("Motor problem: " + RobotDiagnostics.healthReport());
    m_motors.set(motorsBad);
    if (motorsBad && !m_prevMotorsBad) {
      notify(NotificationLevel.ERROR, "Motor problem", RobotDiagnostics.healthReport());
    }
    m_prevMotorsBad = motorsBad;

    boolean controllersBad = !RobotDiagnostics.controllersOk();
    m_controllers.setText(RobotDiagnostics.controllerStatus());
    m_controllers.set(controllersBad);
    if (controllersBad && !m_prevControllersBad) {
      notify(NotificationLevel.ERROR, "Controller problem", RobotDiagnostics.controllerStatus());
    }
    m_prevControllersBad = controllersBad;

    boolean brownedOut = RobotController.isBrownedOut();
    if (brownedOut && !m_prevBrownedOut) {
      String when = DriverStation.getMatchTime() >= 0
          ? String.format(" with %.0f s left", DriverStation.getMatchTime()) : "";
      m_brownout.setText(String.format("Brownout%s (battery %.1f V) - outputs were cut", when, volts));
      m_brownout.set(true);
      notify(NotificationLevel.WARNING, "Brownout",
          String.format("Battery dipped to %.1f V and the robot cut its outputs.", volts));
    }
    m_prevBrownedOut = brownedOut;
  }

  private static void notify(NotificationLevel level, String title, String description) {
    Elastic.sendNotification(new Notification(level, title, description).withDisplaySeconds(6));
  }
}
