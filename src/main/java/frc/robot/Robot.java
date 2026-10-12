// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.diagnostics.RobotDiagnostics;

/** Robot lifecycle: mode transitions, the command scheduler, and logging. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @Logged(name = "Robot")
  private final RobotContainer m_robotContainer;

  private final RobotDiagnostics m_diagnostics;

  public Robot() {
    // Everything is recorded to a .wpilog on the roboRIO (or a USB stick): NetworkTables, DS
    // state and joysticks, diagnostics, and @Logged values. Open it with AdvantageScope.
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer();

    // Which code is running, so the drive team can confirm the build before a match.
    String build = "build " + BuildInfo.GIT_SHA + " (" + BuildInfo.GIT_BRANCH
        + ", uncommitted changes: " + BuildInfo.GIT_DIRTY + ", built " + BuildInfo.BUILD_DATE + ")";
    SmartDashboard.putString("Build", build);

    // Created after RobotContainer so every motor controller already exists.
    m_diagnostics = RobotDiagnostics.start(build);
    m_diagnostics.setDriveCommand(m_robotContainer.getDriveCommand());

    // @Logged values go only to the log file, not over the radio.
    Epilogue.configure(config -> config.backend = new FileBackend(DataLogManager.getLog()));
    Epilogue.bind(this);
  }

  /** For simulation tests only. */
  RobotContainer getContainerForTest() {
    return m_robotContainer;
  }

  @Override
  public void robotPeriodic() {
    double loopStart = Timer.getFPGATimestamp();
    m_diagnostics.loopStart(loopStart);
    try {
      CommandScheduler.getInstance().run();
    } catch (RuntimeException | Error e) {
      // Record it in the log, then let it crash as it would anyway. Never swallowed.
      RobotDiagnostics.recordException(e);
      throw e;
    }
    m_diagnostics.periodic(loopStart);
  }

  @Override
  public void disabledInit() {
    RobotDiagnostics.logEvent("DISABLED INIT");
  }

  @Override
  public void autonomousInit() {
    // The robot is placed facing away from the driver station at the start of a match, so make
    // that field-relative "forward". The driver can re-zero any time with the zero-heading button.
    m_robotContainer.zeroDriverHeading();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    RobotDiagnostics.logEvent("AUTO INIT: selected "
        + (m_autonomousCommand != null ? m_autonomousCommand.getName() : "null (nothing selected)"));
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      RobotDiagnostics.logEvent("TELEOP INIT: auto cancel requested, auto still scheduled = "
          + m_autonomousCommand.isScheduled());
    } else {
      RobotDiagnostics.logEvent("TELEOP INIT: no auto command to cancel");
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  // Empty overrides stop WPILib printing "override me" messages to the console.
  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}
}
