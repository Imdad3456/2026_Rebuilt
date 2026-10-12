// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.diagnostics.RobotDiagnostics;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the Main.java file in the project.
 *
 * This class is the main robot class and is responsible for managing the robot's lifecycle,
 * including initialization, periodic updates for different robot modes (Disabled, Autonomous,
 * Teleoperated, Test, Simulation), and scheduling commands.
 */
public class Robot extends TimedRobot {
  /** The command that will be run during the autonomous period. */
  private Command m_autonomousCommand;

  /** The RobotContainer object, which handles all button bindings and autonomous command selection. */
  private final RobotContainer m_robotContainer;

  /** Writes robot health (comms, power, CAN, joysticks, loop timing) to the .wpilog. */
  private final RobotDiagnostics m_diagnostics;

  /**
   * Constructs the Robot class. This function is run when the robot is first started up and
   * should be used for any initialization code.
   */
  public Robot() {
    // Record NetworkTables + Driver Station data (brownouts, comms drops, joystick
    // inputs) to a .wpilog on the roboRIO/USB stick. This is how you see what went
    // wrong on the field, since these problems don't show up tethered in the pits.
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Which code is running. Shown on the dashboard so the drive team can confirm the
    // deployed build before a match. (At the event we couldn't tell which commit ran.)
    String build = "build " + BuildInfo.GIT_SHA + " (" + BuildInfo.GIT_BRANCH
        + ", uncommitted changes: " + BuildInfo.GIT_DIRTY + ", built " + BuildInfo.BUILD_DATE + ")";
    SmartDashboard.putString("Build", build);

    // Created after RobotContainer so every motor controller already exists.
    m_diagnostics = RobotDiagnostics.start(build);
    m_diagnostics.setDriveCommand(m_robotContainer.getDriveCommand());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  /** For simulation tests only. */
  RobotContainer getContainerForTest() {
    return m_robotContainer;
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    double loopStart = Timer.getFPGATimestamp();
    m_diagnostics.loopStart(loopStart);
    try {
      CommandScheduler.getInstance().run();
    } catch (RuntimeException | Error e) {
      // Record it in the log, then let it crash exactly as before. We never swallow it.
      RobotDiagnostics.recordException(e);
      throw e;
    }
    m_diagnostics.periodic(loopStart);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * Any initialization specific to the disabled state should be performed here.
   */
  @Override
  public void disabledInit() {
    RobotDiagnostics.logEvent("DISABLED INIT");
  }

  /**
   * This function is called periodically while the robot is in Disabled mode.
   * Use this for tasks that need to run continuously during disabled.
   */
  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    RobotDiagnostics.logEvent("AUTO INIT: selected "
        + (m_autonomousCommand != null ? m_autonomousCommand.getName() : "null (nothing selected)"));

    // Schedule the autonomous command (if one is selected).
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /**
   * This function is called periodically during autonomous mode.
   * Use this for tasks that need to run continuously during autonomous.
   */
  @Override
  public void autonomousPeriodic() {}

  /**
   * This function is called once each time the robot enters Teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      RobotDiagnostics.logEvent("TELEOP INIT: auto cancel requested, auto still scheduled = "
          + m_autonomousCommand.isScheduled());
    } else {
      RobotDiagnostics.logEvent("TELEOP INIT: no auto command to cancel");
    }
  }

  /**
   * This function is called periodically during operator control mode (teleop).
   * Use this for tasks that need to run continuously during teleop.
   */
  @Override
  public void teleopPeriodic() {}

  /**
   * This function is called once each time the robot enters Test mode.
   * It cancels all running commands to ensure a clean slate for testing.
   */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   * Use this for tasks that need to run continuously during testing.
   */
  @Override
  public void testPeriodic() {}

  /**
   * This function is called once when the robot is first started up in simulation.
   * Any simulation-specific initialization code should be placed here.
   */
  @Override
  public void simulationInit() {}

  /**
   * This function is called periodically whilst in simulation.
   * Use this for tasks that need to run continuously during simulation.
   */
  @Override
  public void simulationPeriodic() {}
}
