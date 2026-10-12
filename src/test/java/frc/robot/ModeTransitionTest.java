package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Checks the command pattern Robot/RobotContainer use across the FMS sequence
 * (auto enabled -> disabled -> teleop enabled): the drivetrain's default command
 * must come back once the auto command is gone. Uses a stand-in subsystem because
 * the real DriveSubsystem needs motor controllers and a gyro.
 */
public class ModeTransitionTest {
  private CommandScheduler m_scheduler;
  private SubsystemBase m_drive;
  private Command m_defaultDrive;
  private Command m_auto;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    m_scheduler = CommandScheduler.getInstance();
    m_scheduler.cancelAll();
    m_scheduler.unregisterAllSubsystems();
    m_scheduler.getActiveButtonLoop().clear();

    m_drive = new SubsystemBase() {};
    m_defaultDrive = Commands.run(() -> {}, m_drive).withName("DriveCommand");
    m_drive.setDefaultCommand(m_defaultDrive);
    // Like DriveStraight: requires the drivetrain and never finishes on its own.
    m_auto = Commands.run(() -> {}, m_drive).withName("Auto");
  }

  @AfterEach
  void tearDown() {
    m_scheduler.cancelAll();
    m_scheduler.unregisterAllSubsystems();
    DriverStationSim.resetData();
  }

  private void setMode(boolean enabled, boolean autonomous, boolean fms) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.setFmsAttached(fms);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @Test
  void autoTakesOverDrivetrainThenDefaultCommandResumes() {
    setMode(true, true, true);
    m_scheduler.schedule(m_auto);
    m_scheduler.run();
    assertTrue(m_auto.isScheduled());
    assertFalse(m_defaultDrive.isScheduled(), "auto should own the drivetrain");

    // What Robot.teleopInit() does.
    setMode(true, false, true);
    m_auto.cancel();
    assertFalse(m_auto.isScheduled(), "auto must be cancelled immediately");
    m_scheduler.run();
    assertTrue(m_defaultDrive.isScheduled(), "DriveCommand must resume in teleop");
  }

  @Test
  void fmsSequenceAutoDisabledTeleopRestoresDriveCommand() {
    // Auto enabled
    setMode(true, true, true);
    m_scheduler.schedule(m_auto);
    m_scheduler.run();

    // FMS disables the robot between auto and teleop
    setMode(false, false, true);
    m_scheduler.run();
    assertFalse(m_auto.isScheduled(), "a normal command is cancelled when the robot disables");

    // Teleop enabled (teleopInit cancel is then a harmless no-op)
    setMode(true, false, true);
    m_auto.cancel();
    m_scheduler.run();
    assertTrue(m_defaultDrive.isScheduled(), "DriveCommand must be running in teleop");
  }

  @Test
  void repeatedMatchesStillRestoreDriveCommand() {
    for (int match = 0; match < 3; match++) {
      setMode(true, true, true);
      m_scheduler.schedule(m_auto);
      m_scheduler.run();
      assertTrue(m_auto.isScheduled(), "auto should be reschedulable in match " + match);

      setMode(false, false, true);
      m_scheduler.run();

      setMode(true, false, true);
      m_auto.cancel();
      m_scheduler.run();
      assertTrue(m_defaultDrive.isScheduled(), "DriveCommand missing in match " + match);
    }
  }
}
