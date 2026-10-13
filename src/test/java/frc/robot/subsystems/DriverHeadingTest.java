package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Field-relative "forward" must be "away from our driver station" even when an auto starts with
 * the robot at an angle (e.g. AutoShootToClimb starts at -27.6 degrees).
 */
public class DriverHeadingTest {
  // One instance: REVLib allows one SparkMax object per CAN ID.
  private static DriveSubsystem m_drive;

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
    m_drive = new DriveSubsystem();
  }

  private static void setAlliance(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @Test
  void blueAutoStartingAtAnAngle() {
    setAlliance(AllianceStationID.Blue1);
    m_drive.resetOdometry(new Pose2d(2, 7, Rotation2d.fromDegrees(-27.57)));
    // Blue drivers look down +X, so a field heading of -27.57 is -27.57 from their forward.
    assertEquals(-27.57, m_drive.getDriverHeadingDegrees(), 1e-6);
  }

  @Test
  void redAutoStartingAtAnAngle() {
    setAlliance(AllianceStationID.Red1);
    // The same start mirrored for red: facing 180 - 27.57 degrees on the field.
    m_drive.resetOdometry(new Pose2d(14, 1, Rotation2d.fromDegrees(180 - 27.57)));
    assertEquals(-27.57, m_drive.getDriverHeadingDegrees(), 1e-6);
  }

  @Test
  void redRobotFacingAwayFromDriversIsForward() {
    setAlliance(AllianceStationID.Red2);
    m_drive.resetOdometry(new Pose2d(14, 4, Rotation2d.k180deg));
    assertEquals(0.0, m_drive.getDriverHeadingDegrees(), 1e-6);
  }

  @Test
  void zeroButtonStillWorks() {
    setAlliance(AllianceStationID.Blue1);
    m_drive.resetOdometry(new Pose2d(2, 7, Rotation2d.fromDegrees(40)));
    m_drive.zeroDriverHeading();
    assertEquals(0.0, m_drive.getDriverHeadingDegrees(), 1e-6);
  }
}
