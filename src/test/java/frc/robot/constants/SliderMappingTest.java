package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystems.LauncherSubsystem;
import org.junit.jupiter.api.Test;

/** The slider math: -1..1 raw axis to 0..1, and the ranges each slider maps onto. */
public class SliderMappingTest {

  @Test
  void rawAxisMapsToPercent() {
    assertEquals(0.0, OIConstants.sliderToPercent(-1.0, false), 1e-9);
    assertEquals(0.5, OIConstants.sliderToPercent(0.0, false), 1e-9);
    assertEquals(1.0, OIConstants.sliderToPercent(1.0, false), 1e-9);
  }

  @Test
  void invertedSliderFlips() {
    assertEquals(1.0, OIConstants.sliderToPercent(-1.0, true), 1e-9);
    assertEquals(0.0, OIConstants.sliderToPercent(1.0, true), 1e-9);
  }

  @Test
  void outOfRangeReadingsAreClamped() {
    assertEquals(0.0, OIConstants.sliderToPercent(-1.2, false), 1e-9);
    assertEquals(1.0, OIConstants.sliderToPercent(1.2, false), 1e-9);
  }

  @Test
  void slowestSettingsNeverReachZero() {
    // A slider left at the bottom must not make the robot or intake look dead.
    assertTrue(OIConstants.kMinDriveSpeedScale > 0);
    assertTrue(IntakeConstants.kMinSpeed > 0);
  }

  @Test
  void shooterSliderSpansMinToMaxRpm() {
    assertEquals(LauncherConstants.kMinShotRPM.get(), LauncherSubsystem.rpmFromSlider(0.0), 1e-9);
    assertEquals(LauncherConstants.kMaxShotRPM.get(), LauncherSubsystem.rpmFromSlider(1.0), 1e-9);
  }
}
