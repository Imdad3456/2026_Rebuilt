package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * Motion settings must be physically possible. A profile that asks for more speed than the motor
 * can deliver never decelerates properly and slams the mechanism into its end stop.
 */
public class MechanismLimitsTest {
  private static double extenderFreeSpeed() {
    return NeoMotorConstants.kFreeSpeedRpm * ExtenderConstants.kVelocityFactor;
  }

  @Test
  void extenderFeedforwardIsFiniteAndPositive() {
    double kV = ExtenderConstants.kV.get();
    assertTrue(Double.isFinite(kV) && kV > 0, "kV must be a real positive number, was " + kV);
  }

  @Test
  void extenderCruiseLeavesHeadroom() {
    // Under ~80% of top speed, so the motor has voltage left to follow and decelerate.
    assertTrue(ExtenderConstants.kCruiseVelocity < 0.8 * extenderFreeSpeed(),
        "cruise " + ExtenderConstants.kCruiseVelocity + " m/s vs top speed " + extenderFreeSpeed());
  }

  @Test
  void extenderFeedforwardAtCruiseFitsInVoltageCompensation() {
    double volts = ExtenderConstants.kV.get() * ExtenderConstants.kCruiseVelocity + ExtenderConstants.kS.get();
    assertTrue(volts < GlobalConstants.kMedVoltageCompensation,
        "cruise needs " + volts + " V of feedforward");
  }

  @Test
  void driveFeedforwardAtTeleopMaxFitsInVoltageCompensation() {
    double volts = DriveConstants.kDriveV.get() * DriveConstants.kAdjustedMaxSpeedMbpsTeleOp;
    assertTrue(volts < GlobalConstants.kMedVoltageCompensation, "teleop max needs " + volts + " V");
  }
}
