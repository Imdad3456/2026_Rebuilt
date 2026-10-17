package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;

import edu.wpi.first.hal.HAL;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ExtenderConstants;
import frc.robot.constants.NeoSwerveModuleConstants;
import frc.robot.util.hardware.Sparks;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Builds the whole robot in simulation, then reads each SPARK's configuration back from the
 * controller. Proves the settings each motor depends on actually reach it (for example that the
 * extender motors run independently, and the steering motors still use their absolute encoders).
 */
public class MotorConfigTest {
  private static Robot s_robot;

  @BeforeAll
  static void build() {
    assertTrue(HAL.initialize(500, 0));
    s_robot = new Robot();
  }

  @AfterAll
  static void close() {
    s_robot.close();
  }

  private static SparkBase spark(int canId) {
    return Sparks.all().stream().filter(s -> s.getDeviceId() == canId).findFirst()
        .orElseThrow(() -> new AssertionError("no SPARK created for CAN " + canId));
  }

  private static SparkBaseConfigAccessor config(int canId) {
    return Sparks.configAccessor(spark(canId));
  }

  @Test
  void everySparkIgnoresLimitInputs() {
    assertFalse(Sparks.all().isEmpty());
    for (SparkBase spark : Sparks.all()) {
      var limits = Sparks.configAccessor(spark).limitSwitch;
      assertEquals(Behavior.kKeepMovingMotor, limits.getForwardLimitSwitchTriggerBehavior(), "CAN " + spark.getDeviceId());
      assertEquals(Behavior.kKeepMovingMotor, limits.getReverseLimitSwitchTriggerBehavior(), "CAN " + spark.getDeviceId());
    }
  }

  @Test
  void steeringMotorsUseTheAbsoluteEncoder() {
    for (int id : new int[] {CanIdConstants.kFrontLeftTurningCanId, CanIdConstants.kFrontRightTurningCanId,
        CanIdConstants.kRearLeftTurningCanId, CanIdConstants.kRearRightTurningCanId}) {
      SparkBaseConfigAccessor c = config(id);
      assertEquals(FeedbackSensor.kAbsoluteEncoder, c.closedLoop.getFeedbackSensor(), "CAN " + id);
      assertTrue(c.absoluteEncoder.getInverted(), "CAN " + id);
      assertEquals(2 * Math.PI, c.absoluteEncoder.getPositionConversionFactor(), 1e-4, "CAN " + id);
      assertTrue(c.closedLoop.getPositionWrappingEnabled(), "CAN " + id);
      assertEquals(DriveConstants.kTurnP.get(), c.closedLoop.getP(), 1e-6, "CAN " + id);
      assertEquals(IdleMode.kBrake, c.getIdleMode(), "CAN " + id);
      assertEquals(20, c.getSmartCurrentLimit(), "CAN " + id);
    }
  }

  @Test
  void driveMotorsHaveFeedforwardAndLimits() {
    for (int id : new int[] {CanIdConstants.kFrontLeftDrivingCanId, CanIdConstants.kFrontRightDrivingCanId,
        CanIdConstants.kRearLeftDrivingCanId, CanIdConstants.kRearRightDrivingCanId}) {
      SparkBaseConfigAccessor c = config(id);
      assertEquals(FeedbackSensor.kPrimaryEncoder, c.closedLoop.getFeedbackSensor(), "CAN " + id);
      assertEquals(NeoSwerveModuleConstants.kDriveEncoderPositionFactor, c.encoder.getPositionConversionFactor(), 1e-6,
          "CAN " + id);
      assertEquals(DriveConstants.kDriveV.get(), c.closedLoop.feedForward.getkV(), 1e-4, "CAN " + id);
      assertEquals(IdleMode.kBrake, c.getIdleMode(), "CAN " + id);
      assertEquals(50, c.getSmartCurrentLimit(), "CAN " + id);
    }
  }

  @Test
  void extenderMotorsAreIndependent() {
    // No follower mode: each motor runs its own closed loop with the same limits.
    for (int id : new int[] {CanIdConstants.kExtenderMotor1, CanIdConstants.kExtenderMotor2}) {
      SparkBaseConfigAccessor c = config(id);
      assertEquals(0, c.getFollowerModeLeaderId(), "CAN " + id + " must not follow");
      assertEquals(IdleMode.kBrake, c.getIdleMode(), "CAN " + id);
      assertEquals(ExtenderConstants.kCurrentLimit, c.getSmartCurrentLimit(), "CAN " + id);
      assertEquals(ExtenderConstants.kCruiseVelocity, c.closedLoop.maxMotion.getCruiseVelocity(), 1e-4, "CAN " + id);
      assertEquals(ExtenderConstants.kPositionFactor, c.encoder.getPositionConversionFactor(), 1e-9, "CAN " + id);
      assertTrue(c.softLimit.getForwardSoftLimitEnabled(), "CAN " + id);
      assertTrue(c.softLimit.getReverseSoftLimitEnabled(), "CAN " + id);
      assertEquals(ExtenderConstants.kExtended + ExtenderConstants.kSoftLimitMargin,
          c.softLimit.getForwardSoftLimit(), 1e-4, "CAN " + id);
      assertEquals(ExtenderConstants.kRetracted - ExtenderConstants.kSoftLimitMargin,
          c.softLimit.getReverseSoftLimit(), 1e-4, "CAN " + id);
    }
    // Mounted opposite each other.
    assertFalse(config(CanIdConstants.kExtenderMotor1).getInverted());
    assertTrue(config(CanIdConstants.kExtenderMotor2).getInverted());
    // Homing waits for both motors to pass this current, so it must be reachable under the limit.
    assertTrue(ExtenderConstants.kHomingStallCurrent < ExtenderConstants.kCurrentLimit);
  }

  @Test
  void intakeIsASparkFlex() {
    // CAN 10 is a SPARK Flex; driving it as a SPARK MAX logged "not a SPARK MAX" at every boot.
    assertInstanceOf(SparkFlex.class, spark(CanIdConstants.kIntakeMotor));
  }

  @Test
  void intakeAndFeederDirections() {
    assertTrue(config(CanIdConstants.kIntakeMotor).getInverted(), "intake was reversed on the robot");
    assertTrue(config(CanIdConstants.kFeederMotor).getInverted());
    assertEquals(IdleMode.kBrake, config(CanIdConstants.kIntakeMotor).getIdleMode());
    assertEquals(IdleMode.kBrake, config(CanIdConstants.kFeederMotor).getIdleMode());
  }
}
