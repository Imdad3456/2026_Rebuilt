package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

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
 * extender follower still follows, and the steering motors still use their absolute encoders).
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

  private static SparkMaxConfigAccessor config(int canId) {
    SparkMax spark = Sparks.all().stream().filter(s -> s.getDeviceId() == canId).findFirst()
        .orElseThrow(() -> new AssertionError("no SPARK created for CAN " + canId));
    return spark.configAccessor;
  }

  @Test
  void everySparkIgnoresLimitInputs() {
    assertFalse(Sparks.all().isEmpty());
    for (SparkMax spark : Sparks.all()) {
      var limits = spark.configAccessor.limitSwitch;
      assertEquals(Behavior.kKeepMovingMotor, limits.getForwardLimitSwitchTriggerBehavior(), "CAN " + spark.getDeviceId());
      assertEquals(Behavior.kKeepMovingMotor, limits.getReverseLimitSwitchTriggerBehavior(), "CAN " + spark.getDeviceId());
    }
  }

  @Test
  void steeringMotorsUseTheAbsoluteEncoder() {
    for (int id : new int[] {CanIdConstants.kFrontLeftTurningCanId, CanIdConstants.kFrontRightTurningCanId,
        CanIdConstants.kRearLeftTurningCanId, CanIdConstants.kRearRightTurningCanId}) {
      SparkMaxConfigAccessor c = config(id);
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
      SparkMaxConfigAccessor c = config(id);
      assertEquals(FeedbackSensor.kPrimaryEncoder, c.closedLoop.getFeedbackSensor(), "CAN " + id);
      assertEquals(NeoSwerveModuleConstants.kDriveEncoderPositionFactor, c.encoder.getPositionConversionFactor(), 1e-6,
          "CAN " + id);
      assertEquals(DriveConstants.kDriveV.get(), c.closedLoop.feedForward.getkV(), 1e-4, "CAN " + id);
      assertEquals(IdleMode.kBrake, c.getIdleMode(), "CAN " + id);
      assertEquals(50, c.getSmartCurrentLimit(), "CAN " + id);
    }
  }

  @Test
  void extenderFollowerStillFollowsTheLeader() {
    SparkMaxConfigAccessor follower = config(CanIdConstants.kExtenderMotor2);
    assertEquals(CanIdConstants.kExtenderMotor1, follower.getFollowerModeLeaderId());
    assertTrue(follower.getFollowerModeInverted());
  }

  @Test
  void extenderLeaderKeepsItsSafetyLimits() {
    SparkMaxConfigAccessor leader = config(CanIdConstants.kExtenderMotor1);
    assertFalse(leader.getInverted());
    assertEquals(IdleMode.kBrake, leader.getIdleMode());
    assertEquals(ExtenderConstants.kCurrentLimit, leader.getSmartCurrentLimit());
    assertEquals(ExtenderConstants.kCruiseVelocity, leader.closedLoop.maxMotion.getCruiseVelocity(), 1e-4);
    assertTrue(leader.softLimit.getForwardSoftLimitEnabled());
    assertTrue(leader.softLimit.getReverseSoftLimitEnabled());
    assertEquals(ExtenderConstants.kExtended + ExtenderConstants.kSoftLimitMargin,
        leader.softLimit.getForwardSoftLimit(), 1e-4);
    assertEquals(ExtenderConstants.kRetracted - ExtenderConstants.kSoftLimitMargin,
        leader.softLimit.getReverseSoftLimit(), 1e-4);
  }

  @Test
  void intakeAndFeederDirections() {
    assertTrue(config(CanIdConstants.kIntakeMotor).getInverted(), "intake was reversed on the robot");
    assertTrue(config(CanIdConstants.kFeederMotor).getInverted());
    assertEquals(IdleMode.kBrake, config(CanIdConstants.kIntakeMotor).getIdleMode());
    assertEquals(IdleMode.kBrake, config(CanIdConstants.kFeederMotor).getIdleMode());
  }
}
