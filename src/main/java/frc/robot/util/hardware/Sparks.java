package frc.robot.util.hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The one place SPARK MAX controllers are created and reconfigured.
 *
 * <p>Replaces the old MotorControllerWrapper / RealSparkMax / MockSparkMax / HardwareFactory
 * layer, which hid the REV API and caused real bugs (steering motors given the drive config,
 * the absolute-encoder flag ignored, a follower following itself). Subsystems now use
 * {@link SparkMax} directly. REVLib simulates SPARKs on its own in simulation.
 */
public final class Sparks {
  private static final List<SparkMax> s_all = new ArrayList<>();

  private Sparks() {}

  /**
   * Create a brushless SPARK MAX and apply its full config. The config is persisted to flash
   * so a controller that browns out and reboots mid-match comes back configured.
   */
  public static SparkMax create(int canId, SparkMaxConfig config) {
    SparkMax spark = new SparkMax(canId, MotorType.kBrushless);
    // No mechanism on this robot uses hardware limit switches, so never let the limit inputs stop
    // a motor. On the swerve modules the encoder shares the data port with those inputs; a noisy
    // "limit" there can block a steering motor in one direction (seen as red/white LED flicker
    // even while disabled). Set first so the mechanism's own config can still override it.
    SparkMaxConfig full = new SparkMaxConfig();
    full.limitSwitch
        .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
        .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
    full.apply(config);
    check(spark, "boot configure",
        spark.configure(full, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    s_all.add(spark);
    return spark;
  }

  /** Apply a partial config at runtime (RAM only, other settings untouched). */
  public static void apply(SparkMax spark, SparkMaxConfig config, String what) {
    check(spark, what,
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  /** Push new closed-loop gains at runtime. Does not touch the feedback sensor. */
  public static void setGains(SparkMax spark, double p, double i, double d, double kV, double kS) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(p, i, d).feedForward.kV(kV).kS(kS);
    apply(spark, config, "gain update");
  }

  /** Every SPARK created so far (read-only), for fault monitoring. */
  public static List<SparkMax> all() {
    return Collections.unmodifiableList(s_all);
  }

  /**
   * Log a failed configure() once. A missing or duplicate CAN ID or a timeout here means the
   * motor runs on whatever config is in its flash, not ours.
   */
  private static void check(SparkMax spark, String what, REVLibError result) {
    if (result != REVLibError.kOk) {
      DataLogManager.log("[MOTOR] CAN " + spark.getDeviceId() + " " + what + " FAILED: " + result);
    }
  }
}
