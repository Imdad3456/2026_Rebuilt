package frc.robot.util.hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The one place SPARK MAX and SPARK Flex controllers are created and reconfigured.
 *
 * <p>Replaces the old MotorControllerWrapper / RealSparkMax / MockSparkMax / HardwareFactory
 * layer, which hid the REV API and caused real bugs (steering motors given the drive config,
 * the absolute-encoder flag ignored, a follower following itself). Subsystems now use
 * {@link SparkMax} / {@link SparkFlex} directly. REVLib simulates SPARKs on its own in simulation.
 */
public final class Sparks {
  private static final List<SparkBase> s_all = new ArrayList<>();

  /**
   * More fault/warning flags than this at once never happens on a working SPARK; it is what a
   * read from a SPARK that has dropped off the CAN bus looks like.
   */
  private static final int kGarbageFlagCount = 6;

  private Sparks() {}

  /**
   * Create a brushless SPARK MAX and apply its full config. The config is persisted to flash
   * so a controller that browns out and reboots mid-match comes back configured.
   */
  public static SparkMax create(int canId, SparkMaxConfig config) {
    SparkMaxConfig full = new SparkMaxConfig();
    full.apply(ignoreLimitInputs());
    full.apply(config);
    return register(new SparkMax(canId, MotorType.kBrushless), full);
  }

  /** Same as {@link #create(int, SparkMaxConfig)} for a SPARK Flex. */
  public static SparkFlex createFlex(int canId, SparkFlexConfig config) {
    SparkFlexConfig full = new SparkFlexConfig();
    full.apply(ignoreLimitInputs());
    full.apply(config);
    return register(new SparkFlex(canId, MotorType.kBrushless), full);
  }

  /**
   * No mechanism on this robot uses hardware limit switches, so the limit inputs must never stop
   * a motor. REV's default already ignores them (MotorConfigTest confirms); this just makes it
   * explicit. Applied first so a mechanism's own config could still override it.
   */
  private static LimitSwitchConfig ignoreLimitInputs() {
    return new LimitSwitchConfig()
        .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
        .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
  }

  private static <T extends SparkBase> T register(T spark, SparkBaseConfig full) {
    check(spark, "boot configure",
        spark.configure(full, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    s_all.add(spark);
    return spark;
  }

  /** Apply a partial config at runtime (RAM only, other settings untouched). Waits for the SPARK. */
  public static void apply(SparkBase spark, SparkBaseConfig config, String what) {
    check(spark, what,
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  /**
   * Apply a partial config at runtime without waiting for the SPARK to answer. For configs sent
   * while enabled: a blocking configure() to a SPARK that isn't responding stalls the whole robot
   * loop for ~0.3 s (seen in Q2 at NCFAY when the extender's homing ran with CAN 20 gone).
   */
  public static void applyAsync(SparkBase spark, SparkBaseConfig config) {
    spark.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Push new closed-loop gains at runtime. Does not touch the feedback sensor. */
  public static void setGains(SparkBase spark, double p, double i, double d, double kV, double kS) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(p, i, d).feedForward.kV(kV).kS(kS);
    apply(spark, config, "gain update");
  }

  /**
   * True if the SPARK's status frames are arriving. Reads cached frames only (no CAN request).
   * A SPARK that has dropped off the bus (lost power, tripped breaker, broken CAN wire) times out,
   * and its fault/warning words come back as garbage with nearly every flag set.
   */
  public static boolean isResponding(SparkBase spark) {
    int faultBits = spark.getFaults().rawBits;
    REVLibError error = spark.getLastError();
    if (error == REVLibError.kTimeout || error == REVLibError.kCANDisconnected) {
      return false;
    }
    int warningBits = spark.getWarnings().rawBits;
    return Integer.bitCount(faultBits & 0xFFFF) + Integer.bitCount(warningBits & 0xFFFF) <= kGarbageFlagCount;
  }

  /** The config read-back for either SPARK type. */
  public static SparkBaseConfigAccessor configAccessor(SparkBase spark) {
    if (spark instanceof SparkFlex flex) {
      return flex.configAccessor;
    }
    return ((SparkMax) spark).configAccessor;
  }

  /** Every SPARK created so far (read-only), for fault monitoring. */
  public static List<SparkBase> all() {
    return Collections.unmodifiableList(s_all);
  }

  /**
   * Log a failed configure() once. A missing or duplicate CAN ID or a timeout here means the
   * motor runs on whatever config is in its flash, not ours.
   */
  private static void check(SparkBase spark, String what, REVLibError result) {
    if (result != REVLibError.kOk) {
      DataLogManager.log("[MOTOR] CAN " + spark.getDeviceId() + " " + what + " FAILED: " + result);
    }
  }
}
