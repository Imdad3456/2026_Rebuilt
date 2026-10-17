package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ExtenderConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ExtenderConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.util.hardware.Sparks;

/**
 * The two-motor extender ("the box"). The follower mirrors the leader in hardware (see
 * ExtenderConfigs.followConfig), so only the leader is ever commanded. The follower's encoder
 * is read only for the skew safety check.
 *
 * <p>Zero: the SPARKs' encoders start at 0 when they power on, and the rules require the robot to
 * start with the box retracted, so power-on = home. The code never re-zeroes on startup, so a
 * robot-code restart keeps the real position. If a SPARK itself reboots (brownout), its encoder
 * snaps back to 0; that jump is detected and the extender stops until re-homed.
 *
 * <p>The extender is only driven while homed and not faulted. A failed or interrupted homing run
 * leaves it stopped and un-homed. Re-home with {@link #home()} to recover.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class ExtenderSubsystem extends SubsystemBase {
  private final SparkMax m_leader = Sparks.create(CanIdConstants.kExtenderMotor1, ExtenderConfigs.leaderConfig);
  private final SparkMax m_follower = Sparks.create(CanIdConstants.kExtenderMotor2, ExtenderConfigs.followConfig);
  private final RelativeEncoder m_leaderEncoder = m_leader.getEncoder();
  private final RelativeEncoder m_followerEncoder = m_follower.getEncoder();
  private final SparkClosedLoopController m_controller = m_leader.getClosedLoopController();

  /** Target position in meters. Starts wherever the box is (0 on a real power-on). */
  private double m_target;
  private double m_prevLeaderPos;
  private double m_prevFollowerPos;
  private double m_prevTime = Double.NaN;
  /** Both motors must stay stalled briefly before homing counts as done (ignores current spikes). */
  private final Debouncer m_stallDebouncer = new Debouncer(ExtenderConstants.kHomingStallSeconds);
  /** The power-on position counts as home (the robot must start with the box retracted). */
  private boolean m_isHomed = true;
  /** Latched after a skew/current safety trip. The extender isn't driven until re-homed. */
  private boolean m_faulted = false;
  /** True while the homing command drives the motors with raw voltage. */
  private boolean m_homing = false;

  public ExtenderSubsystem() {
    // Deliberately no setPosition(0) here: see the class comment.
    m_target = m_leaderEncoder.getPosition();
    rememberPositions();
    SmartDashboard.putString("Extender/Status", "READY");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Extender/Homed", m_isHomed);
    SmartDashboard.putBoolean("Extender/Faulted", m_faulted);

    if (DriverStation.isDisabled()) {
      if (ExtenderConstants.kP.hasChanged() | ExtenderConstants.kI.hasChanged()
          | ExtenderConstants.kD.hasChanged() | ExtenderConstants.kV.hasChanged()
          | ExtenderConstants.kS.hasChanged()) {
        Sparks.setGains(m_leader, ExtenderConstants.kP.get(), ExtenderConstants.kI.get(),
            ExtenderConstants.kD.get(), ExtenderConstants.kV.get(), ExtenderConstants.kS.get());
      }
    }

    double now = Timer.getFPGATimestamp();
    double leaderPos = m_leaderEncoder.getPosition();
    double followerPos = m_followerEncoder.getPosition();
    double dt = Double.isNaN(m_prevTime) ? 0.0 : now - m_prevTime;
    boolean jumped = isEncoderJump(m_prevLeaderPos, leaderPos, dt)
        || isEncoderJump(m_prevFollowerPos, followerPos, dt);
    m_prevLeaderPos = leaderPos;
    m_prevFollowerPos = followerPos;
    m_prevTime = now;

    // Homing drives the motors itself and is how a fault gets cleared.
    if (m_homing) {
      return;
    }

    if (jumped) {
      emergencyStop("ENCODER JUMPED (a SPARK probably rebooted)");
      return;
    }

    // Only drive while the position is trusted.
    if (m_faulted || !m_isHomed) {
      m_leader.stopMotor();
      return;
    }

    double skew = Math.abs(leaderPos - followerPos);
    if (skew > ExtenderConstants.kMaxSkew) {
      emergencyStop(String.format("SKEW DETECTED: %.2f in", Units.metersToInches(skew)));
      return;
    }
    double leaderAmps = m_leader.getOutputCurrent();
    double followerAmps = m_follower.getOutputCurrent();
    if (Math.abs(leaderAmps - followerAmps) > GlobalConstants.kHighCurrentLimit) {
      emergencyStop("CURRENT IMBALANCE: L:" + leaderAmps + " F:" + followerAmps);
      return;
    }

    // Only the leader is commanded. A setpoint sent to the follower would knock it out of
    // follower mode and the two sides would fight.
    m_controller.setSetpoint(m_target, ControlType.kMAXMotionPositionControl);
  }

  private void emergencyStop(String reason) {
    m_leader.stopMotor();
    m_isHomed = false;
    m_faulted = true;
    SmartDashboard.putString("Extender/Status", "CRITICAL FAILURE: " + reason);
    // Reported once; the fault latches.
    DriverStation.reportError(">>> [EXTENDER] " + reason + " - re-home to clear", false);
  }

  /** Move fully out. */
  public Command extend() {
    return runOnce(() -> m_target = ExtenderConstants.kExtended).withName("ExtenderOut");
  }

  /** Move fully in. */
  public Command retract() {
    return runOnce(() -> m_target = ExtenderConstants.kRetracted).withName("ExtenderIn");
  }

  /**
   * While running: if the box is out, pull it in {@link ExtenderConstants#kAgitateDepth} and push it
   * back out, over and over, to shake loose balls stuck in the box. When it ends the box returns to
   * where it was. If the box is in, it does nothing.
   */
  public Command agitate() {
    Timer timer = new Timer();
    double[] base = new double[1];
    return new FunctionalCommand(
        () -> {
          base[0] = m_target;
          timer.restart();
        },
        () -> {
          if (base[0] < ExtenderConstants.kAgitateMinPosition) {
            return;
          }
          // Starts with an "in" stroke, then alternates.
          boolean inStroke = ((long) (timer.get() / ExtenderConstants.kAgitateStrokeSeconds)) % 2 == 0;
          m_target = inStroke ? base[0] - ExtenderConstants.kAgitateDepth : base[0];
        },
        interrupted -> m_target = base[0],
        () -> false,
        this)
        .withName("ExtenderAgitate");
  }

  /**
   * Drive slowly into the retracted hard stop with soft limits off, then zero both encoders.
   * Gives up after {@link ExtenderConstants#kHomingTimeoutSeconds} instead of pushing forever.
   */
  public Command home() {
    return new FunctionalCommand(
        () -> {
          m_homing = true;
          m_stallDebouncer.calculate(false);
          if (!GlobalConstants.IS_BENCHTOP) {
            Sparks.apply(m_leader, ExtenderConfigs.softLimitsOff, "soft limits off for homing");
          }
        },
        () -> {
          if (!GlobalConstants.IS_BENCHTOP) {
            m_leader.setVoltage(ExtenderConstants.kHomingVoltage);
          }
        },
        interrupted -> {
          m_leader.stopMotor();
          if (!GlobalConstants.IS_BENCHTOP) {
            Sparks.apply(m_leader, ExtenderConfigs.softLimitsOn, "soft limits back on");
          }
          finishHoming(!interrupted);
        },
        () -> GlobalConstants.IS_BENCHTOP || isAtHome(),
        this)
        .withTimeout(ExtenderConstants.kHomingTimeoutSeconds)
        .withName("HomeExtender");
  }

  /**
   * End of a homing run. Success zeroes the encoders and clears any fault. Failure (timeout, or
   * interrupted by another command) leaves the extender stopped and un-homed: the position is not
   * trusted, so it must not resume moving to its old target.
   */
  void finishHoming(boolean succeeded) {
    m_homing = false;
    if (succeeded) {
      m_leaderEncoder.setPosition(0);
      m_followerEncoder.setPosition(0);
      rememberPositions();
      m_target = ExtenderConstants.kRetracted;
      m_isHomed = true;
      m_faulted = false;
      SmartDashboard.putString("Extender/Status", "READY");
    } else {
      m_leader.stopMotor();
      m_isHomed = false;
      m_target = ExtenderConstants.kRetracted;
      SmartDashboard.putString("Extender/Status", "HOMING FAILED - press home again");
      DriverStation.reportWarning(">>> [EXTENDER] homing did not finish; extender stopped until re-homed", false);
    }
  }

  /** The zeroing above is an expected jump; don't flag it next loop. */
  private void rememberPositions() {
    m_prevLeaderPos = m_leaderEncoder.getPosition();
    m_prevFollowerPos = m_followerEncoder.getPosition();
    m_prevTime = Double.NaN;
  }

  /**
   * True if a position changed more than the mechanism can physically move in {@code dtSeconds}
   * (plus margin), which means the encoder was reset, e.g. its SPARK rebooted. A jump that lands
   * near the current position doesn't matter, so tiny changes never count.
   */
  static boolean isEncoderJump(double previous, double current, double dtSeconds) {
    if (dtSeconds <= 0) {
      return false;
    }
    double maxMove = ExtenderConstants.kMaxPlausibleSpeed * dtSeconds + ExtenderConstants.kJumpMargin;
    return Math.abs(current - previous) > maxMove;
  }

  @Logged(name = "Homed")
  public boolean isHomed() {
    return m_isHomed;
  }

  @Logged(name = "Faulted")
  public boolean isFaulted() {
    return m_faulted;
  }

  /** Both motors stalled against the hard stop for a moment. */
  private boolean isAtHome() {
    double threshold = ExtenderConstants.kHomingStallCurrent;
    return m_stallDebouncer.calculate(
        m_leader.getOutputCurrent() > threshold && m_follower.getOutputCurrent() > threshold);
  }

  @Logged(name = "PositionInches")
  public double getPositionInches() {
    return Units.metersToInches(m_leaderEncoder.getPosition());
  }

  @Logged(name = "FollowerPositionInches")
  public double getFollowerPositionInches() {
    return Units.metersToInches(m_followerEncoder.getPosition());
  }

  @Logged(name = "SkewInches")
  public double getSkewInches() {
    return Math.abs(getPositionInches() - getFollowerPositionInches());
  }

  @Logged(name = "TargetInches")
  public double getTargetInches() {
    return Units.metersToInches(m_target);
  }

  @Logged(name = "LeaderCurrentAmps")
  public double getLeaderCurrent() {
    return m_leader.getOutputCurrent();
  }

  @Logged(name = "FollowerCurrentAmps")
  public double getFollowerCurrent() {
    return m_follower.getOutputCurrent();
  }
}
