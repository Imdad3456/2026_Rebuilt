package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.util.diagnostics.RobotDiagnostics;
import frc.robot.util.hardware.Sparks;

/**
 * The two-motor extender ("the box"). No follower mode: each motor runs its own closed loop on its
 * own encoder toward the same target (see ExtenderConfigs). Each side corrects itself, and if one
 * side's CAN flickers the other keeps working. CAN 20 ("leader") is the reference for position and
 * targets; CAN 21 ("follower" in names below, now just the second motor) is checked against it by
 * the skew safety check.
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
  private final SparkMax m_leader = Sparks.create(CanIdConstants.kExtenderMotor1, ExtenderConfigs.motor1Config);
  private final SparkMax m_follower = Sparks.create(CanIdConstants.kExtenderMotor2, ExtenderConfigs.motor2Config);
  private final RelativeEncoder m_leaderEncoder = m_leader.getEncoder();
  private final RelativeEncoder m_followerEncoder = m_follower.getEncoder();
  private final SparkClosedLoopController m_leaderController = m_leader.getClosedLoopController();
  private final SparkClosedLoopController m_followerController = m_follower.getClosedLoopController();

  /** Target position in meters. Starts wherever the box is (0 on a real power-on). */
  private double m_target;
  private double m_prevLeaderPos;
  private double m_prevFollowerPos;
  private double m_prevTime = Double.NaN;
  /**
   * After we zero the encoders ourselves, ignore jumps until this time. A real SPARK applies the
   * new zero over CAN and reports it a status frame or two later, so the old reading briefly
   * lingers and the change to 0 would otherwise look like a reboot.
   */
  private double m_ignoreJumpsUntil = Double.NEGATIVE_INFINITY;
  /** Pushing without moving, short of the target, for a moment (see kStallSeconds). */
  private final Debouncer m_stoppedShortDebouncer = new Debouncer(ExtenderConstants.kStallSeconds);
  /** Both motors must stay stalled briefly before homing counts as done (ignores current spikes). */
  private final Debouncer m_stallDebouncer = new Debouncer(ExtenderConstants.kHomingStallSeconds);
  /** The motors' currents must disagree for a while before it counts (ignores startup surges). */
  private final Debouncer m_imbalanceDebouncer = new Debouncer(ExtenderConstants.kImbalanceSeconds);
  /** Homing refused because the leader SPARK isn't on the CAN bus. */
  private boolean m_homeBlocked = false;
  /** The power-on position counts as home (the robot must start with the box retracted). */
  private boolean m_isHomed = true;
  /** Latched after a skew/current safety trip. The extender isn't driven until re-homed. */
  private boolean m_faulted = false;
  /** True while the homing command drives the motors with raw voltage. */
  private boolean m_homing = false;
  /** Human-readable state for the drive team (dashboard and alerts). */
  private String m_status = "READY";

  public ExtenderSubsystem() {
    // Deliberately no setPosition(0) here: see the class comment.
    m_target = m_leaderEncoder.getPosition();
    rememberPositions();
    setStatus("READY");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Extender/Homed", m_isHomed);
    SmartDashboard.putBoolean("Extender/Faulted", m_faulted);

    if (DriverStation.isDisabled()) {
      if (ExtenderConstants.kP.hasChanged() | ExtenderConstants.kI.hasChanged()
          | ExtenderConstants.kD.hasChanged() | ExtenderConstants.kV.hasChanged()
          | ExtenderConstants.kS.hasChanged()) {
        for (SparkMax motor : new SparkMax[] {m_leader, m_follower}) {
          Sparks.setGains(motor, ExtenderConstants.kP.get(), ExtenderConstants.kI.get(),
              ExtenderConstants.kD.get(), ExtenderConstants.kV.get(), ExtenderConstants.kS.get());
        }
      }
    }

    double now = Timer.getFPGATimestamp();
    double leaderPos = m_leaderEncoder.getPosition();
    double followerPos = m_followerEncoder.getPosition();
    double dt = Double.isNaN(m_prevTime) ? 0.0 : now - m_prevTime;
    boolean jumped = now >= m_ignoreJumpsUntil
        && (isEncoderJump(m_prevLeaderPos, leaderPos, dt) || isEncoderJump(m_prevFollowerPos, followerPos, dt));
    m_prevLeaderPos = leaderPos;
    m_prevFollowerPos = followerPos;
    m_prevTime = now;

    // Homing drives the motors itself and is how a fault gets cleared.
    if (m_homing) {
      return;
    }

    if (jumped) {
      emergencyStop("ENCODER JUMPED (a SPARK probably rebooted)", false);
      return;
    }

    // Only drive while the position is trusted.
    if (m_faulted || !m_isHomed) {
      stopMotors();
      m_imbalanceDebouncer.calculate(false);
      return;
    }

    double skew = Math.abs(leaderPos - followerPos);
    if (skew > ExtenderConstants.kMaxSkew) {
      emergencyStop(String.format("SKEW DETECTED: %.2f in", Units.metersToInches(skew)), true);
      return;
    }
    double leaderAmps = m_leader.getOutputCurrent();
    double followerAmps = m_follower.getOutputCurrent();
    if (m_imbalanceDebouncer.calculate(Math.abs(leaderAmps - followerAmps) > ExtenderConstants.kImbalanceAmps)) {
      m_imbalanceDebouncer.calculate(false);
      emergencyStop("CURRENT IMBALANCE: L:" + leaderAmps + " F:" + followerAmps, true);
      return;
    }

    // Pushing but not moving, short of the target: the target is past a hard stop (or the box is
    // jammed). Holding that target would run the NEOs at stall indefinitely, so accept where the
    // box actually is.
    boolean stoppedShort = Math.abs(m_target - leaderPos) > ExtenderConstants.kAllowedClosedLoopError
        && Math.abs(m_leaderEncoder.getVelocity()) < ExtenderConstants.kStallSpeed
        && Math.abs(m_leader.getAppliedOutput()) > ExtenderConstants.kStallMinOutput;
    if (m_stoppedShortDebouncer.calculate(stoppedShort)) {
      setStatus(String.format("STOPPED SHORT at %.1f in (target %.1f in) - holding there",
          Units.metersToInches(leaderPos), Units.metersToInches(m_target)));
      RobotDiagnostics.logEvent("EXTENDER " + m_status);
      m_target = leaderPos;
      m_stoppedShortDebouncer.calculate(false);
    }

    // Both sides drive to the same target, each on its own encoder.
    m_leaderController.setSetpoint(m_target, ControlType.kMAXMotionPositionControl);
    m_followerController.setSetpoint(m_target, ControlType.kMAXMotionPositionControl);
  }

  private void stopMotors() {
    m_leader.stopMotor();
    m_follower.stopMotor();
  }

  /**
   * Stop and latch a fault. {@code positionStillValid}: the encoders can still be trusted (skew or
   * current trips), so "clear faults" can resume. If not (an encoder reset), it must be re-homed.
   */
  private void emergencyStop(String reason, boolean positionStillValid) {
    stopMotors();
    if (!positionStillValid) {
      m_isHomed = false;
    }
    m_faulted = true;
    // Hold where it stopped when resumed, instead of lunging back to the old target.
    m_target = m_leaderEncoder.getPosition();
    setStatus("CRITICAL FAILURE: " + reason);
    // Reported once; the fault latches.
    DriverStation.reportError(">>> [EXTENDER] " + reason + " - re-home to clear", false);
  }

  /**
   * Clear a latched fault (March "clear extender faults" button). After a skew or current trip it
   * resumes, holding where it is. After an encoder reset it stays stopped until re-homed, because
   * its position can't be trusted.
   */
  public Command clearFaults() {
    return runOnce(() -> {
      m_faulted = false;
      m_target = m_leaderEncoder.getPosition();
      setStatus(m_isHomed ? "READY" : "NOT HOMED - press home");
    }).ignoringDisable(true).withName("ExtenderClearFaults");
  }

  /** Stop where it is (used by stop-all). */
  public Command holdPosition() {
    return runOnce(() -> m_target = m_leaderEncoder.getPosition()).withName("ExtenderHold");
  }

  /**
   * Manual mode while held: the stick moves the box in or out (positive = out), within the normal
   * travel. Uses the same smooth motion control and safety checks as everything else.
   */
  public Command manual(DoubleSupplier stick) {
    double[] lastTime = new double[1];
    return new FunctionalCommand(
        () -> {
          lastTime[0] = Timer.getFPGATimestamp();
          m_target = m_leaderEncoder.getPosition();
        },
        () -> {
          double now = Timer.getFPGATimestamp();
          double input = MathUtil.applyDeadband(stick.getAsDouble(), ExtenderConstants.kManualDeadband);
          m_target = MathUtil.clamp(
              m_target + input * ExtenderConstants.kManualSpeed * (now - lastTime[0]),
              ExtenderConstants.kRetracted, ExtenderConstants.kExtended);
          lastTime[0] = now;
        },
        interrupted -> m_target = m_leaderEncoder.getPosition(),
        () -> false,
        this)
        .withName("ExtenderManual");
  }

  /** Move fully out. */
  public Command extend() {
    return runOnce(() -> moveTo(ExtenderConstants.kExtended)).withName("ExtenderOut");
  }

  /** Move fully in. */
  public Command retract() {
    return runOnce(() -> moveTo(ExtenderConstants.kRetracted)).withName("ExtenderIn");
  }

  private void moveTo(double target) {
    m_target = target;
    if (m_status.startsWith("STOPPED SHORT")) {
      setStatus("READY");
    }
  }

  /**
   * For shooting: if the box is out, move it to the launch position
   * ({@link ExtenderConstants#kLaunchPosition}), then pull in {@link ExtenderConstants#kAgitateDepth}
   * and push back out, over and over, to shake loose balls stuck in the box. When it ends the box
   * returns to where it was. If the box is in, it does nothing.
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
          // Starts at the launch position, then alternates in and back.
          boolean inStroke = ((long) (timer.get() / ExtenderConstants.kAgitateStrokeSeconds)) % 2 == 1;
          double launch = ExtenderConstants.kLaunchPosition;
          m_target = inStroke ? launch - ExtenderConstants.kAgitateDepth : launch;
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
          // A motor that has dropped off the CAN bus can't be homed. Pressing home anyway used to
          // freeze the whole robot ~0.6 s per press (Q2 at NCFAY).
          SparkMax silent = !Sparks.isResponding(m_leader) ? m_leader
              : !Sparks.isResponding(m_follower) ? m_follower : null;
          m_homeBlocked = silent != null;
          if (m_homeBlocked) {
            stopMotors();
            setStatus("CAN " + silent.getDeviceId() + " NOT RESPONDING - can't home; check its breaker/power/CAN wiring");
            DriverStation.reportWarning(">>> [EXTENDER] " + m_status, false);
            return;
          }
          m_homing = true;
          m_stallDebouncer.calculate(false);
          if (!GlobalConstants.IS_BENCHTOP) {
            // Async: never stall the robot loop waiting on the SPARK.
            Sparks.applyAsync(m_leader, ExtenderConfigs.softLimitsOff);
            Sparks.applyAsync(m_follower, ExtenderConfigs.softLimitsOff);
          }
        },
        () -> {
          if (!m_homeBlocked && !GlobalConstants.IS_BENCHTOP) {
            m_leader.setVoltage(ExtenderConstants.kHomingVoltage);
            m_follower.setVoltage(ExtenderConstants.kHomingVoltage);
          }
        },
        interrupted -> {
          stopMotors();
          if (m_homeBlocked) {
            return;
          }
          if (!GlobalConstants.IS_BENCHTOP) {
            Sparks.applyAsync(m_leader, ExtenderConfigs.softLimitsOn);
            Sparks.applyAsync(m_follower, ExtenderConfigs.softLimitsOn);
          }
          finishHoming(!interrupted);
        },
        () -> m_homeBlocked || GlobalConstants.IS_BENCHTOP || isAtHome(),
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
      m_ignoreJumpsUntil = Timer.getFPGATimestamp() + ExtenderConstants.kZeroSettleSeconds;
      m_target = ExtenderConstants.kRetracted;
      m_isHomed = true;
      m_faulted = false;
      setStatus("READY");
    } else {
      stopMotors();
      m_isHomed = false;
      m_target = ExtenderConstants.kRetracted;
      setStatus("HOMING FAILED - press home again");
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

  private void setStatus(String status) {
    m_status = status;
    SmartDashboard.putString("Extender/Status", status);
  }

  public String getStatus() {
    return m_status;
  }

  /** Within the closed-loop tolerance of its target (or it stopped short and now holds there). */
  @Logged(name = "AtTarget")
  public boolean isAtTarget() {
    return Math.abs(m_target - m_leaderEncoder.getPosition()) <= ExtenderConstants.kAllowedClosedLoopError;
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
