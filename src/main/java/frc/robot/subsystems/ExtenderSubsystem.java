package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
 * <p>Assumes the robot powers on with the box fully retracted (the boot position is zero).
 * Re-home with {@link #home()} to re-zero or to clear a safety fault.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class ExtenderSubsystem extends SubsystemBase {
  private final SparkMax m_leader = Sparks.create(CanIdConstants.kExtenderMotor1, ExtenderConfigs.leaderConfig);
  private final SparkMax m_follower = Sparks.create(CanIdConstants.kExtenderMotor2, ExtenderConfigs.followConfig);
  private final RelativeEncoder m_leaderEncoder = m_leader.getEncoder();
  private final RelativeEncoder m_followerEncoder = m_follower.getEncoder();
  private final SparkClosedLoopController m_controller = m_leader.getClosedLoopController();

  /** Target position in meters. */
  private double m_target = ExtenderConstants.kRetracted;
  /** Both motors must stay stalled briefly before homing counts as done (ignores current spikes). */
  private final Debouncer m_stallDebouncer = new Debouncer(ExtenderConstants.kHomingStallSeconds);
  private boolean m_isHomed = GlobalConstants.IS_BENCHTOP;
  /** Latched after a skew/current safety trip. The extender isn't driven until re-homed. */
  private boolean m_faulted = false;
  /** True while the homing command drives the motors with raw voltage. */
  private boolean m_homing = false;

  public ExtenderSubsystem() {
    m_leaderEncoder.setPosition(0);
    m_followerEncoder.setPosition(0);
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

    // Homing drives the motors itself and is how a fault gets cleared.
    if (m_homing) {
      return;
    }

    if (m_faulted) {
      m_leader.stopMotor();
      return;
    }

    double skew = Math.abs(m_leaderEncoder.getPosition() - m_followerEncoder.getPosition());
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
   * Drive slowly into the retracted hard stop with soft limits off, then zero both encoders.
   * Gives up after {@link ExtenderConstants#kHomingTimeoutSeconds} instead of pushing forever.
   */
  public Command home() {
    return new FunctionalCommand(
        () -> {
          m_homing = true;
          m_stallDebouncer.calculate(false);
          if (!GlobalConstants.IS_BENCHTOP) {
            Sparks.apply(m_leader, ExtenderConfigs.homingConfig, "homing config");
          }
        },
        () -> {
          if (!GlobalConstants.IS_BENCHTOP) {
            m_leader.setVoltage(ExtenderConstants.kHomingVoltage);
          }
        },
        interrupted -> {
          m_leader.stopMotor();
          if (!interrupted) {
            resetEncoders();
          }
          if (!GlobalConstants.IS_BENCHTOP) {
            Sparks.apply(m_leader, ExtenderConfigs.leaderConfig, "restore soft limits");
          }
          m_homing = false;
        },
        () -> GlobalConstants.IS_BENCHTOP || isAtHome(),
        this)
        .withTimeout(ExtenderConstants.kHomingTimeoutSeconds)
        .withName("HomeExtender");
  }

  private void resetEncoders() {
    m_leaderEncoder.setPosition(0);
    m_followerEncoder.setPosition(0);
    m_target = ExtenderConstants.kRetracted;
    m_isHomed = true;
    m_faulted = false;
    SmartDashboard.putString("Extender/Status", "READY");
  }

  /** Both motors stalled against the hard stop for a moment. */
  private boolean isAtHome() {
    double threshold = ExtenderConstants.kHomingStallCurrent;
    return m_stallDebouncer.calculate(
        m_leader.getOutputCurrent() > threshold && m_follower.getOutputCurrent() > threshold);
  }

  public boolean atTarget() {
    return Math.abs(m_target - m_leaderEncoder.getPosition()) < ExtenderConstants.kAtTargetTolerance;
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
