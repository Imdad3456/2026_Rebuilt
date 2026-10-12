package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
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

  private double m_targetInches = ExtenderConstants.kExtenderMotorIn;
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
      if (ExtenderConstants.kExtenderP.hasChanged() | ExtenderConstants.kExtenderI.hasChanged()
          | ExtenderConstants.kExtenderD.hasChanged() | ExtenderConstants.kExtenderFF.hasChanged()
          | ExtenderConstants.kExtenderStatic.hasChanged()) {
        Sparks.setGains(m_leader, ExtenderConstants.kExtenderP.get(), ExtenderConstants.kExtenderI.get(),
            ExtenderConstants.kExtenderD.get(), ExtenderConstants.kExtenderFF.get(),
            ExtenderConstants.kExtenderStatic.get());
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

    double skew = getSkewInches();
    if (skew > ExtenderConstants.kMaxPositionDifference.get()) {
      emergencyStop("SKEW DETECTED: " + skew);
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
    m_controller.setSetpoint(m_targetInches, ControlType.kMAXMotionPositionControl);
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
    return runOnce(() -> m_targetInches = ExtenderConstants.kExtenderMotorOut).withName("ExtenderOut");
  }

  /** Move fully in. */
  public Command retract() {
    return runOnce(() -> m_targetInches = ExtenderConstants.kExtenderMotorIn).withName("ExtenderIn");
  }

  /**
   * Drive slowly into the retracted hard stop with soft limits off, then zero both encoders.
   * Gives up after {@link ExtenderConstants#kHomingTimeoutSeconds} instead of pushing forever.
   */
  public Command home() {
    return new FunctionalCommand(
        () -> {
          m_homing = true;
          if (!GlobalConstants.IS_BENCHTOP) {
            Sparks.apply(m_leader, ExtenderConfigs.homingConfig, "homing config");
          }
        },
        () -> {
          if (!GlobalConstants.IS_BENCHTOP) {
            m_leader.setVoltage(ExtenderConstants.kHomingVoltage.get());
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
    m_targetInches = ExtenderConstants.kExtenderMotorIn;
    m_isHomed = true;
    m_faulted = false;
    SmartDashboard.putString("Extender/Status", "READY");
  }

  /** Both motors stalled against the hard stop. */
  private boolean isAtHome() {
    double threshold = ExtenderConstants.kMaxHomingVoltage.get();
    return m_leader.getOutputCurrent() > threshold && m_follower.getOutputCurrent() > threshold;
  }

  public boolean atTarget() {
    return Math.abs(m_targetInches - getPositionInches()) < ExtenderConstants.kAtTargetTolerance.get();
  }

  @Logged(name = "PositionInches")
  public double getPositionInches() {
    return m_leaderEncoder.getPosition();
  }

  @Logged(name = "FollowerPositionInches")
  public double getFollowerPositionInches() {
    return m_followerEncoder.getPosition();
  }

  @Logged(name = "SkewInches")
  public double getSkewInches() {
    return Math.abs(getPositionInches() - getFollowerPositionInches());
  }

  @Logged(name = "TargetInches")
  public double getTargetInches() {
    return m_targetInches;
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
