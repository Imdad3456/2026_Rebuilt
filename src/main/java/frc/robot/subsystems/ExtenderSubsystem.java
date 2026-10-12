package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HomeExtender;
import frc.robot.configs.ExtenderConfigs;
import frc.robot.constants.ExtenderConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.util.hardware.MotorControllerWrapper;

/**
 * The ExtenderSubsystem controls a two-motor extender mechanism on the robot.
 * It manages motor control, position tracking, synchronization between motors,
 * and implements safety features like skew error detection and homing.
 */
public class ExtenderSubsystem extends SubsystemBase {
  /** The leader motor for the extender mechanism. */
  private final MotorControllerWrapper m_ExtenderLeaderMotor;
  /**
   * The follower motor for the extender mechanism, synchronized with the leader.
   */
  private final MotorControllerWrapper m_ExtenderFollowMotor;

  /** The global target position in inches for the extender mechanism. */
  private double m_globalTargetInches = 0.0;
  /**
   * Flag indicating if the extender mechanism has been homed and is trusted for
   * position control.
   */
  private boolean m_isHomed;
  /**
   * Latched after a skew/current safety trip. While faulted the extender is not
   * driven at all; re-homing clears it.
   */
  private boolean m_faulted = false;

  /**
   * Constructs a new ExtenderSubsystem.
   * Initializes the leader and follower motors, their closed-loop controllers,
   * and configures them with predefined settings.
   */
  public ExtenderSubsystem(MotorControllerWrapper leader, MotorControllerWrapper follower) {
    this.m_ExtenderLeaderMotor = leader;
    this.m_ExtenderFollowMotor = follower;

    // The follower mirrors the leader in hardware (see ExtenderConfigs.followConfig).
    // Only the leader is ever commanded; the follower's encoder is just used for the skew check.

    if (GlobalConstants.IS_BENCHTOP) {
      this.m_isHomed = true;
      this.m_ExtenderLeaderMotor.setPosition(0.0);
      this.m_ExtenderFollowMotor.setPosition(0.0);
      System.out.println(">>> [EXTENDER] Benchtop detected: Auto-Homing enabled.");
    } else {
      this.m_isHomed = false;
    }
    enableSoftLimits();
  }

  @Override
  public void periodic() {
    double leaderPos = getPositionInInches(m_ExtenderLeaderMotor);
    double followPos = getPositionInInches(m_ExtenderFollowMotor);
    double error = Math.abs(leaderPos - followPos); 

    SmartDashboard.putNumber("Extender Ldr Pos", leaderPos);
    SmartDashboard.putNumber("Extender Flw Pos", followPos);
    SmartDashboard.putNumber("Extender Error", error);
    SmartDashboard.putBoolean("EXT HOMED", m_isHomed);
    SmartDashboard.putNumber("Extender Target", m_globalTargetInches);
    SmartDashboard.putBoolean("Extender/Faulted", m_faulted);

    double leaderCurrent = m_ExtenderLeaderMotor.getOutputCurrent();
    double followCurrent = m_ExtenderFollowMotor.getOutputCurrent();
    SmartDashboard.putNumber("EXT LEAD CURRENT", leaderCurrent);
    SmartDashboard.putNumber("EXT FOLL CURRENT", followCurrent);

    if (DriverStation.isDisabled()) {
        if (ExtenderConstants.kExtenderP.hasChanged() || ExtenderConstants.kExtenderI.hasChanged()
               || ExtenderConstants.kExtenderD.hasChanged() || ExtenderConstants.kExtenderFF.hasChanged()) {
            m_ExtenderLeaderMotor.setPID(ExtenderConstants.kExtenderP.get(), ExtenderConstants.kExtenderI.get(),
             ExtenderConstants.kExtenderD.get(), ExtenderConstants.kExtenderFF.get(), ExtenderConstants.kExtenderStatic.get());
        }
    }

    // Homing drives the motors with raw voltage and is how a fault gets cleared,
    // so don't run the safety checks or fight it with a setpoint.
    if (getCurrentCommand() instanceof HomeExtender) {
      return;
    }

    // Latched fault: keep the motors off until the extender is re-homed.
    if (m_faulted) {
      stop();
      return;
    }

    // Safety 1: The Skew Check
    if (error > ExtenderConstants.kMaxPositionDifference.get()) {
        emergencyStop("SKEW DETECTED: " + error);
        return;
    }

    // Safety 2: The Current Imbalance
    if (Math.abs(leaderCurrent - followCurrent) > GlobalConstants.kHighCurrentLimit) {
        emergencyStop("CURRENT IMBALANCE: L:" + leaderCurrent + " F:" + followCurrent);
        return;
    }

    // Only the leader is commanded. Sending a setpoint to the follower would knock
    // it out of follower mode and the two sides would fight each other.
    m_ExtenderLeaderMotor.setTargetValue(m_globalTargetInches, ControlType.kMAXMotionPositionControl);
}

private void emergencyStop(String reason) {
    stop();
    m_isHomed = false;
    m_faulted = true;
    SmartDashboard.putString("Extender/Status", "CRITICAL FAILURE: " + reason);
    // Printed once (the fault latches). Printing every 20ms floods the console
    // stream to the Driver Station and adds to radio traffic.
    DriverStation.reportError(">>> [EXTENDER] " + reason + " - re-home to clear", false);
}

  /**
   * Sets the global target position for the extender to its maximum outward
   * position.
   * Movement will only occur if the extender is homed.
   */
  public void moveOut() {
    // Safety check: Do not move if the system is not homed, as position is
    // untrusted
    /*if (!m_isHomed) {
      return;
    }*/
    m_globalTargetInches = ExtenderConstants.kExtenderMotorOut;
  }

  /**
   * Sets the global target position for the extender to its maximum inward
   * (retracted) position.
   * Movement will only occur if the extender is homed.
   */
  public void moveIn() {
    // Safety check: Do not move if the system is not homed
    /*if (!m_isHomed) {
      return;
    }*/
    m_globalTargetInches = ExtenderConstants.kExtenderMotorIn;
  }

  /**
   * Resets the encoders of both extender motors to zero and sets the system as
   * homed.
   * This assumes the extender is in a known zero position (e.g., against a limit
   * switch).
   */
  public void resetEncoders() {
    m_ExtenderLeaderMotor.setPosition(0);
    m_ExtenderFollowMotor.setPosition(0);
    this.m_globalTargetInches = 0; // Reset target to zero
    this.m_isHomed = true; // Extender is now square and trusted
    this.m_faulted = false;
    SmartDashboard.putString("Extender/Status", "READY");
    SmartDashboard.putBoolean("Extender/SKEW_ERROR", false);
  }

  /**
   * Stops both extender motors immediately.
   */
  public void stop() {
    m_ExtenderLeaderMotor.stop();
  }

  /**
   * Retrieves the current position of a specified extender motor in inches.
   * 
   * @param motor The SparkMax motor to query.
   * @return The position of the motor's encoder in inches.
   */
  public double getPositionInInches(MotorControllerWrapper motor) {
    return motor.getPosition();
  }

  /**
   * Checks if the extender mechanism is within a defined tolerance of its target
   * position.
   * 
   * @param target The target position in inches to check against.
   * @return True if the leader motor's position is within `kAtTargetTolerance` of
   *         the target, false otherwise.
   */
  public boolean atTarget(double target) {
    // Checking leader is usually enough, but you can check both for extra safety
    return Math.abs(target - getPositionInInches(m_ExtenderLeaderMotor)) < ExtenderConstants.kAtTargetTolerance.get();
  }

  /**
   * Sets the raw voltage for both extender motors.
   * This method is typically used during homing procedures to bypass PID control
   * and apply direct motor power.
   * 
   * @param voltage The voltage to apply to the motors.
   */
  public void setHomingVoltages(double voltage) {
    // The follower mirrors the leader in hardware.
    m_ExtenderLeaderMotor.setOutputVoltage(voltage);
  }

  /**
   * Checks if the extender mechanism is at its home (retracted) position,
   * typically indicated
   * by both motors drawing significant current due to hitting a mechanical stop
   * during homing.
   * 
   * @return True if both leader and follower motors exceed the homing voltage
   *         threshold, false otherwise.
   */
  public boolean isAtHome() {
    return m_ExtenderLeaderMotor.getOutputCurrent() > ExtenderConstants.kMaxHomingVoltage.get()
        && m_ExtenderFollowMotor.getOutputCurrent() > ExtenderConstants.kMaxHomingVoltage.get();
  }

  /**
   * Sets the homing status of the extender.
   * 
   * @param homed True if the extender has been successfully homed, false
   *              otherwise.
   */
  public void setIsHomed(boolean homed) {
    this.m_isHomed = homed;
  }

  /**
   * Prepares the extender motors for a homing sequence by applying specific
   * homing configurations.
   * This typically disables soft limits to allow the mechanism to reach its
   * physical limits.
   * Only applied to the leader motor; follower motor is expected to follow.
   */
  public void prepareForHoming() {
    // Apply homing configuration to the leader motor, without resetting other
    // parameters and without persisting to flash.
    m_ExtenderLeaderMotor.setConfiguration(ExtenderConfigs.homingConfig);
  }

  /**
   * Re-enables soft limits on the extender motors after a homing sequence is
   * complete.
   * This restores normal operational safety limits to the leader motor.
   */
  public void enableSoftLimits() {
    // Apply the standard leader configuration to the leader motor, without
    // resetting other parameters and without persisting to flash.
    m_ExtenderLeaderMotor.setConfiguration(ExtenderConfigs.leaderConfig);
    m_ExtenderFollowMotor.setConfiguration(ExtenderConfigs.followConfig);
  }
}