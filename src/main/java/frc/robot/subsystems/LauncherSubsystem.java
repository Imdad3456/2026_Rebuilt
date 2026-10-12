// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.LauncherConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.VisionUtils;
import frc.robot.util.hardware.Sparks;

/** The flywheel launcher, run with closed-loop velocity control in RPM. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class LauncherSubsystem extends SubsystemBase {
  private final SparkMax m_motor =
      Sparks.create(CanIdConstants.kLauncherLeaderMotor, LauncherConfigs.config);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();

  // Limelight values, subscribed once instead of looked up by name on every call.
  private final DoubleSubscriber m_llValid;
  private final DoubleSubscriber m_llTagId;
  private final DoubleSubscriber m_llTy;

  private double m_targetRPM = 0.0;

  public LauncherSubsystem() {
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);
    m_llValid = limelight.getDoubleTopic(VisionConstants.kTargetValidKey).subscribe(VisionConstants.kDefaultTargetValid);
    m_llTagId = limelight.getDoubleTopic(VisionConstants.kTargetIdKey).subscribe(VisionConstants.kDefaultTargetId);
    m_llTy = limelight.getDoubleTopic(VisionConstants.kTargetYKey).subscribe(VisionConstants.kDefaultTargetY);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (LauncherConstants.kLauncherP.hasChanged() | LauncherConstants.kLauncherI.hasChanged()
          | LauncherConstants.kLauncherD.hasChanged() | LauncherConstants.kLauncherkV.hasChanged()
          | LauncherConstants.kLauncherStatic.hasChanged()) {
        Sparks.setGains(m_motor, LauncherConstants.kLauncherP.get(), LauncherConstants.kLauncherI.get(),
            LauncherConstants.kLauncherD.get(), LauncherConstants.kLauncherkV.get(),
            LauncherConstants.kLauncherStatic.get());
      }
    }

    if (m_targetRPM > 0) {
      m_controller.setSetpoint(m_targetRPM, ControlType.kVelocity);
    } else {
      m_motor.stopMotor();
    }

    // Operator-facing only; detailed values go to the log through @Logged.
    SmartDashboard.putBoolean("Launcher/At Speed", atSpeed());
    SmartDashboard.putNumber("Launcher/Target RPM", m_targetRPM);
  }

  /** Spin to an RPM read every loop (e.g. the operator's shooter slider). Stops when released. */
  public Command spinUp(DoubleSupplier rpm) {
    return runEnd(() -> m_targetRPM = rpm.getAsDouble(), this::stopLauncher).withName("LauncherOn");
  }

  /**
   * Hold idle speed until another launcher command takes over. The flywheel can't be driven
   * backwards (see LauncherConfigs), so dropping from shot speed coasts down instead of braking.
   */
  public Command idle() {
    return runEnd(() -> m_targetRPM = LauncherConstants.kLauncherMotorSpeedIdle.get(), this::stopLauncher)
        .withName("LauncherIdle");
  }

  public Command off() {
    return runOnce(this::stopLauncher).withName("LauncherOff");
  }

  private void stopLauncher() {
    m_targetRPM = 0.0;
    m_motor.stopMotor();
  }

  @Logged(name = "TargetRPM")
  public double getTargetRPM() {
    return m_targetRPM;
  }

  @Logged(name = "ActualRPM")
  public double getActualVelocity() {
    return m_encoder.getVelocity();
  }

  @Logged(name = "CurrentAmps")
  public double getCurrent() {
    return m_motor.getOutputCurrent();
  }

  /** At the target speed, and the target is a real shot (not idle or stopped). */
  @Logged(name = "AtSpeed")
  public boolean atSpeed() {
    boolean nearTarget = Math.abs(m_targetRPM - getActualVelocity()) < LauncherConstants.kLauncherTolerance.get();
    boolean shotSpeed = m_targetRPM
        > LauncherConstants.kLauncherMotorSpeedIdle.get() + LauncherConstants.kLaunchMinShotBuffer.get();
    return nearTarget && shotSpeed;
  }

  /** Map a 0..1 slider position onto the min..max shot RPM range. */
  public static double rpmFromSlider(double percent) {
    double min = LauncherConstants.kLaunchMinRPM.get();
    double max = LauncherConstants.kLaunchMaxRPM.get();
    return min + (max - min) * MathUtil.clamp(percent, 0.0, 1.0);
  }

  /**
   * Shot RPM from the Limelight's distance to our hub, or idle speed (so atSpeed() is false and
   * nothing feeds) when the right target isn't visible. Not bound to a button yet.
   */
  public double calculateRPMFromLimelight() {
    boolean hasTarget = m_llValid.get() == 1.0;
    int tagId = (int) m_llTagId.get();
    if (!hasTarget || !VisionUtils.isTargetingCorrectHoop(tagId)) {
      return LauncherConstants.kLauncherMotorSpeedIdle.get();
    }
    double distance = (LauncherConstants.kAprilTagHeight - LauncherConstants.kCameraHeight)
        / Math.tan(Math.toRadians(LauncherConstants.kMountAngle + m_llTy.get()));
    double rpm = distance * LauncherConstants.kRPMPerInch.get() + LauncherConstants.kBaseRPM.get();
    return MathUtil.clamp(rpm, LauncherConstants.kLaunchMinRPM.get(), LauncherConstants.kLaunchMaxRPM.get());
  }
}
