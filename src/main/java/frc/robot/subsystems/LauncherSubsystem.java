// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.LauncherConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.VisionUtils;
import frc.robot.util.hardware.Talons;

/**
 * The flywheel launcher: a Kraken/Falcon (TalonFX) under closed-loop velocity control.
 *
 * <p>Idle works like Doyenne_2026: turning idle on makes the launcher hold idle speed whenever
 * it isn't shooting, including right after a shot, until idle is turned off.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class LauncherSubsystem extends SubsystemBase {
  private final TalonFX m_motor = Talons.create(CanIdConstants.kLauncherMotor, LauncherConfigs.config);
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final StatusSignal<AngularVelocity> m_velocity = m_motor.getVelocity();
  private final StatusSignal<Current> m_supplyCurrent = m_motor.getSupplyCurrent();

  // Limelight values, subscribed once instead of looked up by name on every call.
  private final DoubleSubscriber m_llValid;
  private final DoubleSubscriber m_llTagId;
  private final DoubleSubscriber m_llTy;

  /** RPM requested by an active shot, or 0 when not shooting. */
  private double m_shotRPM = 0.0;
  private boolean m_idleEnabled = false;

  public LauncherSubsystem() {
    // Only send the signals we use, at the rate we need, and turn the rest off (less CAN traffic).
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_velocity, m_supplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(4, m_motor.getFaultField(), m_motor.getStickyFault_BootDuringEnable());
    m_motor.optimizeBusUtilization();

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);
    m_llValid = limelight.getDoubleTopic(VisionConstants.kTargetValidKey).subscribe(VisionConstants.kDefaultTargetValid);
    m_llTagId = limelight.getDoubleTopic(VisionConstants.kTargetIdKey).subscribe(VisionConstants.kDefaultTargetId);
    m_llTy = limelight.getDoubleTopic(VisionConstants.kTargetYKey).subscribe(VisionConstants.kDefaultTargetY);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(m_velocity, m_supplyCurrent);

    if (DriverStation.isDisabled()) {
      if (LauncherConstants.kP.hasChanged() | LauncherConstants.kI.hasChanged() | LauncherConstants.kD.hasChanged()
          | LauncherConstants.kV.hasChanged() | LauncherConstants.kS.hasChanged()) {
        Slot0Configs gains = new Slot0Configs()
            .withKP(LauncherConstants.kP.get()).withKI(LauncherConstants.kI.get())
            .withKD(LauncherConstants.kD.get()).withKV(LauncherConstants.kV.get())
            .withKS(LauncherConstants.kS.get());
        Talons.check(m_motor, "gain update", m_motor.getConfigurator().apply(gains));
      }
    }

    double target = getTargetRPM();
    if (target > 0) {
      m_motor.setControl(m_velocityRequest.withVelocity(target / 60.0));
    } else {
      m_motor.stopMotor();
    }

    // Operator-facing only; detailed values go to the log through @Logged.
    SmartDashboard.putBoolean("Launcher/At Speed", atSpeed());
    SmartDashboard.putNumber("Launcher/Target RPM", target);
  }

  /** Spin to an RPM read every loop (the operator's shooter slider). Back to idle/off when released. */
  public Command spinUp(DoubleSupplier rpm) {
    return runEnd(() -> m_shotRPM = rpm.getAsDouble(), () -> m_shotRPM = 0.0).withName("LauncherOn");
  }

  /** Hold idle speed whenever not shooting, until turned off. */
  public Command idle() {
    return runOnce(() -> m_idleEnabled = true).withName("LauncherIdle");
  }

  /** Stop idling (and any shot in progress). */
  public Command off() {
    return runOnce(() -> {
      m_idleEnabled = false;
      m_shotRPM = 0.0;
    }).withName("LauncherOff");
  }

  @Logged(name = "TargetRPM")
  public double getTargetRPM() {
    if (m_shotRPM > 0) {
      return m_shotRPM;
    }
    return m_idleEnabled ? LauncherConstants.kIdleRPM.get() : 0.0;
  }

  @Logged(name = "ActualRPM")
  public double getActualRPM() {
    return m_velocity.getValueAsDouble() * 60.0;
  }

  @Logged(name = "SupplyCurrentAmps")
  public double getSupplyCurrent() {
    return m_supplyCurrent.getValueAsDouble();
  }

  /** At the target speed, and the target is a real shot (not idle or stopped). */
  @Logged(name = "AtSpeed")
  public boolean atSpeed() {
    double tolerance = LauncherConstants.kToleranceRPM.get();
    return m_shotRPM > LauncherConstants.kIdleRPM.get() + tolerance
        && Math.abs(m_shotRPM - getActualRPM()) < tolerance;
  }

  /** Map a 0..1 slider position onto the min..max shot RPM range. */
  public static double rpmFromSlider(double percent) {
    double min = LauncherConstants.kMinShotRPM.get();
    double max = LauncherConstants.kMaxShotRPM.get();
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
      return LauncherConstants.kIdleRPM.get();
    }
    double distance = (LauncherConstants.kAprilTagHeight - LauncherConstants.kCameraHeight)
        / Math.tan(Math.toRadians(LauncherConstants.kMountAngle + m_llTy.get()));
    double rpm = distance * LauncherConstants.kRPMPerInch.get() + LauncherConstants.kBaseRPM.get();
    return MathUtil.clamp(rpm, LauncherConstants.kMinShotRPM.get(), LauncherConstants.kMaxShotRPM.get());
  }
}
