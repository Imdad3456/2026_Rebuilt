// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.configs.ClimberConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.util.hardware.Sparks;

/**
 * Two-motor climber; the follower mirrors the leader in hardware. NOT INSTALLED YET: it is not
 * created in RobotContainer, so these CAN IDs are not used.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax m_leader = Sparks.create(CanIdConstants.kClimberMotor1, ClimberConfigs.leaderConfig);
  @SuppressWarnings("unused") // Only needs to exist and follow.
  private final SparkMax m_follower = Sparks.create(CanIdConstants.kClimberMotor2, ClimberConfigs.followConfig);
  private final RelativeEncoder m_encoder = m_leader.getEncoder();
  private final SparkClosedLoopController m_controller = m_leader.getClosedLoopController();

  private double m_targetPosition = ClimberConstants.kClimberZero;
  private boolean m_isHomed = GlobalConstants.IS_BENCHTOP;
  private boolean m_homing = false;

  @Override
  public void periodic() {
    SmartDashboard.putString("Climber/Status", m_isHomed ? "READY" : "RE-ZERO REQUIRED");

    if (DriverStation.isDisabled()) {
      if (ClimberConstants.kClimberP.hasChanged() | ClimberConstants.kClimberI.hasChanged()
          | ClimberConstants.kClimberD.hasChanged() | ClimberConstants.kClimberFF.hasChanged()) {
        Sparks.setGains(m_leader, ClimberConstants.kClimberP.get(), ClimberConstants.kClimberI.get(),
            ClimberConstants.kClimberD.get(), ClimberConstants.kClimberFF.get(),
            ClimberConstants.kClimberStatic.get());
      }
    }

    if (m_isHomed && !m_homing) {
      m_controller.setSetpoint(m_targetPosition, ControlType.kMAXMotionPositionControl);
    }
  }

  /** Climb up while held; holds position when released. */
  public Command extend() {
    return moveWhileHeld(ClimberConstants.kClimberMaxExtend).withName("ClimberExtend");
  }

  /** Climb down while held; holds position when released. */
  public Command retract() {
    return moveWhileHeld(ClimberConstants.kClimberZero).withName("ClimberRetract");
  }

  private Command moveWhileHeld(double target) {
    return runEnd(
        () -> {
          if (m_isHomed) {
            m_targetPosition = target;
          }
        },
        () -> m_targetPosition = m_encoder.getPosition());
  }

  /** Drive down into the hard stop with soft limits off, then zero. */
  public Command home() {
    return new FunctionalCommand(
        () -> {
          m_homing = true;
          Sparks.apply(m_leader, ClimberConfigs.homingConfig, "homing config");
        },
        () -> m_leader.setVoltage(ClimberConstants.kHomingVoltage.get()),
        interrupted -> {
          m_leader.stopMotor();
          if (!interrupted) {
            m_encoder.setPosition(0);
            m_targetPosition = ClimberConstants.kClimberZero;
            m_isHomed = true;
          }
          Sparks.apply(m_leader, ClimberConfigs.leaderConfig, "restore soft limits");
          m_homing = false;
        },
        () -> m_leader.getOutputCurrent() > ClimberConstants.kMaxHomingVoltage.get(),
        this)
        .withName("HomeClimber");
  }

  @Logged(name = "HeightInches")
  public double getHeight() {
    return m_encoder.getPosition();
  }
}
