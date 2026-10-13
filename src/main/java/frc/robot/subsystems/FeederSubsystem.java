// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.FeederConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.util.hardware.Sparks;

/** Loads game pieces into the launcher. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class FeederSubsystem extends SubsystemBase {
  private final SparkMax m_motor = Sparks.create(CanIdConstants.kFeederMotor, FeederConfigs.config);

  /**
   * Feed while {@code ready} is true and pause while it's false, for as long as this command runs.
   * Used for shooting: each ball waits until the flywheel is back up to speed.
   */
  public Command feedWhile(BooleanSupplier ready) {
    return runEnd(
        () -> m_motor.set(ready.getAsBoolean() ? FeederConstants.kFeederSpeed.get() : 0.0),
        m_motor::stopMotor)
        .withName("FeedWhenReady");
  }

  /** Backwards while held, to clear a jam. */
  public Command reverse() {
    return runEnd(() -> m_motor.set(-FeederConstants.kFeederSpeed.get()), m_motor::stopMotor)
        .withName("FeederReverse");
  }

  @Logged(name = "CurrentAmps")
  public double getCurrent() {
    return m_motor.getOutputCurrent();
  }
}
