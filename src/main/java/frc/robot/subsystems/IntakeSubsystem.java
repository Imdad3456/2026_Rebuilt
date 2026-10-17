// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.IntakeConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.util.hardware.Sparks;

/** Front pickup rollers that pull game pieces into the robot. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex m_motor = Sparks.createFlex(CanIdConstants.kIntakeMotor, IntakeConfigs.config);

  /** Run inward while held, at a speed (0..1) read every loop (the operator's intake slider). */
  public Command receive(DoubleSupplier speed) {
    return runEnd(() -> m_motor.set(speed.getAsDouble()), m_motor::stopMotor).withName("IntakeReceive");
  }

  /** Run outward while held, at a speed (0..1) read every loop. */
  public Command eject(DoubleSupplier speed) {
    return runEnd(() -> m_motor.set(-speed.getAsDouble()), m_motor::stopMotor).withName("IntakeEject");
  }

  public Command stop() {
    return runOnce(m_motor::stopMotor).withName("IntakeStop");
  }

  @Logged(name = "AppliedOutput")
  public double getAppliedOutput() {
    return m_motor.getAppliedOutput();
  }

  @Logged(name = "CurrentAmps")
  public double getCurrent() {
    return m_motor.getOutputCurrent();
  }
}
