// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.RollerConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.RollerConstants;
import frc.robot.util.hardware.Sparks;

/** Rollers that move game pieces through the robot toward the feeder. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class Rollers extends SubsystemBase {
  private final SparkMax m_motor = Sparks.create(CanIdConstants.kRollerMotor, RollerConfigs.config);

  /** Toward the feeder while held. */
  public Command forward() {
    return runEnd(() -> m_motor.set(RollerConstants.kRollerTargetSpeed.get()), m_motor::stopMotor)
        .withName("RollerForward");
  }

  /** Back out of the robot while held. */
  public Command reverse() {
    return runEnd(() -> m_motor.set(-RollerConstants.kRollerTargetSpeed.get()), m_motor::stopMotor)
        .withName("RollerReverse");
  }

  @Logged(name = "CurrentAmps")
  public double getCurrent() {
    return m_motor.getOutputCurrent();
  }
}
