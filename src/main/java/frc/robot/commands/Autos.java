// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Hand-written autonomous routines. Path-based autos come from PathPlanner. */
public final class Autos {
  /** Drive straight forward at a fixed speed for a fixed time (robot-relative). */
  public static Command driveStraight(DriveSubsystem drive) {
    return drive.runEnd(() -> drive.drive(AutoConstants.kExampleAutoSpeed, 0, 0), drive::stop)
        .withTimeout(AutoConstants.kExampleAutoTimeout)
        .withName("Simple Auto");
  }

  private Autos() {}
}
