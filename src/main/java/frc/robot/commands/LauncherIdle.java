// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * The LauncherIdle command is responsible for setting the robot's launcher mechanism
 * to a predefined idle speed. This keeps the launcher spinning at a low RPM,
 * ready for a quick ramp-up to launch speed.
 */
public class LauncherIdle extends Command {
  /** The LauncherSubsystem instance that this command will control. */
  LauncherSubsystem m_Launcher;

  /**
   * Creates a new LauncherIdle command.
   *
   * @param launcher The LauncherSubsystem to be controlled by this command.
   */
  public LauncherIdle(LauncherSubsystem launcher) {
    this.m_Launcher = launcher;
    addRequirements(this.m_Launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The launcher's closed loop can't drive in reverse (see LauncherConfigs), so
    // dropping from shot speed to idle just coasts down instead of braking hard.
    m_Launcher.setLauncherVelocity(LauncherConstants.kLauncherMotorSpeedIdle.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the launcher motor completely when the command ends or is interrupted.
    this.m_Launcher.stopLauncher();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command is designed to run indefinitely until interrupted,
    // for example, by releasing a button or another command taking over.
    return false;
  }
}
