// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * The LauncherOn command is responsible for engaging the robot's launcher mechanism.
 * It continuously calculates the required launcher speed using Limelight vision data
 * and commands the LauncherSubsystem to maintain that velocity.
 */
public class LauncherOn extends Command {
  /** The LauncherSubsystem instance that this command will control. */
  private final LauncherSubsystem m_launcher;
  private final DoubleSupplier m_speedSource;
  
  /**
   * Creates a new LauncherOn command.
   *
   * @param launcher The LauncherSubsystem to be controlled by this command.
   */
  public LauncherOn(LauncherSubsystem launcher, DoubleSupplier launchSpeed) {
    this.m_launcher = launcher;
    // Keep the supplier so the slider is read every loop. (Reading it here only
    // sampled the slider once, when the robot code booted.)
    this.m_speedSource = launchSpeed;
    // This command should require the LauncherSubsystem to prevent other commands from
    // interfering with launcher operation.
    addRequirements(this.m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate the target launcher speed based on vision data (e.g., distance to target).
    SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
    //double launcherSpeed = m_launcher.calculateRPMFromLimeLight();
    // Set the launcher motor to the calculated velocity.
    // Slider is -1..1; map it to 0..1.
    double sliderPercent = (m_speedSource.getAsDouble() + 1) / 2;
    this.m_launcher.setLauncherVelocity(m_launcher.calcualteRPMFromSlider(sliderPercent));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the launcher motor completely when the command ends or is interrupted.
    this.m_launcher.stopLauncher();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command is designed to run indefinitely until interrupted,
    // for example, by releasing a button or another command taking over.
    return false;
  }
}
