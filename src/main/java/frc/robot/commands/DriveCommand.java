// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.diagnostics.RobotDiagnostics;

/** Teleop driving from the driver's stick. The drivetrain's default command. */
public class DriveCommand extends Command {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_ccwRotation;
  private final DoubleSupplier m_speedScale;
  private final BooleanSupplier m_fieldRelative;

  // Ramp the inputs: going 0 to full stick instantly pulls a huge current spike (brownouts).
  private final SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(OIConstants.kTranslationSlewRate);
  private final SlewRateLimiter m_leftLimiter = new SlewRateLimiter(OIConstants.kTranslationSlewRate);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(OIConstants.kRotationSlewRate);

  /**
   * @param forward forward input, -1..1 (positive = forward)
   * @param left strafe input, -1..1 (positive = left)
   * @param ccwRotation rotation input, -1..1 (positive = counter-clockwise)
   * @param speedScale 0..1 multiplier on all three (the driver's speed slider)
   * @param fieldRelative whether to drive relative to the driver's "forward"
   */
  public DriveCommand(DriveSubsystem drive, DoubleSupplier forward, DoubleSupplier left,
      DoubleSupplier ccwRotation, DoubleSupplier speedScale, BooleanSupplier fieldRelative) {
    m_drive = drive;
    m_forward = forward;
    m_left = left;
    m_ccwRotation = ccwRotation;
    m_speedScale = speedScale;
    m_fieldRelative = fieldRelative;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Start from rest so stale values from before auto can't jerk the robot.
    m_forwardLimiter.reset(0);
    m_leftLimiter.reset(0);
    m_rotLimiter.reset(0);
  }

  @Override
  public void execute() {
    double scale = m_speedScale.getAsDouble();
    double forward = m_forwardLimiter.calculate(
        MathUtil.applyDeadband(m_forward.getAsDouble(), OIConstants.kDriveDeadband)) * scale;
    double left = m_leftLimiter.calculate(
        MathUtil.applyDeadband(m_left.getAsDouble(), OIConstants.kDriveDeadband)) * scale;
    double rot = m_rotLimiter.calculate(
        MathUtil.applyDeadband(m_ccwRotation.getAsDouble(), OIConstants.kDriveDeadband)) * scale;

    // What actually reaches the drivetrain (raw stick values are in the DS log).
    RobotDiagnostics.recordDriveInput(forward, left, rot, scale);
    m_drive.drive(forward, left, rot, m_fieldRelative.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
}
