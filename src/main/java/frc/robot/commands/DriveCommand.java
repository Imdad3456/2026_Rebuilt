// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.util.diagnostics.RobotDiagnostics;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The DriveCommand enables continuous teleoperated driving of the swerve drivetrain.
 * It reads joystick inputs for X (forward/backward), Y (strafe), and Rotation,
 * applies a deadband, and commands the DriveSubsystem accordingly.
 * It supports both field-relative and robot-relative driving.
 */
public class DriveCommand extends Command {
  /** The DriveSubsystem instance that this command will control. */
  private final DriveSubsystem m_drive;
  /** Supplier for the X-axis (forward/backward) joystick input. */
  private final DoubleSupplier m_xSupplier;
  /** Supplier for the Y-axis (left/right strafe) joystick input. */
  private final DoubleSupplier m_ySupplier;
  /** Supplier for the rotation (Z-axis) joystick input. */
  private final DoubleSupplier m_rotSupplier;
  /** Supplier for the boolean indicating whether to use field-relative control. */
  private final BooleanSupplier m_fieldRelativeSupplier;

  /**
   * Creates a new DriveCommand.
   * This command is typically set as the default command for the DriveSubsystem,
   * allowing for continuous driver control during teleoperated period.
   *
   * @param drive                 The DriveSubsystem instance to be controlled by this command.
   * @param xSupplier             A DoubleSupplier that provides the desired speed in the X direction (forward/backward).
   * @param ySupplier             A DoubleSupplier that provides the desired speed in the Y direction (left/right strafe).
   * @param rotSupplier           A DoubleSupplier that provides the desired angular velocity (rotation).
   * @param fieldRelativeSupplier A BooleanSupplier that indicates whether the movement
   *                              should be relative to the field (true) or to the robot (false).
   */
  public DriveCommand(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier, BooleanSupplier fieldRelativeSupplier) {
    m_drive = drive;
    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;
    m_rotSupplier = rotSupplier;
    m_fieldRelativeSupplier = fieldRelativeSupplier;

    // Declare that this command requires the DriveSubsystem, ensuring exclusive access.
    addRequirements(drive);
  }

  // No initialize method needed as continuous reading and driving happens in execute().
  // @Override
  // public void initialize() {}

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * This method continuously reads joystick inputs, applies a deadband for precision,
   * and commands the DriveSubsystem to move.
   */
  @Override
  public void execute() {
    // Get joystick inputs and apply deadband to filter out small, unintended movements.
    double xSpeed = -MathUtil.applyDeadband(m_xSupplier.getAsDouble(), OIConstants.kDriveDeadband);
    double ySpeed = -MathUtil.applyDeadband(m_ySupplier.getAsDouble(), OIConstants.kDriveDeadband);
    double rot = -MathUtil.applyDeadband(m_rotSupplier.getAsDouble(), OIConstants.kDriveDeadband);

    // Log what actually reaches the drivetrain (raw stick values are in the DS log).
    RobotDiagnostics.recordDriveInput(xSpeed, ySpeed, rot);

    // Drive the robot using the processed inputs and the current field-relative state.
    m_drive.drive(xSpeed, ySpeed, rot, m_fieldRelativeSupplier.getAsBoolean());
  }

  /**
   * Called once the command ends or is interrupted.
   * Stops the robot's movement to prevent unexpected behavior.
   * @param interrupted True if the command was interrupted by another, false otherwise.
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the drive by commanding zero speeds in all directions.
    m_drive.drive(0, 0, 0, false);
  }

  /**
   * Returns true when the command should end.
   * This command is designed to run continuously during teleoperated control
   * and is typically interrupted by the driver releasing the joystick or a new command
   * taking over the DriveSubsystem.
   * @return Always false, as this command never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false; // This command runs continuously
  }
}