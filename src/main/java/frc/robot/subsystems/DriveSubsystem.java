// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;

/**
 * The DriveSubsystem manages the robot's swerve drive train.
 * It controls the four individual swerve modules, integrates gyroscope data for field-relative driving,
 * and uses odometry to track the robot's position on the field.
 */
public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  /** Front-left swerve module. */
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      CanIdConstants.kFrontLeftDrivingCanId,
      CanIdConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  /** Front-right swerve module. */
  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      CanIdConstants.kFrontRightDrivingCanId,
      CanIdConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  /** Rear-left swerve module. */
  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      CanIdConstants.kRearLeftDrivingCanId,
      CanIdConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  /** Rear-right swerve module. */
  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      CanIdConstants.kRearRightDrivingCanId,
      CanIdConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  /** The gyroscope sensor for measuring robot heading. */
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private static final String[] kModuleNames = {"FrontLeft", "FrontRight", "RearLeft", "RearRight"};
  private final BooleanLogEntry m_gyroConnectedLog;
  private final DoubleLogEntry[] m_cmdAngleLog = new DoubleLogEntry[4];
  private final DoubleLogEntry[] m_measAngleLog = new DoubleLogEntry[4];
  private final DoubleLogEntry[] m_cmdSpeedLog = new DoubleLogEntry[4];
  private final DoubleLogEntry[] m_measSpeedLog = new DoubleLogEntry[4];
  private int m_logCounter = 0;

  /** Odometry class for tracking robot pose on the field. */
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      getGyroRotation(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /**
   * Constructs a new DriveSubsystem.
   * Initializes swerve modules, gyroscope, and odometry.
   */
public DriveSubsystem() {
  // Report usage of the MAXSwerve template to WPILib for analytics.
  HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

  DataLog log = DataLogManager.getLog();
  m_gyroConnectedLog = new BooleanLogEntry(log, "/Diag/Drive/GyroConnected");
  for (int i = 0; i < kModuleNames.length; i++) {
    String prefix = "/Diag/Drive/" + kModuleNames[i] + "/";
    m_cmdAngleLog[i] = new DoubleLogEntry(log, prefix + "CommandedAngleRad");
    m_measAngleLog[i] = new DoubleLogEntry(log, prefix + "MeasuredAngleRad");
    m_cmdSpeedLog[i] = new DoubleLogEntry(log, prefix + "CommandedSpeedMps");
    m_measSpeedLog[i] = new DoubleLogEntry(log, prefix + "MeasuredSpeedMps");
  }
}

/**
 * The robot heading, counter-clockwise positive as WPILib expects. The navX's
 * getAngle() is clockwise positive, so using it directly mirrored the heading.
 */
private Rotation2d getGyroRotation() {
  return m_gyro.getRotation2d();
}

/**
 * Log commanded vs measured state for each module at 10 Hz. If the steering can't
 * follow its command (e.g. wrong config/encoder), the angles visibly disagree here.
 */
private void logModules() {
  if (++m_logCounter % 5 != 0) {
    return;
  }
  m_gyroConnectedLog.update(m_gyro.isConnected());
  MAXSwerveModule[] modules = {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};
  for (int i = 0; i < modules.length; i++) {
    m_cmdAngleLog[i].update(modules[i].getCommandedAngle());
    m_measAngleLog[i].update(modules[i].getMeasuredAngle());
    m_cmdSpeedLog[i].update(modules[i].getCommandedSpeed());
    m_measSpeedLog[i].update(modules[i].getMeasuredSpeed());
  }
}

@Override
public void periodic() {
    logModules();

    // Update the odometry in the periodic block
    m_odometry.update(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    if (DriverStation.isDisabled()) {
      if (DriveConstants.kDriveP.hasChanged() || DriveConstants.kDriveI.hasChanged() || DriveConstants.kDriveD.hasChanged() 
            || DriveConstants.kDriveV.hasChanged() || DriveConstants.kDriveS.hasChanged()) {
        this.m_frontRight.setDrivePID(DriveConstants.kDriveP.get(), DriveConstants.kDriveI.get(), DriveConstants.kDriveD.get(),
            DriveConstants.kDriveV.get(), DriveConstants.kDriveS.get());
        this.m_frontLeft.setDrivePID(DriveConstants.kDriveP.get(), DriveConstants.kDriveI.get(), DriveConstants.kDriveD.get(),
            DriveConstants.kDriveV.get(), DriveConstants.kDriveS.get());
        this.m_rearRight.setDrivePID(DriveConstants.kDriveP.get(), DriveConstants.kDriveI.get(), DriveConstants.kDriveD.get(),
            DriveConstants.kDriveV.get(), DriveConstants.kDriveS.get());
        this.m_rearLeft.setDrivePID(DriveConstants.kDriveP.get(), DriveConstants.kDriveI.get(), DriveConstants.kDriveD.get(),
            DriveConstants.kDriveV.get(), DriveConstants.kDriveS.get());
      }
      if (DriveConstants.kTurnP.hasChanged() || DriveConstants.kTurnI.hasChanged() || DriveConstants.kTurnD.hasChanged()
            || DriveConstants.kTurnV.hasChanged() || DriveConstants.kTurnS.hasChanged()){
        this.m_frontRight.setTurnPID(DriveConstants.kTurnP.get(), DriveConstants.kTurnI.get(), DriveConstants.kTurnD.get(),
            DriveConstants.kTurnV.get(), DriveConstants.kTurnS.get());
        this.m_frontLeft.setTurnPID(DriveConstants.kTurnP.get(), DriveConstants.kTurnI.get(), DriveConstants.kTurnD.get(),
            DriveConstants.kTurnV.get(), DriveConstants.kTurnS.get());
        this.m_rearRight.setTurnPID(DriveConstants.kTurnP.get(), DriveConstants.kTurnI.get(), DriveConstants.kTurnD.get(),
            DriveConstants.kTurnV.get(), DriveConstants.kTurnS.get());
        this.m_rearLeft.setTurnPID(DriveConstants.kTurnP.get(), DriveConstants.kTurnI.get(), DriveConstants.kTurnD.get(),
            DriveConstants.kTurnV.get(), DriveConstants.kTurnS.get());
      }
    }
}


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose (position and heading) of the robot on the field.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   * This is typically used at the start of autonomous or after an event
   * that requires recalibrating the robot's known position.
   *
   * @param pose The pose (position and heading) to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick input.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward/backward).
   *                      A positive value moves the robot forward.
   * @param ySpeed        Speed of the robot in the y direction (sideways/strafe).
   *                      A positive value moves the robot left.
   * @param rot           Angular rate of the robot. A positive value rotates counter-clockwise.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field (true) or to the robot's current orientation (false).
   */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.kAdjustedMaxSpeedMbpsTeleOp;
        double ySpeedDelivered = ySpeed * DriveConstants.kAdjustedMaxSpeedMbpsTeleOp;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed.get();

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                    getGyroRotation())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
        // Desaturate wheel speeds to ensure no wheel exceeds the maximum allowed speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kAdjustedMaxSpeedMbpsTeleOp);
        
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }


  /**
   * Sets the swerve modules into an "X" formation.
   * In this mode, the wheels are angled such that the robot resists movement,
   * effectively braking the robot.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(DriveConstants.kFrontLeftXMode)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(DriveConstants.kFrontRightXMode)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(DriveConstants.kBackLeftXMode)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(DriveConstants.kBackRightXMode)));
  }

  /**
   * Sets the desired states for each individual swerve module.
   * This method is typically used by autonomous routines or advanced control algorithms.
   *
   * @param desiredStates An array of {@link SwerveModuleState} objects, one for each module,
   *                      specifying the desired speed and angle for that module.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Desaturate wheel speeds to ensure no wheel exceeds the maximum allowed speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kAdjustedMaxSpeedMbpsTeleOp);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the position encoders of all swerve modules to read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot's gyroscope, effectively setting the current orientation as 0 degrees. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the current heading of the robot as measured by the gyroscope.
   *
   * @return The robot's heading in degrees, ranging from -180 to 180.
   */
  public double getHeading() {
    return getGyroRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second.
   */
  public double getTurnRate() {
    // navX getRate() is clockwise positive; negate so this matches the CCW-positive heading.
    // Apply a further reversal if kGyroReversed is true.
    return -m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the current chassis speeds (translational and rotational velocities) of the robot.
   * These are derived from the states of the individual swerve modules.
   *
   * @return A {@link ChassisSpeeds} object representing the robot's current velocities.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }
}
