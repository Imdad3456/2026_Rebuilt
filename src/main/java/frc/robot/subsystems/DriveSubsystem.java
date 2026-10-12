// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.epilogue.Logged;
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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.NeoSwerveModuleConstants;
import frc.robot.util.diagnostics.RobotDiagnostics;

/** The swerve drivetrain: four MAXSwerve modules, a navX gyro, and odometry. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class DriveSubsystem extends SubsystemBase {
  private static final String[] kModuleNames = {"FrontLeft", "FrontRight", "RearLeft", "RearRight"};

  // Order matters everywhere below: FL, FR, RL, RR (matches DriveConstants.kDriveKinematics).
  private final MAXSwerveModule[] m_modules = {
      new MAXSwerveModule(CanIdConstants.kFrontLeftDrivingCanId,
          CanIdConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset),
      new MAXSwerveModule(CanIdConstants.kFrontRightDrivingCanId,
          CanIdConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset),
      new MAXSwerveModule(CanIdConstants.kRearLeftDrivingCanId,
          CanIdConstants.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset),
      new MAXSwerveModule(CanIdConstants.kRearRightDrivingCanId,
          CanIdConstants.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset)};

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final SwerveDriveOdometry m_odometry;

  /**
   * Gyro reading that the driver considers "forward" (away from the driver station). Field-relative
   * driving is relative to this. It is kept in software instead of calling m_gyro.reset(): the
   * navX applies a reset asynchronously, so readings right after it can still be the old angle,
   * which makes the field-relative heading jump.
   */
  private Rotation2d m_driverForward = Rotation2d.kZero;

  private boolean m_gyroWasConnected = true;

  private final DoubleLogEntry[] m_cmdAngleLog = new DoubleLogEntry[4];
  private final DoubleLogEntry[] m_measAngleLog = new DoubleLogEntry[4];
  private final DoubleLogEntry[] m_cmdSpeedLog = new DoubleLogEntry[4];
  private final DoubleLogEntry[] m_measSpeedLog = new DoubleLogEntry[4];
  private int m_logCounter = 0;

  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getGyroRotation(), getModulePositions());

    DataLog log = DataLogManager.getLog();
    for (int i = 0; i < kModuleNames.length; i++) {
      String prefix = "/Diag/Drive/" + kModuleNames[i] + "/";
      m_cmdAngleLog[i] = new DoubleLogEntry(log, prefix + "CommandedAngleRad");
      m_measAngleLog[i] = new DoubleLogEntry(log, prefix + "MeasuredAngleRad");
      m_cmdSpeedLog[i] = new DoubleLogEntry(log, prefix + "CommandedSpeedMps");
      m_measSpeedLog[i] = new DoubleLogEntry(log, prefix + "MeasuredSpeedMps");
    }

    configurePathPlanner();
  }

  /**
   * Set up PathPlanner. Needs the robot settings saved from the PathPlanner app
   * (deploy/pathplanner/settings.json). If they're missing, path autos are unavailable
   * but the robot still runs.
   */
  private void configurePathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportWarning("PathPlanner robot settings not found (open the PathPlanner app, "
          + "set the robot config, and save). Path autos disabled: " + e.getMessage(), false);
      return;
    }
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.kPXController, 0, 0),
            new PIDConstants(AutoConstants.kPThetaController, 0, 0)),
        config,
        // Paths are drawn for the blue alliance; mirror them when we are red.
        () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red,
        this);
  }

  @Override
  public void periodic() {
    m_odometry.update(getGyroRotation(), getModulePositions());
    logModules();

    if (DriverStation.isDisabled()) {
      if (DriveConstants.kDriveP.hasChanged() | DriveConstants.kDriveI.hasChanged()
          | DriveConstants.kDriveD.hasChanged() | DriveConstants.kDriveV.hasChanged()
          | DriveConstants.kDriveS.hasChanged()) {
        for (MAXSwerveModule module : m_modules) {
          module.setDriveGains(DriveConstants.kDriveP.get(), DriveConstants.kDriveI.get(),
              DriveConstants.kDriveD.get(), DriveConstants.kDriveV.get(), DriveConstants.kDriveS.get());
        }
      }
      if (DriveConstants.kTurnP.hasChanged() | DriveConstants.kTurnI.hasChanged()
          | DriveConstants.kTurnD.hasChanged() | DriveConstants.kTurnV.hasChanged()
          | DriveConstants.kTurnS.hasChanged()) {
        for (MAXSwerveModule module : m_modules) {
          module.setTurnGains(DriveConstants.kTurnP.get(), DriveConstants.kTurnI.get(),
              DriveConstants.kTurnD.get(), DriveConstants.kTurnV.get(), DriveConstants.kTurnS.get());
        }
      }
    }
  }

  /**
   * Drive from joystick-style inputs.
   *
   * @param xSpeed forward, -1..1 of max speed
   * @param ySpeed left, -1..1 of max speed
   * @param rot counter-clockwise, -1..1 of max angular speed
   * @param fieldRelative relative to the driver's "forward" (falls back to robot-relative if the
   *     gyro is disconnected, so a dead gyro can't send the robot the wrong way)
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xMps = xSpeed * DriveConstants.kAdjustedMaxSpeedMbpsTeleOp;
    double yMps = ySpeed * DriveConstants.kAdjustedMaxSpeedMbpsTeleOp;
    double rotRadPerSec = rot * DriveConstants.kMaxAngularSpeed.get();

    // No input: stop, but keep each wheel pointing where it is. Otherwise every module snaps
    // back to straight ahead each time the stick is released.
    if (xSpeed == 0 && ySpeed == 0 && rot == 0) {
      for (MAXSwerveModule module : m_modules) {
        module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
      }
      return;
    }

    ChassisSpeeds speeds = fieldRelative && isGyroConnected()
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xMps, yMps, rotRadPerSec, getDriverHeading())
        : new ChassisSpeeds(xMps, yMps, rotRadPerSec);
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds),
        DriveConstants.kAdjustedMaxSpeedMbpsTeleOp);
  }

  /** Drive at robot-relative chassis speeds (used by PathPlanner). */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds),
        NeoSwerveModuleConstants.kMaxSpeedMetersPerSecond);
  }

  public void stop() {
    driveRobotRelative(new ChassisSpeeds());
  }

  /** Point the wheels in an X so the robot resists being pushed. */
  public void setX() {
    m_modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(DriveConstants.kFrontLeftXMode)));
    m_modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(DriveConstants.kFrontRightXMode)));
    m_modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(DriveConstants.kBackLeftXMode)));
    m_modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(DriveConstants.kBackRightXMode)));
  }

  private void setModuleStates(SwerveModuleState[] states, double maxSpeedMps) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMps);
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(states[i]);
    }
  }

  /** Make the robot's current facing the driver's "forward" for field-relative driving. */
  public void zeroDriverHeading() {
    m_driverForward = getGyroRotation();
    RobotDiagnostics.logEvent("DRIVER HEADING ZEROED (gyro " + m_driverForward.getDegrees() + " deg)");
  }

  /** Heading relative to the driver's "forward", counter-clockwise positive. */
  @Logged(name = "DriverHeadingDegrees")
  public double getDriverHeadingDegrees() {
    return getDriverHeading().getDegrees();
  }

  private Rotation2d getDriverHeading() {
    return getGyroRotation().minus(m_driverForward);
  }

  /** Robot heading, counter-clockwise positive (navX getAngle() is clockwise positive). */
  private Rotation2d getGyroRotation() {
    return m_gyro.getRotation2d();
  }

  @Logged(name = "GyroConnected")
  public boolean isGyroConnected() {
    boolean connected = m_gyro.isConnected();
    if (connected != m_gyroWasConnected) {
      RobotDiagnostics.logEvent(connected ? "GYRO RECONNECTED"
          : "GYRO DISCONNECTED - field-relative disabled, driving robot-relative");
      m_gyroWasConnected = connected;
    }
    return connected;
  }

  @Logged(name = "Pose")
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getGyroRotation(), getModulePositions(), pose);
  }

  /** Robot-relative chassis speeds measured from the modules. */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  @Logged(name = "ModuleStates")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  public void resetEncoders() {
    for (MAXSwerveModule module : m_modules) {
      module.resetEncoders();
    }
  }

  /** Commanded vs measured state per module at 10 Hz; shows a steering fault immediately. */
  private void logModules() {
    if (++m_logCounter % 5 != 0) {
      return;
    }
    for (int i = 0; i < m_modules.length; i++) {
      m_cmdAngleLog[i].update(m_modules[i].getCommandedAngle());
      m_measAngleLog[i].update(m_modules[i].getMeasuredAngle());
      m_cmdSpeedLog[i].update(m_modules[i].getCommandedSpeed());
      m_measSpeedLog[i].update(m_modules[i].getMeasuredSpeed());
    }
  }
}
