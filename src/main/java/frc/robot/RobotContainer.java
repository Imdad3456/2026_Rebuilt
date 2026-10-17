// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ExtenderConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.util.DriverAlerts;
import frc.robot.util.diagnostics.RobotDiagnostics;

/** Creates the subsystems and connects the controllers to them. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
  @Logged(name = "Drive")
  private final DriveSubsystem m_drive = new DriveSubsystem();
  @Logged(name = "Launcher")
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  @Logged(name = "Intake")
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  @Logged(name = "Extender")
  private final ExtenderSubsystem m_extender = new ExtenderSubsystem();
  @Logged(name = "Feeder")
  private final FeederSubsystem m_feeder = new FeederSubsystem();

  private final CommandJoystick m_driver = new CommandJoystick(OIConstants.kDriverJoystickPort);
  private final CommandJoystick m_operator = new CommandJoystick(OIConstants.kOperatorJoystickPort);

  /** Plain-language warnings and pop-ups for the drive team (Elastic). */
  private final DriverAlerts m_alerts = new DriverAlerts(m_extender);

  private final DriveCommand m_driveCommand;
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    // Stick: forward is -Y, left is -X, counter-clockwise is -Z (twist).
    m_driveCommand = new DriveCommand(
        m_drive,
        () -> -m_driver.getY(),
        () -> -m_driver.getX(),
        () -> -m_driver.getZ(),
        this::getDriveSpeedScale);
    m_drive.setDefaultCommand(m_driveCommand);

    configureBindings();
    registerNamedCommands();

    // PathPlanner autos (if its settings are deployed) plus the hand-written ones.
    m_autoChooser = AutoBuilder.isConfigured() ? AutoBuilder.buildAutoChooser() : new SendableChooser<>();
    m_autoChooser.setDefaultOption("Extend and Shoot", extendAndShoot());
    m_autoChooser.addOption("Do Nothing", Commands.none().withName("Do Nothing"));
    m_autoChooser.addOption("Simple Auto", Autos.driveStraight(m_drive));
    SmartDashboard.putData("Auto choices", m_autoChooser);
  }

  private void configureBindings() {
    // ---- Driver ---- (no gyro, so no heading or field-relative buttons)
    // Hold to lock the wheels in an X so the robot is hard to push (defense, or holding still to shoot).
    m_driver.button(OIConstants.kXLockButton).whileTrue(m_drive.run(m_drive::setX).withName("XLock"));

    // ---- Operator ---- (March 2026 layout)
    // Intake in, speed from the intake slider. Intake motor only: the loader (which also runs
    // the live floor) isn't needed while intaking.
    m_operator.button(OIConstants.kIntakeReceiveButton).whileTrue(m_intake.receive(this::getIntakeSpeed));

    // Eject backwards. Reversing the feeder too makes this the unjam button.
    m_operator.button(OIConstants.kIntakeEjectButton)
        .whileTrue(m_intake.eject(this::getIntakeSpeed).alongWith(m_feeder.reverse()));

    m_operator.button(OIConstants.kExtenderOutButton).onTrue(m_extender.extend());
    m_operator.button(OIConstants.kExtenderInButton).onTrue(m_intake.stop().alongWith(m_extender.retract()));
    m_operator.button(OIConstants.kExtenderManualButton)
        .whileTrue(m_extender.manual(() -> m_operator.getRawAxis(OIConstants.kExtenderManualAxis)));
    m_operator.button(OIConstants.kExtenderHomeButton).onTrue(m_extender.home());
    m_operator.button(OIConstants.kClearExtenderFaultsButton).onTrue(m_extender.clearFaults());

    // Shoot: spin up to the shooter-slider RPM and feed only while the flywheel is at speed.
    // After each ball the RPM dips; the feeder pauses until it recovers, so every ball leaves at
    // the same speed and nothing gets fed into a slowed flywheel (which is what jammed it).
    // The box is left alone while shooting: the shake popped a sprocket and tripped the skew
    // safety mid-shot in Q16 at NCFAY, freezing the box partway out and jamming the balls.
    m_operator.button(OIConstants.kLauncherButton).whileTrue(shoot(this::getShooterRPM));

    m_operator.button(OIConstants.kLauncherIdleOnButton).onTrue(m_launcher.idle());
    m_operator.button(OIConstants.kLauncherIdleOffButton).onTrue(m_launcher.off());

    m_operator.button(OIConstants.kStopAllButton).onTrue(stopAll());
  }

  /**
   * Stop every mechanism: interrupts whatever is running on each subsystem. The launcher stops
   * (idle off), intake and feeder stop, the box holds where it is, and the drivetrain stops until
   * the driver moves the stick again.
   */
  private Command stopAll() {
    return Commands.parallel(
        m_launcher.off(), m_intake.stop(), m_feeder.stop(), m_extender.holdPosition(),
        m_drive.runOnce(m_drive::stop))
        .withName("StopAll");
  }

  /**
   * Spin the launcher to {@code rpm} and feed only while it's ready (at speed and steady).
   * Runs until interrupted.
   */
  private Command shoot(DoubleSupplier rpm) {
    return m_launcher.spinUp(rpm).alongWith(m_feeder.feedWhile(m_launcher::isReadyToFeed))
        .withName("Shoot");
  }

  /**
   * The default auto: no driving. Extend the box while the flywheel spins up, then shoot the
   * preloaded balls (feeding only while at speed; the box stays fully out), then stop
   * with the box out, ready to intake in teleop.
   *
   * <p>Never shoots with the box in: the balls jam. If the box's safety trips on the way out (Q2
   * and Q8 at NCFAY; homing fixed it both times), it re-homes and goes out again first. If it still
   * isn't out, the auto only spins the flywheel and does not feed.
   */
  private Command extendAndShoot() {
    Command waitForBox = Commands.waitUntil(() -> m_extender.isAtTarget() || m_extender.isFaulted())
        .withTimeout(AutoConstants.kExtendWaitSeconds);
    Command recoverBox = m_extender.home().andThen(m_extender.extend())
        .andThen(Commands.waitUntil(m_extender::isAtTarget).withTimeout(AutoConstants.kExtendWaitSeconds));
    Command shoot = m_feeder.feedWhile(m_launcher::isReadyToFeed)
        .withTimeout(AutoConstants.kPreloadShootSeconds);
    return Commands.deadline(
        Commands.sequence(
            m_extender.extend(),
            waitForBox,
            Commands.either(recoverBox, Commands.none(), () -> m_extender.isFaulted() || !m_extender.isHomed()),
            Commands.either(shoot,
                Commands.runOnce(() -> RobotDiagnostics.logEvent("AUTO: box not out - not shooting (balls would jam)")),
                this::isBoxOutForShooting)),
        m_launcher.spinUp(AutoConstants.kAutoShotRPM::get))
        .withName("Extend and Shoot");
  }

  /** The box is far enough out to shoot without jamming. */
  private boolean isBoxOutForShooting() {
    return !m_extender.isFaulted() && m_extender.isHomed()
        && m_extender.getPositionInches() >= Units.metersToInches(ExtenderConstants.kMinShootPosition) - 0.25;
  }

  /**
   * Commands PathPlanner autos can use by name (event markers, or command blocks in an .auto).
   * Must be registered before any auto is built. "Intake" and "SpinUp" run until the path or
   * group ends them; "Shoot" ends on its own after {@link AutoConstants#kShootSeconds}.
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", m_intake.receive(() -> IntakeConstants.kAutoSpeed));
    NamedCommands.registerCommand("IntakeStop", m_intake.stop());
    NamedCommands.registerCommand("SpinUp", m_launcher.spinUp(AutoConstants.kAutoShotRPM::get));
    Command autoShoot = shoot(AutoConstants.kAutoShotRPM::get)
        .withTimeout(AutoConstants.kShootSeconds);
    NamedCommands.registerCommand("Shoot", autoShoot);
    // Names used by the March 2026 autos (TestAuto).
    NamedCommands.registerCommand("autoLaunch", autoShoot);
    NamedCommands.registerCommand("autoIntake", m_intake.receive(() -> IntakeConstants.kAutoSpeed));
    NamedCommands.registerCommand("ExtenderOut", m_extender.extend());
    NamedCommands.registerCommand("ExtenderIn", m_extender.retract());
  }

  /** Driver slider: fraction of max drive speed, from kMinDriveSpeedScale to 1. */
  @Logged(name = "DriveSpeedScale")
  public double getDriveSpeedScale() {
    double percent = OIConstants.sliderToPercent(
        m_driver.getRawAxis(OIConstants.kDriveSpeedSliderAxis), OIConstants.kDriveSpeedSliderInverted);
    return OIConstants.kMinDriveSpeedScale + (1.0 - OIConstants.kMinDriveSpeedScale) * percent;
  }

  /** Operator intake slider: intake duty cycle. */
  @Logged(name = "IntakeSpeed")
  public double getIntakeSpeed() {
    double percent = OIConstants.sliderToPercent(
        m_operator.getRawAxis(OIConstants.kIntakeSpeedSliderAxis), OIConstants.kIntakeSpeedSliderInverted);
    return IntakeConstants.kMinSpeed + (IntakeConstants.kMaxSpeed - IntakeConstants.kMinSpeed) * percent;
  }

  /** Operator shooter slider: shot RPM. */
  @Logged(name = "ShooterRPM")
  public double getShooterRPM() {
    return LauncherSubsystem.rpmFromSlider(OIConstants.sliderToPercent(
        m_operator.getRawAxis(OIConstants.kShooterSpeedSliderAxis), OIConstants.kShooterSpeedSliderInverted));
  }

  /**
   * Operator-facing values for the Elastic dashboard (deploy/elastic-layout.json). Called at
   * 10 Hz. Only what the drive team needs; detailed telemetry goes to the log file.
   */
  public void updateDashboard() {
    SmartDashboard.putNumber("Robot/Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Robot/Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Launcher/Actual RPM", Math.round(m_launcher.getActualRPM()));
    SmartDashboard.putNumber("Operator/Shooter Slider RPM", Math.round(getShooterRPM()));
    SmartDashboard.putNumber("Operator/Intake Speed %", Math.round(getIntakeSpeed() * 100));
    SmartDashboard.putNumber("Driver/Drive Speed %", Math.round(getDriveSpeedScale() * 100));
    SmartDashboard.putNumber("Extender/Position (in)", Math.round(m_extender.getPositionInches() * 10) / 10.0);

    Command auto = m_autoChooser.getSelected();
    m_alerts.update(auto == null ? "Do Nothing" : auto.getName());
  }

  /** For simulation tests only. */
  ExtenderSubsystem getExtenderForTest() {
    return m_extender;
  }

  /** For simulation tests only. */
  LauncherSubsystem getLauncherForTest() {
    return m_launcher;
  }

  /** The drivetrain's default (teleop) drive command. */
  public Command getDriveCommand() {
    return m_driveCommand;
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
