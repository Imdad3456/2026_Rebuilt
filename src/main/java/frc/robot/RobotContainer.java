// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

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
  // Climber not installed yet: private final ClimberSubsystem m_climber = new ClimberSubsystem();

  private final CommandJoystick m_driver = new CommandJoystick(OIConstants.kDriverJoystickPort);
  private final CommandJoystick m_operator = new CommandJoystick(OIConstants.kOperatorJoystickPort);

  private boolean m_fieldRelative = true;

  private final DriveCommand m_driveCommand;
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    // Stick: forward is -Y, left is -X, counter-clockwise is -Z (twist).
    m_driveCommand = new DriveCommand(
        m_drive,
        () -> -m_driver.getY(),
        () -> -m_driver.getX(),
        () -> -m_driver.getZ(),
        this::getDriveSpeedScale,
        () -> m_fieldRelative);
    m_drive.setDefaultCommand(m_driveCommand);

    setFieldRelative(true);
    configureBindings();

    // PathPlanner autos (if its settings are deployed) plus the hand-written ones.
    m_autoChooser = AutoBuilder.isConfigured() ? AutoBuilder.buildAutoChooser() : new SendableChooser<>();
    m_autoChooser.setDefaultOption("Do Nothing", Commands.none().withName("Do Nothing"));
    m_autoChooser.addOption("Simple Auto", Autos.driveStraight(m_drive));
    SmartDashboard.putData("Auto choices", m_autoChooser);
  }

  private void configureBindings() {
    // ---- Driver ----
    m_driver.button(OIConstants.kZeroHeadingButton)
        .onTrue(Commands.runOnce(m_drive::zeroDriverHeading).ignoringDisable(true).withName("ZeroHeading"));
    m_driver.button(OIConstants.kFieldRelativeToggleButton)
        .onTrue(Commands.runOnce(() -> setFieldRelative(!m_fieldRelative)).ignoringDisable(true)
            .withName("ToggleFieldRelative"));

    // ---- Operator ----
    // Intake in, speed from the intake slider. The intake motor also runs the floor conveyor.
    m_operator.button(OIConstants.kIntakeReceiveButton).whileTrue(m_intake.receive(this::getIntakeSpeed));

    // Eject backwards. Reversing the feeder too makes this the unjam button.
    m_operator.button(OIConstants.kIntakeEjectButton)
        .whileTrue(m_intake.eject(this::getIntakeSpeed).alongWith(m_feeder.reverse()));

    m_operator.button(OIConstants.kExtenderOutButton).onTrue(m_extender.extend());
    m_operator.button(OIConstants.kExtenderInButton).onTrue(m_intake.stop().alongWith(m_extender.retract()));
    m_operator.button(OIConstants.kExtenderHomeButton).onTrue(m_extender.home());

    // Shoot: spin up to the shooter-slider RPM, and only feed once at speed. Feeding into a
    // flywheel that isn't up to speed is what jammed the loader.
    m_operator.button(OIConstants.kLauncherButton)
        .whileTrue(m_launcher.spinUp(this::getShooterRPM)
            .alongWith(Commands.waitUntil(m_launcher::atSpeed).andThen(m_feeder.feed())));

    m_operator.button(OIConstants.kLauncherIdleOnButton).onTrue(m_launcher.idle());
    m_operator.button(OIConstants.kLauncherIdleOffButton).onTrue(m_launcher.off());
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

  private void setFieldRelative(boolean fieldRelative) {
    m_fieldRelative = fieldRelative;
    SmartDashboard.putBoolean("Drive/Field Relative", fieldRelative);
  }

  @Logged(name = "FieldRelative")
  public boolean isFieldRelative() {
    return m_fieldRelative;
  }

  /** Zero field-relative "forward" to the robot's current facing. */
  public void zeroDriverHeading() {
    m_drive.zeroDriverHeading();
  }

  /** The drivetrain's default (teleop) drive command. */
  public Command getDriveCommand() {
    return m_driveCommand;
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
