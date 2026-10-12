// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ClimberExtend;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExtenderIn;
import frc.robot.commands.ExtenderOut;
import frc.robot.commands.FeederOff;
import frc.robot.commands.FeederOn;
import frc.robot.commands.HomeClimber;
import frc.robot.commands.HomeExtender;
import frc.robot.commands.IntakeEject;
import frc.robot.commands.IntakeReceive;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.RollerForward;
import frc.robot.commands.RollerOff;
import frc.robot.commands.RollerReverse;
import frc.robot.commands.LauncherIdle;
import frc.robot.commands.LauncherOff;
import frc.robot.commands.LauncherOn;
import frc.robot.commands.ClimberRetract;
import frc.robot.configs.ClimberConfigs;
import frc.robot.configs.ExtenderConfigs;
import frc.robot.configs.FeederConfigs;
import frc.robot.configs.IntakeConfigs;
import frc.robot.configs.LauncherConfigs;
import frc.robot.configs.RollerConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ExtenderConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.GlobalConstants;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Rollers;
import frc.robot.util.hardware.HardwareFactory;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * The `RobotContainer` is the central place where the robot's subsystems,
 * operator interface (OI) devices, and commands are instantiated and
 * connected. It's responsible for defining the robot's overall behavior,
 * including default commands, autonomous routines, and button bindings.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  /** The robot's drive subsystem, controlling movement. */
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  /** The robot's launcher subsystem, responsible for shooting game pieces. */
  private final LauncherSubsystem m_Launcher = new LauncherSubsystem(HardwareFactory.createSparkMax(CanIdConstants.kLauncherLeaderMotor, LauncherConfigs.config));
  /** The robot's intake subsystem, for acquiring game pieces. */
  private final IntakeSubsystem m_Intake = new IntakeSubsystem(HardwareFactory.createSparkMax(CanIdConstants.kIntakeMotor, IntakeConfigs.config));
  /** The robot's extender subsystem, for extending and retracting mechanisms. */
  private final ExtenderSubsystem m_Extender = new ExtenderSubsystem(HardwareFactory.createSparkMax(CanIdConstants.kExtenderMotor1, ExtenderConfigs.leaderConfig), 
                            HardwareFactory.createSparkMax(CanIdConstants.kExtenderMotor2,ExtenderConfigs.followConfig));
  /** The robot's roller subsystem, for manipulating game pieces within the robot. */
  private final Rollers m_Rollers = new Rollers(HardwareFactory.createSparkMax(CanIdConstants.kRollerMotor, RollerConfigs.config));
  /** The robot's feeder subsystem, for transferring game pieces to the launcher. */
  private final FeederSubsystem m_Feeder = new FeederSubsystem(HardwareFactory.createSparkMax(CanIdConstants.kFeederMotor, FeederConfigs.config));
  /** The robot's climber subsystem, for ascending vertical structures. */
 // private final ClimberSubsystem m_Climber = new ClimberSubsystem(HardwareFactory.createFollowerSparkMax(CanIdConstants.kClimberMotor1, CanIdConstants.kClimberMotor2, ClimberConfigs.leaderConfig, true));

  /** A boolean flag to toggle between field-relative and robot-relative driving. */
  private boolean m_fieldRelative = false;

  // Operator Interface (OI) devices and their button bindings
  /** The joystick used by the driver for robot movement. */
  Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
  /** The joystick used by the operator for controlling mechanisms. */
  Joystick m_operatorJoystick = new Joystick(OIConstants.kOperatorJoystickPort);
  //XboxController m_operatorJoystick = new XboxController(OIConstants.kOperatorJoystickPort);
  
  /** Button for initiating the intake receive action. */
  JoystickButton intakeRecieve = new JoystickButton(m_operatorJoystick, OIConstants.kIntakeReceiveButton);
  /** Button for initiating the intake eject action. */
  JoystickButton intakeEject = new JoystickButton(m_operatorJoystick, OIConstants.kIntakeEjectButton);
  /** Button for launching game pieces. */
  JoystickButton launch = new JoystickButton(m_operatorJoystick, OIConstants.kLauncherButton);
  /** Button for extending a mechanism outwards. */
  JoystickButton extenderOut = new JoystickButton(m_operatorJoystick, OIConstants.kExtenderOutButton);
  /** Button for retracting a mechanism inwards. */
  JoystickButton extenderIn = new JoystickButton(m_operatorJoystick, OIConstants.kExtenderInButton);
  /** Button for turning on the launcher's idle speed. */
  JoystickButton launcherIdleOn = new JoystickButton(m_operatorJoystick, OIConstants.kLauncherIdleOnButton);
  /** Button for turning off the launcher's idle speed. */
  JoystickButton launcherIdleOff = new JoystickButton(m_operatorJoystick, OIConstants.kLauncherIdleOffButton);
  /** Button for homing the extender mechanism. */
  JoystickButton homeExtender = new JoystickButton(m_operatorJoystick, OIConstants.kExtenderHomeButton);
  /** Button for extending the climber mechanism. */
  JoystickButton climbExtend = new JoystickButton(m_operatorJoystick, OIConstants.kClimberExtendButton);
  /** Button for retracting the climber mechanism. */
  JoystickButton climbRetract = new JoystickButton(m_operatorJoystick, OIConstants.kClimberRetractButton);

  /** The drivetrain's default command (kept so diagnostics can check it is running). */
  private final Command m_driveCommand;

  /** A SendableChooser for selecting the autonomous command from the SmartDashboard. */
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * This is where default commands are configured and button-to-command
   * mappings are established.
   */
  public RobotContainer() {
    // 1. Check the serial number the code is actually reading
    System.out.println("System Serial: " + edu.wpi.first.wpilibj.RobotController.getSerialNumber());
    
    // 2. Check the final boolean result
    System.out.println("Is Benchtop Detection: " + GlobalConstants.IS_BENCHTOP);

    SmartDashboard.putBoolean("Is Benchtop?", GlobalConstants.IS_BENCHTOP);
    // Configure default commands for subsystems.
    // The DriveSubsystem's default command allows the driver to control the robot's movement.
    m_driveCommand = new DriveCommand(
        m_robotDrive,
        () -> m_driverJoystick.getY(), // Y-axis for forward/backward
        () -> m_driverJoystick.getX(), // X-axis for strafing
        () -> m_driverJoystick.getZ(), // Z-axis for rotation
        () -> m_fieldRelative);        // Boolean supplier for field-relative control
    m_robotDrive.setDefaultCommand(m_driveCommand);

    // Configure the button bindings and their associated commands.
    configureBindings();

    // Add various autonomous commands to the chooser for selection via SmartDashboard.
    m_autoChooser.setDefaultOption("Do Nothing", Commands.none().withName("Do Nothing")); // Default: no autonomous action
    m_autoChooser.addOption("Simple Auto", Autos.exampleAuto(m_robotDrive).withName("Simple Auto")); // Example autonomous routine

    // Put the autonomous command chooser on the SmartDashboard for driver selection.
    SmartDashboard.putData("Auto choices", m_autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings.
   * This is where you connect joystick buttons to specific robot actions.
   */
  private void configureBindings() {
    // intakeRecieve: When held, runs the intake and rollers to receive a ball.
    intakeRecieve.whileTrue(
        new ParallelCommandGroup(
            new IntakeReceive(m_Intake),
            new RollerForward(m_Rollers)));

    // intakeEject: When held, runs the intake, rollers and feeder in reverse to eject a ball.
    // Reversing the feeder too makes this the "unjam" button for the shooter loader.
    intakeEject.whileTrue(
        new ParallelCommandGroup(
            new IntakeEject(m_Intake),
            new RollerReverse(m_Rollers),
            Commands.startEnd(
                () -> m_Feeder.engageFeeder(-FeederConstants.kFeederSpeed.get()),
                m_Feeder::stopFeeder,
                m_Feeder)));

    // extenderOut: When pressed, extends the extender.
    extenderOut.onTrue(
        new ExtenderOut(m_Extender));

    // extenderIn: When pressed, stops the intake and retracts the extender.
    extenderIn.onTrue(
        new ParallelCommandGroup(
            new IntakeStop(m_Intake),
            new ExtenderIn(m_Extender)));

    // launch: When held, spins up the launcher, then runs the rollers and feeder together.
    // The rollers used to start immediately, packing balls into the stopped feeder while
    // the flywheel spun up, which is what jammed the loader. Now nothing moves balls
    // until the launcher is at speed.
    // (No ball-present gate: there's no hopper sensor yet, and a floating DIO reads true.)
    launch.whileTrue(
        new ParallelCommandGroup(
            new LauncherOn(m_Launcher, () -> m_operatorJoystick.getRawAxis(6)),
            new WaitUntilCommand(m_Launcher::atSpeed).andThen(
                new ParallelCommandGroup(
                    new FeederOn(m_Feeder),
                    new RollerForward(m_Rollers)))));

    // homeExtender: When pressed, homes the extender.
    // The timeout stops it pushing into the hard stop forever if the current
    // threshold is never reached.
    homeExtender.onTrue(
        new HomeExtender(m_Extender).withTimeout(ExtenderConstants.kHomingTimeoutSeconds));

    // launcherIdleOn: When pressed, sets the launcher to idle speed.
    launcherIdleOn.onTrue(
        new LauncherIdle(m_Launcher));

    // launcherIdleOff: When pressed, turns the launcher off.
    launcherIdleOff.onTrue(
        new LauncherOff(m_Launcher));

/* CLIMBER HAS NOT BEEN INSTALLED
        // climbExtend: When held, extends the climber and turns off other systems.
    climbExtend.whileTrue(
      new ParallelCommandGroup(
        new LauncherOff(m_Launcher),
        new RollerOff(m_Rollers),
        new IntakeStop(m_Intake),
        new FeederOff(m_Feeder),
        new ClimberExtend(m_Climber)
        ));

    // climbRetract: When held, retracts the climber and turns off other systems.
    climbRetract.whileTrue(
      new ParallelCommandGroup(
        new LauncherOff(m_Launcher),
        new RollerOff(m_Rollers),
        new IntakeStop(m_Intake),
        new FeederOff(m_Feeder),
        new ClimberRetract(m_Climber)
        ));
        */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * This method retrieves the selected autonomous command from the SmartDashboard
   * chooser and wraps it with a pre-auto routine (e.g., homing the extender).
   *
   * @return The {@link Command} to run in autonomous mode.
   */
  /** The drivetrain's default (teleop) drive command. */
  public Command getDriveCommand() {
    return m_driveCommand;
  }

  public Command getAutonomousCommand() {
    // 1. Get whatever auto the driver picked from the dashboard
    Command selectedAuto = m_autoChooser.getSelected();
    return selectedAuto;
    /*/ 2. Wrap it: Run HomeExtender FIRST, then run the selected auto
    // This ensures the extender is in a known state before autonomous actions begin.
    return Commands.parallel(
        new HomeExtender(m_Extender),
        new HomeClimber(m_Climber)
    ).andThen(selectedAuto);*/
  }
}
