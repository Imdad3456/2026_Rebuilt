package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TuningManager;
import frc.robot.util.TunableNumber;

/**
 * The DriveConstants class stores all constant values related to the robot's drivetrain.
 * This includes speed limits, chassis dimensions, angular offsets for swerve modules,
 * and PID controller gains for driving and turning.
 */
public class DriveConstants {
    
    /*
     ****** TUNABLE VARIABLES ******
     */

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds

    /** Speed adjustment factor for teleoperated driving. Multiplied by theoretical max speed. */
    public static final TunableNumber speedAdjustTeleOp = TuningManager.register("Drive/speedAdjustTeleOp", 0.9);
    /** Speed adjustment factor for autonomous driving. Multiplied by theoretical max speed. */
    public static final TunableNumber speedAdjustAuto = TuningManager.register("Drive/speedAdjustAuto", 0.5);
            
    /** Maximum adjusted speed for teleoperated driving, in meters per second. */
    public static final double kAdjustedMaxSpeedMbpsTeleOp = NeoSwerveModuleConstants.kMaxSpeedMetersPerSecond * speedAdjustTeleOp.get();
    /** Maximum adjusted speed for autonomous driving, in meters per second. */
    public static final double kAdjustedMaxSpeedMbpsAuto = NeoSwerveModuleConstants.kMaxSpeedMetersPerSecond * speedAdjustAuto.get();
    
    /** Maximum angular speed of the robot, in radians per second. */
    public static final TunableNumber kMaxAngularSpeed = TuningManager.register("Drive/kMaxAngularSpeed", 2 * Math.PI); // radians per second
    
    /** Proportional gain for the drive PID controller. */
    public static final TunableNumber kDriveP = TuningManager.register("Drive/kP", 0.04);
    /** Integral gain for the drive PID controller. */
    public static final TunableNumber kDriveI = TuningManager.register("Drive/kI", 0.0);
    /** Derivative gain for the drive PID controller. */
    public static final TunableNumber kDriveD = TuningManager.register("Drive/kD", 0.0);

    /**
     * Drive velocity feedforward in volts per (m/s): full battery voltage at free speed.
     * Same approach as REV's MAXSwerve template.
     */
    public static final TunableNumber kDriveV = TuningManager.register("Drive/kV",
            12.0 / NeoSwerveModuleConstants.kMaxSpeedMetersPerSecond);

    public static final TunableNumber kDriveS = TuningManager.register("Drive/kS", 0.0);

    /** Proportional gain for the turn PID controller. */
    public static final TunableNumber kTurnP = TuningManager.register("Turn/kP", 1.0);
    /** Integral gain for the turn PID controller. */
    public static final TunableNumber kTurnI = TuningManager.register("Turn/kI", 0.0);
    /** Derivative gain for the turn PID controller. */
    public static final TunableNumber kTurnD = TuningManager.register("Turn/kD", 0.0);

    public static final TunableNumber kTurnV = TuningManager.register("Turn/kV", 0.0);

    public static final TunableNumber kTurnS = TuningManager.register("Turn/kS", 0.0);

    /*
     ****** NON TUNABLE VARIABLES ******
     */

    // Chassis configuration
    /** The track width of the robot chassis, measured in meters. */
    public static final double kTrackWidth = Units.inchesToMeters(26);
    /** The wheel base of the robot chassis, measured in meters. */
    public static final double kWheelBase = Units.inchesToMeters(26);

    /**
     * Defines the swerve drive kinematics, specifying the positions of each wheel relative to the robot's center.
     * This is crucial for calculating individual wheel speeds and angles based on desired robot movement.
     */
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    /** Angular offset for the front-left swerve module, in radians. */
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    /** Angular offset for the front-right swerve module, in radians. */
    public static final double kFrontRightChassisAngularOffset = 0;
    /** Angular offset for the back-left swerve module, in radians. */
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    /** Angular offset for the back-right swerve module, in radians. */
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    /** Indicates if the gyroscope is reversed. */
    public static final boolean kGyroReversed = false;

    /** Front-left module angle in X-mode, in degrees. */
    public static final double kFrontLeftXMode = 45.0;
    /** Front-right module angle in X-mode, in degrees. */
    public static final double kFrontRightXMode = -45.0;
    /** Back-left module angle in X-mode, in degrees. */
    public static final double kBackLeftXMode = -45.0;
    /** Back-right module angle in X-mode, in degrees. */
    public static final double kBackRightXMode = 45.0;
}