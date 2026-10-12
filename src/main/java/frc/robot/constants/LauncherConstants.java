// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.util.TunableNumber;
import frc.robot.util.TuningManager;

/**
 * The LauncherConstants class stores all constant values related to the robot's launcher mechanism.
 * This includes motor speeds, tolerances, PID gains, sensor IDs, timing parameters,
 * and physical measurements used for Limelight calculations.
 */
public class LauncherConstants {

    /*
     ****** TUNABLE VARIABLES ******
     */

    /** The target RPM for the launcher when it is in an idle state. */
    public static final TunableNumber kLauncherMotorSpeedIdle = TuningManager.register("Launcher/IdleSpeed", 500);

    /** The acceptable tolerance for the launcher's actual RPM to be considered "at speed". */
    public static final TunableNumber kLauncherTolerance = TuningManager.register("Launcher/Tolerance", 50.0);
    
    /** A buffer value added to the idle speed to determine the minimum RPM for a "launch" speed. */
    public static final TunableNumber kLaunchMinShotBuffer = TuningManager.register("Launcher/MinShotBuffer", 500);

    /** The minimum RPM the launcher should achieve for a successful launch. */
    public static final TunableNumber kLaunchMinRPM = TuningManager.register("Launcher/MinRPM", 1500);
    
    /** The maximum safe RPM the launcher can operate at. */
    public static final TunableNumber kLaunchMaxRPM = TuningManager.register("Launcher/MaxRPM", 4500);

    // Velocity Closed-Loop Constants (for variable speed)
    /**
     * Velocity feed-forward gain (kV) for the launcher motor.
     * Calculated as (max voltage / max RPM), e.g., 11.0 / 5676 (Max RPM of NEO).
     */
    public static final TunableNumber kLauncherkV = TuningManager.register("Launcher/kV", .0021);

    public static final TunableNumber kLauncherStatic = TuningManager.register("Launcher/Static", 0.002);
    
    /** Proportional gain for the launcher's velocity PID controller. */
    public static final TunableNumber kLauncherP = TuningManager.register("Launcher/P", 0.00038);
    
    /** Integral gain for the launcher's velocity PID controller. */
    public static final TunableNumber kLauncherI = TuningManager.register("Launcher/I", 0.0);
    
    /** Derivative gain for the launcher's velocity PID controller. */
    public static final TunableNumber kLauncherD = TuningManager.register("Launcher/D", 0.0);

    /** The base RPM for Limelight calculations, used as an offset. */
    public static final TunableNumber kBaseRPM = TuningManager.register("Launcher/BaseRPM", 15);
    
    /** The RPM increase per inch of distance for Limelight calculations. */
    public static final TunableNumber kRPMPerInch = TuningManager.register("Launcher/RPMPerInch", 15);

    /*
     ****** NON TUNABLE VARIABLES ******
     */
    // direct motor - no gears, sprockets, chains etc.
    // measure with direct RPM
    public static final double kPositionFactor = 1.0;
    public static final double kVelocityFactor = 1.0;

    // values of the field components for the limelight calculations
    /** The height of the AprilTag on the field, in inches. */
    public static final double kAprilTagHeight = 44.25;
    /** The height of the Limelight camera from the ground, in inches. */
    public static final double kCameraHeight = 12.0;
    /** The mounting angle of the Limelight camera, in degrees. */
    public static final double kMountAngle = 15.0;
}
