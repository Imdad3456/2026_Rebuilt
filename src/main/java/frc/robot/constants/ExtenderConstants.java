package frc.robot.constants;

import frc.robot.util.RobotUtils;
import frc.robot.util.TunableNumber;
import frc.robot.util.TuningManager;

/**
 * The ExtenderConstants class centralizes all constant values related to the robot's extender mechanism.
 * This includes motor speeds, extension limits, physical conversion factors (e.g., inches per rotation),
 * PID gains for control, homing parameters, and various operational tolerances.
 */
public class ExtenderConstants {

    /*
     ****** TUNABLE VARIABLES ******
     */

    /** Default speed for the extender motor during operation. */
    public static final TunableNumber kExtenderMotorSpeed = TuningManager.register("Extender/MotorSpeed", 0.5);
    
    /** kV: The voltage required to sustain a given velocity. Units: Volts/(Unit of Speed). */
    public static final TunableNumber kExtenderFF = TuningManager.register("Extender/FF", 0.5);

/** kS: The 'Oomph' required to break static friction. Units: Volts or Percent. */
    public static final TunableNumber kExtenderStatic = TuningManager.register("Extender/Static", 0.5);

    /** Proportional gain for the extender's position PID controller. */
    public static final TunableNumber kExtenderP = TuningManager.register("Extender/P", 0.001);
    
    /** Integral gain for the extender's position PID controller. */
    public static final TunableNumber kExtenderI = TuningManager.register("Extender/I", 0.0);
    
    /** Derivative gain for the extender's position PID controller. */
    public static final TunableNumber kExtenderD = TuningManager.register("Extender/D", 0.0);
    

    public static final TunableNumber kExtAcceleration = TuningManager.register("Extender/Acceleration", 25.0);
    
    /**
     * Allowed error (in inches) for the extender's closed-loop position control.
     * If the extender is within this tolerance of the target, it's considered "at target".
     */
    public static final TunableNumber kExtenderAllowedError = TuningManager.register("Extender/AllowedError", 0.1);

    /**
     * Maximum acceleration for the extender mechanism, likely used in motion profiling or trapezoidal control
     * to ensure smooth and controlled movement.

    /** The voltage used for homing the extender mechanism. */
    public static final TunableNumber kHomingVoltage = TuningManager.register("Extender/HomingVoltage", -1.5);
    
    /** Maximum allowed voltage during the homing process. */
    public static final TunableNumber kMaxHomingVoltage = TuningManager.register("Extender/MaxHomingVoltage", 10.0);
    
    /** Maximum allowed difference in position between the leader and follower motors before an error is flagged, in inches. */
    public static final TunableNumber kMaxPositionDifference = TuningManager.register("Extender/MaxPositionDifference", 0.5);

    /** Proportional gain for synchronizing the extender's leader and follower motors. */
    public static final TunableNumber kSyncP = TuningManager.register("Extender/SyncP", 0.1);

    /** Tolerance for checking if the extender has reached its target position, in inches. */
    public static final TunableNumber kAtTargetTolerance = TuningManager.register("Extender/AtTargetTolerance", 0.03);

    /*
     ****** NON TUNABLE VARIABLES ******
     */

    /** Give up on homing after this long (seconds) instead of stalling into the hard stop. */
    public static final double kHomingTimeoutSeconds = 3.0;

    /** The target position for the extender when fully retracted, in inches. */
    public static final double kExtenderMotorIn = 0.0;
    
    /** The target position for the extender when fully extended, in inches. */
    public static final double kExtenderMotorOut = 12.0;    // -- PHYSICAL MATH --
    
    /** Gear ratio of the extender mechanism. A value of 1.0 indicates no gear reduction. */
    public static final double kGearRatio = 1.0; 

    // pully/sprocket circumference = Diameter * PI
    /** Pitch diameter of the pulley or sprocket used in the extender mechanism. */
    public static final double pitchDiameter = 1.25;
    
    /** Inches traveled per rotation of the extender motor, calculated from pitch diameter. */
    public static final double kInchesPerRotation = pitchDiameter * Math.PI;

    // factor: (1 / GearRatio) * InchesPerRotation
    /**
     * Conversion factor to translate motor rotations into inches of extension.
     * Calculated as (1.0 / kGearRatio) * kInchesPerRotation.
     */
    public static final double kPositionFactor = RobotUtils.calculateLinearFactor(kGearRatio, pitchDiameter);;
    public static final double kVelocityFactor = RobotUtils.toVelocityPerSecond(kPositionFactor);
    /**
     * Cruise velocity for the extender mechanism, used in motion profiling.
     * Calculated based on motor free speed, extender motor speed constant, and velocity conversion factor.
     */
    public static final double kExtenderCruiseVelocity = NeoMotorConstants.kFreeSpeedRpm * kExtenderMotorSpeed.get() * kVelocityFactor;
}
