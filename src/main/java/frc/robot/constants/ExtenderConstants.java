package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.util.RobotUtils;
import frc.robot.util.TunableNumber;
import frc.robot.util.TuningManager;

/**
 * Extender ("the box"). Positions are in meters along the travel. Mechanics and gains come from
 * Doyenne_2026 (the current robot).
 */
public final class ExtenderConstants {
    // Hardware (declared first: the gains below are computed from it, and Java initializes
    // static fields top to bottom)
    public static final int kCurrentLimit = GlobalConstants.kMediumCurrentLimit;
    public static final double kGearRatio = 36.0;
    public static final double kPulleyPitchDiameterInches = 2.10;
    /** Meters of travel per motor rotation. */
    public static final double kPositionFactor = RobotUtils.calculateLinearFactor(kGearRatio,
        Units.inchesToMeters(kPulleyPitchDiameterInches));
    /** Meters per second per motor RPM. */
    public static final double kVelocityFactor = RobotUtils.toVelocityPerSecond(kPositionFactor);


    // Closed-loop gains (MAXMotion position, per meter)
    public static final TunableNumber kP = TuningManager.register("Extender/kP", 0.8);
    public static final TunableNumber kI = TuningManager.register("Extender/kI", 0.0);
    public static final TunableNumber kD = TuningManager.register("Extender/kD", 0.0);
    /**
     * Volts per (m/s): 12 V divided by the box's top speed (from the gearing above).
     * Doyenne_2026 used 0.8, about 34x too low, so the controller couldn't follow the profile.
     */
    public static final TunableNumber kV = TuningManager.register("Extender/kV",
        12.0 / (NeoMotorConstants.kFreeSpeedRpm * kVelocityFactor));
    public static final TunableNumber kS = TuningManager.register("Extender/kS", 0.6);

    /**
     * Motion limits. The box's physical top speed is only about 0.44 m/s (17 in/s) at 12 V.
     * Doyenne_2026 asked for 1.5 m/s: the profile ran far ahead of the box, which then hit the end
     * at full power with no slow-down, which is likely why it was slamming. Keep the cruise speed
     * well under the top speed so the profile can actually be followed and decelerated.
     */
    public static final double kCruiseVelocity = 0.25; // m/s (~10 in/s, full travel in ~1.3 s)
    public static final double kMaxAcceleration = 1.0; // m/s^2

    public static final double kAllowedProfileError = Units.inchesToMeters(0.5);
    /** Within this of the target counts as "at target". */
    public static final double kAtTargetTolerance = Units.inchesToMeters(0.1);

    // Positions
    public static final double kRetracted = 0.0;
    public static final double kExtended = Units.inchesToMeters(12.0);

    // Safety
    /** Leader/follower disagreement that trips the skew fault (Doyenne_2026 used 2 in). */
    public static final double kMaxSkew = Units.inchesToMeters(2.0);

    /**
     * Encoder-reset detection: a position change faster than this (m/s) is not real motion.
     * Twice the physical top speed, so normal driving (even a slow loop) never trips it.
     */
    public static final double kMaxPlausibleSpeed = 2.0 * NeoMotorConstants.kFreeSpeedRpm * kVelocityFactor;
    /** Plus this much (m), so small changes never count. */
    public static final double kJumpMargin = Units.inchesToMeters(1.0);

    // Homing: drive gently into the retracted hard stop until both motors stall.
    public static final double kHomingVoltage = -2.4; // about 20% output
    /** Both motors above this current (amps) for kHomingStallSeconds = at the hard stop. */
    public static final double kHomingStallCurrent = 15.0;
    public static final double kHomingStallSeconds = 0.1;
    public static final double kHomingTimeoutSeconds = 3.0;

    private ExtenderConstants() {}
}
