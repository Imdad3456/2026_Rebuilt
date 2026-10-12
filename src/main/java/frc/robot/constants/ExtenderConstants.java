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
    // Closed-loop gains (MAXMotion position, per meter)
    public static final TunableNumber kP = TuningManager.register("Extender/kP", 0.8);
    public static final TunableNumber kI = TuningManager.register("Extender/kI", 0.0);
    public static final TunableNumber kD = TuningManager.register("Extender/kD", 0.0);
    public static final TunableNumber kV = TuningManager.register("Extender/kV", 0.8);
    public static final TunableNumber kS = TuningManager.register("Extender/kS", 0.6);

    /**
     * Motion limits. Deliberately gentler than Doyenne_2026 (1.5 m/s, 12 m/s^2) because the box
     * slamming in and out was damaging it. Raise these after testing if it's too slow.
     */
    public static final double kCruiseVelocity = 0.5; // m/s (~20 in/s)
    public static final double kMaxAcceleration = 2.0; // m/s^2

    public static final double kAllowedProfileError = Units.inchesToMeters(0.5);
    /** Within this of the target counts as "at target". */
    public static final double kAtTargetTolerance = Units.inchesToMeters(0.1);

    // Positions
    public static final double kRetracted = 0.0;
    public static final double kExtended = Units.inchesToMeters(12.0);

    // Safety
    /** Leader/follower disagreement that trips the skew fault (Doyenne_2026 used 2 in). */
    public static final double kMaxSkew = Units.inchesToMeters(2.0);

    // Homing: drive gently into the retracted hard stop until both motors stall.
    public static final double kHomingVoltage = -2.4; // about 20% output
    /** Both motors above this current (amps) for kHomingStallSeconds = at the hard stop. */
    public static final double kHomingStallCurrent = 15.0;
    public static final double kHomingStallSeconds = 0.1;
    public static final double kHomingTimeoutSeconds = 3.0;

    // Hardware
    public static final int kCurrentLimit = GlobalConstants.kMediumCurrentLimit;
    public static final double kGearRatio = 36.0;
    public static final double kPulleyPitchDiameterInches = 2.10;
    /** Meters of travel per motor rotation. */
    public static final double kPositionFactor = RobotUtils.calculateLinearFactor(kGearRatio,
        Units.inchesToMeters(kPulleyPitchDiameterInches));
    /** Meters per second per motor RPM. */
    public static final double kVelocityFactor = RobotUtils.toVelocityPerSecond(kPositionFactor);

    private ExtenderConstants() {}
}
