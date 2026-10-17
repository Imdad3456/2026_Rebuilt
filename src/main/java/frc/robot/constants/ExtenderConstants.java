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
    /**
     * Smart current limit per motor (amps), recommended by another team. Set here, not only in REV
     * Hardware Client: the code re-applies the full config at every boot and would overwrite it.
     */
    public static final int kCurrentLimit = 15;
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
    /**
     * Within this of the target the controller stops correcting. Without it, the 36:1 gearing's
     * slop makes the box overshoot and hunt in and out around the target (REV: "useful in
     * preventing oscillation around the setpoint"). Doyenne_2026 used 0.1 in.
     */
    public static final double kAllowedClosedLoopError = Units.inchesToMeters(0.25);

    // Positions
    public static final double kRetracted = 0.0;
    /**
     * Fully out: 11 3/4 in from the start position, the full physical travel, which puts the intake
     * at the right height above the floor (measured Oct 2026). The target sits at the hard stop;
     * kAllowedClosedLoopError (1/4 in) means the controller stops pushing once it's within 1/4 in,
     * and the stopped-short check catches it if the stop reads lower than that.
     */
    public static final double kExtended = Units.inchesToMeters(11.75);
    /**
     * Soft limits sit this far beyond the in/out targets. With the limit exactly on the target, any
     * tiny overshoot hits the limit, cuts the output, and the box bounces off it in and out.
     */
    public static final double kSoftLimitMargin = Units.inchesToMeters(0.5);

    /** Where the box goes while shooting (March 2026 "launch" position). */
    public static final double kLaunchPosition = Units.inchesToMeters(8.0);
    // Shake while shooting: from the launch position, pull in this far and back, repeatedly,
    // to loosen stuck balls.
    public static final double kAgitateDepth = Units.inchesToMeters(2.0);
    /**
     * Time for each in or out stroke (0.35 s = about 1.4 shakes per second). A full stop-to-stop
     * 2 in move takes about 0.45 s at the motion limits above; the box is still moving when it
     * reverses, so it still covers roughly the full depth. Faster shaking needs more acceleration.
     */
    public static final double kAgitateStrokeSeconds = 0.35;
    /** Only shake if the box is at least this far out (never jab into the retracted stop). */
    public static final double kAgitateMinPosition = Units.inchesToMeters(6.0);

    /** Manual mode: box speed (m/s) at full stick deflection. */
    public static final double kManualSpeed = 0.15;
    /** Manual mode: stick movement below this is ignored. */
    public static final double kManualDeadband = 0.1;

    // Safety
    /** Disagreement between the two motors' encoders that trips the skew fault (March 2026 tuning). */
    public static final double kMaxSkew = Units.inchesToMeters(1.5);

    /**
     * Encoder-reset detection: a position change faster than this (m/s) is not real motion.
     * Twice the physical top speed, so normal driving (even a slow loop) never trips it.
     */
    public static final double kMaxPlausibleSpeed = 2.0 * NeoMotorConstants.kFreeSpeedRpm * kVelocityFactor;
    /** After zeroing the encoders ourselves, ignore jumps this long while the SPARKs report it. */
    public static final double kZeroSettleSeconds = 0.25;
    /** Plus this much (m), so small changes never count. */
    public static final double kJumpMargin = Units.inchesToMeters(1.0);

    // Stopped short: pushing but not moving, short of the target (target past a hard stop, or the
    // box is jammed). After this long, hold where it is instead of pushing the NEOs at stall.
    public static final double kStallSeconds = 0.5;
    /** Slower than this (m/s, ~0.2 in/s) counts as not moving. */
    public static final double kStallSpeed = 0.005;
    /** More than this motor output (fraction, ~0.3 V) counts as pushing. */
    public static final double kStallMinOutput = 0.03;

    /**
     * The two motors' currents must differ by more than kImbalanceAmps for this long before the
     * imbalance trip fires. A single sample tripped it 0.2 s into auto in Q2 at NCFAY (leader
     * 60.7 A, follower 0 A), killing the box for the whole match; a startup surge is shorter.
     */
    public static final double kImbalanceSeconds = 0.3;
    /**
     * Current difference (amps) that counts as an imbalance. Was 60 A (kHighCurrentLimit), which
     * can't happen with a 15 A limit: one side stalled at the limit while the other draws nothing.
     */
    public static final double kImbalanceAmps = 12.0;

    // Homing: drive gently into the retracted hard stop until both motors stall.
    public static final double kHomingVoltage = -2.4; // about 20% output
    /**
     * Both motors above this current (amps) for kHomingStallSeconds = at the hard stop. Must stay
     * below kCurrentLimit: the limit caps the current, so a threshold at the limit is never reached.
     */
    public static final double kHomingStallCurrent = 10.0;
    public static final double kHomingStallSeconds = 0.1;
    public static final double kHomingTimeoutSeconds = 3.0;

    private ExtenderConstants() {}
}
