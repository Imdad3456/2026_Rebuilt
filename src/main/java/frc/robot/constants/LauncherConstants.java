package frc.robot.constants;

import frc.robot.util.TunableNumber;
import frc.robot.util.TuningManager;

/**
 * Launcher (Kraken/Falcon TalonFX) settings. Speeds are in RPM for readability and converted to
 * rotations per second for the TalonFX. Gains from Doyenne_2026; speeds from the March 2026
 * competition tuning (26_Rebuilt fine-tuning: idle 10, slider 45-75, tolerance 3 rotations/s).
 */
public final class LauncherConstants {
    // Speeds (RPM)
    public static final TunableNumber kIdleRPM = TuningManager.register("Launcher/IdleRPM", 600);
    /** Within this many RPM of the target counts as "at speed". */
    public static final TunableNumber kToleranceRPM = TuningManager.register("Launcher/ToleranceRPM", 180);
    /** How long the flywheel must stay at speed before (re)starting the feeder. */
    public static final double kReadySettleSeconds = 0.1;
    /**
     * If a shot hasn't reached the target (within tolerance) after this long, feed anyway once it's
     * at least kFallbackFraction of the target. With a sagging battery the flywheel can't always
     * reach the top of the slider range (4500 RPM needs ~9 V), and the feeder used to wait forever.
     */
    public static final double kSpinUpTimeoutSeconds = 1.0;
    public static final double kFallbackFraction = 0.85;
    /** Shooter slider all the way down. */
    public static final TunableNumber kMinShotRPM = TuningManager.register("Launcher/MinShotRPM", 2700);
    /** Shooter slider all the way up. */
    public static final TunableNumber kMaxShotRPM = TuningManager.register("Launcher/MaxShotRPM", 4500);

    // TalonFX Slot 0 velocity gains (per rotation per second)
    public static final TunableNumber kP = TuningManager.register("Launcher/kP", 0.11);
    public static final TunableNumber kI = TuningManager.register("Launcher/kI", 0.0);
    public static final TunableNumber kD = TuningManager.register("Launcher/kD", 0.01);
    public static final TunableNumber kV = TuningManager.register("Launcher/kV", 0.12);
    public static final TunableNumber kS = TuningManager.register("Launcher/kS", 0.15);

    // Hardware
    public static final double kGearRatio = 1.0;
    public static final double kSupplyCurrentLimit = 40.0;

    private LauncherConstants() {}
}
