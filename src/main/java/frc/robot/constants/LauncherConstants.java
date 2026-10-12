package frc.robot.constants;

import frc.robot.util.TunableNumber;
import frc.robot.util.TuningManager;

/**
 * Launcher (Kraken/Falcon TalonFX) settings. Speeds are in RPM for readability and converted to
 * rotations per second for the TalonFX. Gains and speeds come from Doyenne_2026.
 */
public final class LauncherConstants {
    // Speeds (RPM)
    public static final TunableNumber kIdleRPM = TuningManager.register("Launcher/IdleRPM", 480);
    /** Within this many RPM of the target counts as "at speed". */
    public static final TunableNumber kToleranceRPM = TuningManager.register("Launcher/ToleranceRPM", 180);
    /** Shooter slider all the way down. */
    public static final TunableNumber kMinShotRPM = TuningManager.register("Launcher/MinShotRPM", 1500);
    /** Shooter slider all the way up. */
    public static final TunableNumber kMaxShotRPM = TuningManager.register("Launcher/MaxShotRPM", 3300);

    // TalonFX Slot 0 velocity gains (per rotation per second)
    public static final TunableNumber kP = TuningManager.register("Launcher/kP", 0.11);
    public static final TunableNumber kI = TuningManager.register("Launcher/kI", 0.0);
    public static final TunableNumber kD = TuningManager.register("Launcher/kD", 0.01);
    public static final TunableNumber kV = TuningManager.register("Launcher/kV", 0.12);
    public static final TunableNumber kS = TuningManager.register("Launcher/kS", 0.15);

    // Limelight distance -> RPM (not bound to a button yet)
    public static final TunableNumber kBaseRPM = TuningManager.register("Launcher/BaseRPM", 15);
    public static final TunableNumber kRPMPerInch = TuningManager.register("Launcher/RPMPerInch", 15);
    public static final double kAprilTagHeight = 44.25; // inches
    public static final double kCameraHeight = 12.0; // inches
    public static final double kMountAngle = 15.0; // degrees

    // Hardware
    public static final double kGearRatio = 1.0;
    public static final double kSupplyCurrentLimit = 40.0;

    private LauncherConstants() {}
}
