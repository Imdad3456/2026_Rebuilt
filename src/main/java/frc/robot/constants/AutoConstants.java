package frc.robot.constants;

import frc.robot.util.TunableNumber;
import frc.robot.util.TuningManager;

/** Autonomous settings. */
public final class AutoConstants {
    /** PathPlanner path-following P gains: translation (per meter of error) and rotation (per radian). */
    public static final double kPXController = 1;
    public static final double kPThetaController = 1;

    /** "Simple Auto": drive forward at this fraction of max speed for this long. */
    public static final double kExampleAutoSpeed = 0.5;
    public static final double kExampleAutoTimeout = 3.0;

    /**
     * Shot RPM for the "Shoot"/"SpinUp" named commands (no slider in auto). Starts mid-range of the
     * shooter slider; set it from test shots at your auto shooting spot.
     */
    public static final TunableNumber kAutoShotRPM = TuningManager.register("Auto/ShotRPM", 2400);
    /** How long the "Shoot" named command runs before ending. */
    public static final double kShootSeconds = 2.0;

    private AutoConstants() {}
}
