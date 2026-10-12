package frc.robot.constants;

import frc.robot.util.TunableNumber;
import frc.robot.util.TuningManager;

/** Feeder settings. */
public final class FeederConstants {
    /** Feeder duty cycle (0..1). */
    public static final TunableNumber kFeederSpeed = TuningManager.register("Feeder/Speed", 0.5);

    private FeederConstants() {}
}
