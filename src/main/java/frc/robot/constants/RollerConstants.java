package frc.robot.constants;

import frc.robot.util.TunableNumber;
import frc.robot.util.TuningManager;

/** Roller settings. */
public final class RollerConstants {
    /** Roller duty cycle (0..1). */
    public static final TunableNumber kRollerTargetSpeed = TuningManager.register("Roller/TargetSpeed", 0.05);

    private RollerConstants() {}
}
