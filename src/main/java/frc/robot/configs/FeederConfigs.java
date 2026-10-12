package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.GlobalConstants;

/** Feeder. Runs open-loop. 40A: at 20A it stalled as soon as a ball got pinched against the flywheel. */
public final class FeederConfigs {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(GlobalConstants.kMediumCurrentLimit);
    }

    private FeederConfigs() {}
}
