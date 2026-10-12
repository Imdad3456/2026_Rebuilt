package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.GlobalConstants;

/** Rollers. Run open-loop (percent output), so no closed-loop gains. */
public final class RollerConfigs {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
        config.inverted(false);
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(GlobalConstants.kLowCurrentLimit);
    }

    private RollerConfigs() {}
}
