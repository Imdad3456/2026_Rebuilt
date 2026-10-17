package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.GlobalConstants;

/**
 * Front intake rollers. Runs open-loop (percent output), so no
 * closed-loop gains. Brake, 40A, voltage compensation from Doyenne_2026.
 */
public final class IntakeConfigs {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
        // Inverted: with false, the intake button spun it outward (confirmed on the robot).
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(GlobalConstants.kMediumCurrentLimit);
        config.voltageCompensation(GlobalConstants.kLowVoltageCompensation);
    }

    private IntakeConfigs() {}
}
