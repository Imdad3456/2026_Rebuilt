package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.constants.GlobalConstants;

/**
 * Front intake rollers (SPARK Flex on CAN 10). Runs open-loop (percent output), so no
 * closed-loop gains. Brake, 40A, voltage compensation from Doyenne_2026.
 */
public final class IntakeConfigs {
    public static final SparkFlexConfig config = new SparkFlexConfig();

    static {
        // Inverted: with false, the intake button spun it outward (confirmed on the robot).
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(GlobalConstants.kMediumCurrentLimit);
        config.voltageCompensation(GlobalConstants.kLowVoltageCompensation);
    }

    private IntakeConfigs() {}
}
