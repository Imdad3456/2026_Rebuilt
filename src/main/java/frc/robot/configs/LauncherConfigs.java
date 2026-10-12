package frc.robot.configs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.LauncherConstants;

/** Launcher flywheel: a Kraken/Falcon (TalonFX). Velocity is in flywheel rotations per second. */
public final class LauncherConfigs {
    public static final TalonFXConfiguration config = new TalonFXConfiguration();

    static {
        config.Slot0.kP = LauncherConstants.kP.get();
        config.Slot0.kI = LauncherConstants.kI.get();
        config.Slot0.kD = LauncherConstants.kD.get();
        config.Slot0.kV = LauncherConstants.kV.get();
        config.Slot0.kS = LauncherConstants.kS.get();

        config.Feedback.SensorToMechanismRatio = LauncherConstants.kGearRatio;

        config.CurrentLimits.SupplyCurrentLimit = LauncherConstants.kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Never drive the flywheel backwards: slowing from shot speed coasts down instead of
        // reverse-braking, which would pull a big current spike.
        config.Voltage.PeakReverseVoltage = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    private LauncherConfigs() {}
}
