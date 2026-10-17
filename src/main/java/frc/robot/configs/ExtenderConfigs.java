package frc.robot.configs;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.ExtenderConstants;
import frc.robot.constants.GlobalConstants;

/**
 * SPARK MAX configs for the two-motor extender. Each motor runs its own closed loop on its own
 * encoder toward the same target (no follower mode), so if one side's CAN or power flickers the
 * other side still drives and holds its position.
 */
public final class ExtenderConfigs {
    /** CAN 20. */
    public static final SparkMaxConfig motor1Config = motorConfig(false);
    /** CAN 21, mounted opposite so inverted. Inverting also flips its encoder, so both read + out. */
    public static final SparkMaxConfig motor2Config = motorConfig(true);
    /** Only turns soft limits off, so homing can reach the hard stop (nothing else is touched). */
    public static final SparkMaxConfig softLimitsOff = new SparkMaxConfig();
    /** Only turns soft limits back on after homing (keeps any gains tuned at runtime). */
    public static final SparkMaxConfig softLimitsOn = new SparkMaxConfig();

    private static SparkMaxConfig motorConfig(boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder
            .positionConversionFactor(ExtenderConstants.kPositionFactor)
            .velocityConversionFactor(ExtenderConstants.kVelocityFactor);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ExtenderConstants.kP.get(), ExtenderConstants.kI.get(), ExtenderConstants.kD.get())
            .allowedClosedLoopError(ExtenderConstants.kAllowedClosedLoopError, ClosedLoopSlot.kSlot0);
        config.closedLoop.feedForward
            .kV(ExtenderConstants.kV.get())
            .kS(ExtenderConstants.kS.get());
        config.closedLoop.maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .cruiseVelocity(ExtenderConstants.kCruiseVelocity)
            .maxAcceleration(ExtenderConstants.kMaxAcceleration)
            .allowedProfileError(ExtenderConstants.kAllowedProfileError);
        // Soft limits relative to the boot/homed zero, so a bad command can't drive it into the frame.
        config.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(ExtenderConstants.kExtended + ExtenderConstants.kSoftLimitMargin)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(ExtenderConstants.kRetracted - ExtenderConstants.kSoftLimitMargin);
        config
            .inverted(inverted)
            // Brake (Doyenne_2026 used coast) so the box doesn't drift when disabled.
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ExtenderConstants.kCurrentLimit)
            .voltageCompensation(GlobalConstants.kMedVoltageCompensation);
        return config;
    }

    static {
        softLimitsOff.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);
        softLimitsOn.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);
    }

    private ExtenderConfigs() {}
}
