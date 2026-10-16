package frc.robot.configs;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ExtenderConstants;
import frc.robot.constants.GlobalConstants;

/** SPARK MAX configs for the two-motor extender. */
public final class ExtenderConfigs {
    public static final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig followConfig = new SparkMaxConfig();
    /** Only turns soft limits off, so homing can reach the hard stop (nothing else is touched). */
    public static final SparkMaxConfig softLimitsOff = new SparkMaxConfig();
    /** Only turns soft limits back on after homing (keeps any gains tuned at runtime). */
    public static final SparkMaxConfig softLimitsOn = new SparkMaxConfig();

    static {
        leaderConfig.encoder
            .positionConversionFactor(ExtenderConstants.kPositionFactor)
            .velocityConversionFactor(ExtenderConstants.kVelocityFactor);
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ExtenderConstants.kP.get(), ExtenderConstants.kI.get(), ExtenderConstants.kD.get())
            .allowedClosedLoopError(ExtenderConstants.kAllowedClosedLoopError, ClosedLoopSlot.kSlot0);
        leaderConfig.closedLoop.feedForward
            .kV(ExtenderConstants.kV.get())
            .kS(ExtenderConstants.kS.get());
        leaderConfig.closedLoop.maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .cruiseVelocity(ExtenderConstants.kCruiseVelocity)
            .maxAcceleration(ExtenderConstants.kMaxAcceleration)
            .allowedProfileError(ExtenderConstants.kAllowedProfileError);
        // Soft limits relative to the boot/homed zero, so a bad command can't drive it into the frame.
        leaderConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(ExtenderConstants.kExtended + ExtenderConstants.kSoftLimitMargin)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(ExtenderConstants.kRetracted - ExtenderConstants.kSoftLimitMargin);
        leaderConfig
            .inverted(false)
            // Brake (Doyenne_2026 used coast) so the box doesn't drift when disabled.
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ExtenderConstants.kCurrentLimit)
            .voltageCompensation(GlobalConstants.kMedVoltageCompensation);

        // The follower mirrors the leader in hardware, mounted opposite so inverted. Set in the
        // persisted config, so a follower that browns out and reboots keeps following.
        followConfig.encoder
            .positionConversionFactor(ExtenderConstants.kPositionFactor)
            .velocityConversionFactor(ExtenderConstants.kVelocityFactor);
        followConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ExtenderConstants.kCurrentLimit)
            .follow(CanIdConstants.kExtenderMotor1, true);

        softLimitsOff.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);
        softLimitsOn.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);
    }

    private ExtenderConfigs() {}
}
