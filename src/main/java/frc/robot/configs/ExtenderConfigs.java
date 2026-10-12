package frc.robot.configs;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ExtenderConstants;
import frc.robot.constants.GlobalConstants;

/**
 * Configuration class for the Extender subsystem's SparkMax motor controllers.
 * Defines various settings for PID, feedforward, motion profiling, soft limits,
 * and current limits for leader, follower, and homing operational modes.
 */
public class ExtenderConfigs {
    /** Configuration for the primary extender motor (leader). */
    public static final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    /** Configuration for the secondary extender motor (follower). */
    public static final SparkMaxConfig followConfig = new SparkMaxConfig();

    /** Configuration specifically for the homing sequence of the extender. */
    public static final SparkMaxConfig homingConfig = new SparkMaxConfig();
    // Indicates if the leader motor's direction needs to be inverted.
    // This value will be applied to leaderConfig.
    private static boolean leaderInverted = true;

    static {
        // Configure the leader motor's encoder for position and velocity conversion
        // factors.
        leaderConfig.encoder
                .positionConversionFactor(ExtenderConstants.kPositionFactor)
                .velocityConversionFactor(ExtenderConstants.kVelocityFactor);

        // Configure PID constants for closed-loop control.
        leaderConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ExtenderConstants.kExtenderP.get(), ExtenderConstants.kExtenderI.get(), ExtenderConstants.kExtenderD.get());

        // Configure Feedforward constants to improve control accuracy.
        leaderConfig.closedLoop.feedForward
                .kV(ExtenderConstants.kExtenderFF.get())
                .kS(ExtenderConstants.kExtenderStatic.get());

        // Configure motion profiling parameters for smooth movement.
        leaderConfig.closedLoop.maxMotion
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .cruiseVelocity(ExtenderConstants.kExtenderCruiseVelocity)
                .maxAcceleration(ExtenderConstants.kExtAcceleration.get())
                .allowedProfileError(ExtenderConstants.kExtenderAllowedError.get());

        // Configure soft limits to prevent physical over-extension or retraction.
        leaderConfig.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(ExtenderConstants.kExtenderMotorOut)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(ExtenderConstants.kExtenderMotorIn);

        // Apply the inverted setting to the leader motor.
        leaderConfig.inverted(leaderInverted);
        // Set a smart current limit to protect the motor and battery.
        leaderConfig.smartCurrentLimit(GlobalConstants.kMediumCurrentLimit);
        // Set the idle mode to brake when the motor is not actively driven.
        leaderConfig.idleMode(IdleMode.kBrake);

        // Apply the leader's configuration to the follower motor as a base.
        // followConfig.apply(leaderConfig);

        followConfig.encoder
                .positionConversionFactor(ExtenderConstants.kPositionFactor)
                .velocityConversionFactor(ExtenderConstants.kVelocityFactor);

        followConfig.inverted(!leaderInverted); // Match physical direction
        followConfig.idleMode(IdleMode.kBrake);
        followConfig.smartCurrentLimit(GlobalConstants.kMediumCurrentLimit);

        // NO PID, NO FF, NO MOTION MAGIC. JUST FOLLOW.
        // Set here (not at runtime) so it is persisted to flash: if the follower
        // SPARK browns out and reboots mid-match it comes back still following.
        followConfig.follow(CanIdConstants.kExtenderMotor1, true);

        // Apply the leader's configuration to the homing motor config as a base.
        homingConfig.apply(leaderConfig);

        // Disable soft limits for homing, allowing the extender to reach a mechanical
        // stop.
        homingConfig.softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);
    }
}
