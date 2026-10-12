package frc.robot.configs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.constants.NeoSwerveModuleConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.GlobalConstants;

public final class DriveConfigs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        public static final IdleMode driveIdle = IdleMode.kBrake; // brake: robot stops when released (Doyenne_2026)
        public static final IdleMode turnIdle = IdleMode.kBrake; //

        private static final int kDrivingCurrentLimit = 50;

        static {
            // --- Driving Motor Configuration ---
            drivingConfig
                    .idleMode(driveIdle) //
                    // 50A is REV's recommended MAXSwerve drive limit. 60A x 4 drive motors
                    // under full field load browns out the robot (radio + CAN drop out).
                    .smartCurrentLimit(kDrivingCurrentLimit) //
                    .voltageCompensation(GlobalConstants.kMedVoltageCompensation);

            drivingConfig.encoder
                    .positionConversionFactor(NeoSwerveModuleConstants.kDriveEncoderPositionFactor) //
                    .velocityConversionFactor(NeoSwerveModuleConstants.kDriveEncoderVelocityFactor); //

            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //
                    .pid(DriveConstants.kDriveP.get(), DriveConstants.kDriveI.get(), DriveConstants.kDriveD.get()) //
                    .outputRange(-1, 1) //
                    // Without a feedforward the velocity loop is P-only and never reaches the
                    // requested speed (sluggish, inconsistent driving).
                    .feedForward.kV(DriveConstants.kDriveV.get()).kS(DriveConstants.kDriveS.get());

            // --- Turning Motor Configuration ---
            turningConfig
                    .idleMode(turnIdle) //
                    .smartCurrentLimit(GlobalConstants.kLowCurrentLimit) //
                    .voltageCompensation(GlobalConstants.kLowVoltageCompensation);

            turningConfig.absoluteEncoder
                    .inverted(true) //
                    .positionConversionFactor(NeoSwerveModuleConstants.kTurningEncoderPositionFactor) //
                    .velocityConversionFactor(NeoSwerveModuleConstants.kTurningEncoderVelocityFactor); //

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //
                    .pid(DriveConstants.kTurnP.get(), DriveConstants.kTurnI.get(), DriveConstants.kTurnD.get()) //
                    .outputRange(-1, 1) //
                    .positionWrappingEnabled(true) //
                    .positionWrappingInputRange(0, NeoSwerveModuleConstants.kTurningEncoderPositionFactor); //
        }
    }
}