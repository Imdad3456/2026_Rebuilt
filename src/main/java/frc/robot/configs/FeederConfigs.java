// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configs;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.FeederConstants;
import frc.robot.constants.GlobalConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class FeederConfigs {
    public static final SparkMaxConfig config = new SparkMaxConfig();
    
    static {
        config.encoder
            .positionConversionFactor(FeederConstants.kPositionFactor)
            .velocityConversionFactor(FeederConstants.kPositionFactor);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        config.closedLoop.feedForward
            .kS(FeederConstants.kFeederStatic.get())
            .kV(FeederConstants.kFeederFF.get());

        config.inverted(false);        
        config.idleMode(IdleMode.kBrake); 
        // 20A stalled the feeder as soon as a ball got pinched against the flywheel.
        config.smartCurrentLimit(GlobalConstants.kMediumCurrentLimit);
    }
}
